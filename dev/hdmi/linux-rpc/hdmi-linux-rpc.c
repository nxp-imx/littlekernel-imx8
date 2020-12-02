/*
 * The Clear BSD License
 * Copyright 2019-2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "platform/interrupts.h"
#include <kernel/spinlock.h>
#include <dev/class/hdmi.h>
#include <ivshmem-rpc.h>
#include <string.h>
#include <delay.h>
#include "debug.h"
#include "err.h"
#include <lib/appargs.h>

#include <malloc.h>


IVSHM_RPC_CALLBACK(RPC_HDMI_HANDLE_EVENT_ID, rpc_hdmi_event_cb);

static struct rpc_client_callback *rpc_hdmi_callbacks[] = {
    &rpc_cb_rpc_hdmi_event_cb,
    NULL,
};

struct rpc_hdmi_s_format_s {
    hdmi_audio_direction_t direction;
    hdmi_audio_fmt_t fmt;
};

struct rpc_hdmi_s_if_s {
    hdmi_audio_direction_t direction;
    hdmi_audio_if_t in;
    hdmi_audio_if_t out;
};

struct rpc_hdmi_s_pkt_s {
    hdmi_audio_direction_t direction;
    hdmi_audio_pkt_t pkt;
};


struct linux_rpc_hdmi_state {
    struct device *dev;
    spin_lock_t cb_lock;
    hdmi_cb_t cb;
    void * cb_cookie;
    struct ivshm_rpc_service *rpc_service;
};

static status_t linux_rpc_hdmi_init(struct device *dev)
{
    struct linux_rpc_hdmi_state *state = malloc(sizeof(struct linux_rpc_hdmi_state));
    uint32_t service_id;
    status_t ret;

    memset(state, 0, sizeof(struct linux_rpc_hdmi_state));

    struct device *rpc_dev = of_device_lookup_device(dev, "rpc,service");
    if (rpc_dev == NULL) {
        free(state);
        printlk(LK_ERR, "%s:%d: Invalid rpc phandle\n", __PRETTY_FUNCTION__,
                __LINE__);
        return ERR_INVALID_ARGS;
    }

    if (of_device_get_int32(rpc_dev, "id", &service_id)) {
        free(state);
        printlk(LK_ERR, "%s:%d: Missing id property in device node. Failing..,\n",
                __PRETTY_FUNCTION__, __LINE__);
        return ERR_INVALID_ARGS;
    }

    state->rpc_service = ivshm_get_rpc_service(service_id);
    ASSERT(state->rpc_service);

    /* Register local function which may be called from second OS */
    ivshm_rpc_register_client(state->rpc_service, rpc_hdmi_callbacks);
    ivshm_rpc_set_data(state->rpc_service, dev);

    dev->state = state;
    state->dev = dev;

    int tries = 20;
    while (tries-- &&
           (ret = ivshm_rpc_cmd(state->rpc_service, RPC_HDMI_INIT_ID)) != 0) {
        printlk(LK_INFO, "%s:%d: RPC service not ready. Retry %d ...\n",
                __PRETTY_FUNCTION__, __LINE__, tries);
        thread_sleep(30); /* wait 30ms */
    }

    if (ret)
        goto error;

    return ret;

error:
    free(state);
    return ret;
}

static status_t linux_rpc_hdmi_open(struct device *dev)
{
    status_t ret;
    struct linux_rpc_hdmi_state *state = dev->state;

    printlk(LK_INFO, "Open\n");

    ret = ivshm_rpc_cmd(state->rpc_service, RPC_HDMI_OPEN_ID);

    return ret;
}

static status_t linux_rpc_hdmi_close(struct device *dev)
{
    status_t ret;
    struct linux_rpc_hdmi_state *state = dev->state;

    printlk(LK_INFO, "Close\n");

    ret = ivshm_rpc_cmd(state->rpc_service, RPC_HDMI_CLOSE_ID);

    return ret;
}

static status_t linux_rpc_hdmi_get_capabilities(const struct device * dev, uint32_t *capabilities)
{
    status_t ret;
    struct linux_rpc_hdmi_state *state = dev->state;

    printlk(LK_INFO, "Get capabilities\n");

    ret = ivshm_rpc_cmd_get(state->rpc_service, RPC_HDMI_G_CAP_ID,
                                                capabilities, sizeof(uint32_t));

    return ret;
};

static status_t linux_rpc_hdmi_set_callback(struct device *dev, hdmi_cb_t cb, void *cookie)
{
    status_t ret = 0;
    struct linux_rpc_hdmi_state *state = dev->state;

    printlk(LK_INFO, "Set Linux RPC HDMI callback/cookie: [%p/%p]\n", cb, cookie);
    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->cb_lock, lock_state);

    state->cb = cb;
    state->cb_cookie = cookie;

    spin_unlock_irqrestore(&state->cb_lock, lock_state);

    return ret;
};

static status_t linux_rpc_hdmi_set_audio_format(struct device *dev, hdmi_audio_direction_t direction, hdmi_audio_fmt_t fmt)
{
    status_t ret;
    struct linux_rpc_hdmi_state *state = dev->state;
    struct rpc_hdmi_s_format_s format = {
        .direction = direction,
        .fmt = fmt,
    };

    printlk(LK_INFO, "Set audio format\n");

    ret = ivshm_rpc_cmd_set(state->rpc_service, RPC_HDMI_S_FORMAT_ID,
                            &format, sizeof(struct rpc_hdmi_s_format_s));

    return ret;
}

static status_t linux_rpc_hdmi_set_audio_packet(struct device *dev, hdmi_audio_direction_t direction, hdmi_audio_pkt_t pkt)
{
    status_t ret;
    struct linux_rpc_hdmi_state *state = dev->state;
    struct rpc_hdmi_s_pkt_s _pkt = {
        .direction = direction,
        .pkt = pkt,
    };

    printlk(LK_INFO, "Set audio pkt\n");

    ret = ivshm_rpc_cmd_set(state->rpc_service, RPC_HDMI_S_PKT_ID,
                                &_pkt, sizeof(struct rpc_hdmi_s_pkt_s));

    return ret;
};

static status_t linux_rpc_hdmi_set_audio_interface(struct device *dev,
                                    hdmi_audio_direction_t direction,
                                    hdmi_audio_if_t in, hdmi_audio_if_t out)
{
    status_t ret;
    struct linux_rpc_hdmi_state *state = dev->state;

    printlk(LK_INFO, "Set audio interface\n");

    if ((direction != HDMI_AUDIO_SINK) && (direction != HDMI_AUDIO_SOURCE)) {
        return ERR_INVALID_ARGS;
    }
    struct rpc_hdmi_s_if_s interface = {
        .direction = direction,
        .in = in,
        .out = out,
    };

    ret = ivshm_rpc_cmd_set(state->rpc_service, RPC_HDMI_S_IF_ID,
                        &interface, sizeof(struct rpc_hdmi_s_if_s));

    return ret;
};

static status_t linux_rpc_hdmi_get_channel_status(struct device *dev, hdmi_channel_status_t *channel_status)
{
    status_t ret;
    struct linux_rpc_hdmi_state *state = dev->state;

    printlk(LK_INFO, "Get audio Channel status\n");

    ret = ivshm_rpc_cmd_get(state->rpc_service, RPC_HDMI_G_CS_ID,
                        channel_status, sizeof(hdmi_channel_status_t));

    return ret;
};

static status_t linux_rpc_hdmi_get_audio_infoframe_pkt(struct device *dev,
                                                hdmi_audio_infoframe_pkt_t *pkt)
{
    status_t ret;
    struct linux_rpc_hdmi_state *state = dev->state;

    printlk(LK_INFO, "Get audio infoframe packet\n");

    ret = ivshm_rpc_cmd_get(state->rpc_service, RPC_HDMI_G_INFOFRAME_ID,
                        pkt, sizeof(hdmi_audio_infoframe_pkt_t));

    return ret;
}

static status_t linux_rpc_hdmi_get_audio_custom_fmt_layout(
                            struct device *dev,
                            iec60958_custom_fmt_layout_t *layout)
{
    status_t ret;
    struct linux_rpc_hdmi_state *state = dev->state;

    printlk(LK_INFO, "Get audio custom format\n");

    ret = ivshm_rpc_cmd_get(state->rpc_service, RPC_HDMI_G_CUSTOM_FMT_ID,
                        layout,
                        sizeof(iec60958_custom_fmt_layout_t));

    return ret;
}

static status_t linux_rpc_hdmi_get_audio_pkt_type(struct device *dev,
                                                        hdmi_audio_pkt_t *pkt)
{
    status_t ret;
    struct linux_rpc_hdmi_state *state = dev->state;

    printlk(LK_INFO, "Get audio pkt type\n");

    ret = ivshm_rpc_cmd_get(state->rpc_service, RPC_HDMI_G_PKT_TYPE_ID,
                        pkt,
                        sizeof(hdmi_audio_pkt_t));

    return ret;
}

static status_t linux_rpc_hdmi_get_audio_pkt_layout(struct device *dev, hdmi_audio_pkt_layout_t *layout)
{
    status_t ret;
    struct linux_rpc_hdmi_state *state = dev->state;

    printlk(LK_INFO, "Get audio pkt layout\n");

    ret = ivshm_rpc_cmd_get(state->rpc_service, RPC_HDMI_G_PKT_LAYOUT_ID,
                        layout,
                        sizeof(hdmi_audio_pkt_layout_t));

    return ret;
}

/* RPC callback function */
static int rpc_hdmi_event_cb(struct ivshm_rpc_service *service,
                             struct rpc_client_callback *cb,
                             void *payload)
{
    struct device *dev = ivshm_rpc_get_data(service);
    struct linux_rpc_hdmi_state *state = dev->state;
    hdmi_cb_evt_t evt = *(hdmi_cb_evt_t *)payload;
    void *param = payload + sizeof(hdmi_cb_evt_t);

    switch (evt) {
    case HDMI_EVENT_AUDIO_SAMPLE_RATE: {
        uint32_t *sample_rate = (uint32_t *)param;

        if (state && state->cb)
            state->cb(HDMI_EVENT_AUDIO_SAMPLE_RATE,
                      sample_rate, state->cb_cookie);

        break;
    }

    case HDMI_EVENT_AUDIO_STREAM_TYPE: {
        hdmi_audio_pkt_t *pkt = (hdmi_audio_pkt_t *)param;

        if (state && state->cb)
            state->cb(HDMI_EVENT_AUDIO_STREAM_TYPE, pkt, state->cb_cookie);

        break;
    }
    case HDMI_EVENT_AUDIO_INFOFRAME: {
        hdmi_audio_infoframe_pkt_t *pkt = (hdmi_audio_infoframe_pkt_t *)param;

        if (state && state->cb)
            state->cb(HDMI_EVENT_AUDIO_INFOFRAME, pkt, state->cb_cookie);

        break;
    }

    case HDMI_EVENT_AUDIO_CHANNEL_STATUS: {
        hdmi_channel_status_t *cs = (hdmi_channel_status_t *)param;

        if (state && state->cb)
            state->cb(HDMI_EVENT_AUDIO_CHANNEL_STATUS, cs, state->cb_cookie);

        break;
    }

    case HDMI_EVENT_AUDIO_LAYOUT_CHANGE: {
        hdmi_audio_pkt_layout_t *layout = (hdmi_audio_pkt_layout_t *)param;

        if (state && state->cb)
            state->cb(HDMI_EVENT_AUDIO_LAYOUT_CHANGE, layout, state->cb_cookie);

        break;
    }

    case HDMI_EVENT_AUDIO_LINK: {
        bool *is_connected = (bool *)param;

        if (state && state->cb)
            state->cb(HDMI_EVENT_AUDIO_LINK, is_connected, state->cb_cookie);
        break;
    }

    case HDMI_EVENT_AUDIO_MCLK:
    default:
        printlk(LK_ERR, "%s:%d: Unsupported event %d\n", __PRETTY_FUNCTION__,
                __LINE__, evt);
	return ERR_IO;
    }

    return 0;
}

static struct device_class linux_rpc_hdmi_device_class = {
    .name = "hdmi",
};
static struct hdmi_ops the_ops = {
    .std = {
        .device_class = &linux_rpc_hdmi_device_class,
        .init = linux_rpc_hdmi_init,
    },
    .open = linux_rpc_hdmi_open,
    .close = linux_rpc_hdmi_close,
    .get_capabilities = linux_rpc_hdmi_get_capabilities,
    .set_callback = linux_rpc_hdmi_set_callback,
    .set_audio_format = linux_rpc_hdmi_set_audio_format,
    .set_audio_packet = linux_rpc_hdmi_set_audio_packet,
    .set_audio_interface = linux_rpc_hdmi_set_audio_interface,
    .get_channel_status = linux_rpc_hdmi_get_channel_status,
    .get_audio_infoframe_pkt = linux_rpc_hdmi_get_audio_infoframe_pkt,
    .get_audio_custom_fmt_layout = linux_rpc_hdmi_get_audio_custom_fmt_layout,
    .get_audio_pkt_type = linux_rpc_hdmi_get_audio_pkt_type,
    .get_audio_pkt_layout = linux_rpc_hdmi_get_audio_pkt_layout,
};

DRIVER_EXPORT_WITH_LVL(linux_rpc_hdmi, &the_ops.std, DRIVER_INIT_HAL);
