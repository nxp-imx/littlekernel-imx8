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

#include <platform/interrupts.h>
#include <dev/class/adc.h>
#include <ivshmem-rpc.h>
#include <lib/appargs.h>
#include <err.h>
#include <malloc.h>
#include <string.h>
#include <debug.h>


struct rpc_adc_s_format_s {
    adc_audio_pcm_format_t pcm_fmt;
    adc_audio_fmt_t fmt;
    adc_audio_pkt_t pkt;
    unsigned num_ch;
    unsigned rate;
};

struct linux_rpc_adc_state {
    struct device *device;
    adc_audio_hw_params_t hw_params;
    struct ivshm_rpc_service *rpc_service;
};

static status_t linux_rpc_adc_init(struct device *dev)
{
    int ret;

    struct linux_rpc_adc_state *state =
                                malloc(sizeof(struct linux_rpc_adc_state));

    uint32_t service_id;
    ASSERT(state);
    memset(state, 0, sizeof(struct linux_rpc_adc_state));

    state->device = dev;

    struct device *rpc_dev = of_device_lookup_device(dev, "rpc,service");
    if (rpc_dev == NULL) {
        free(state);
        printlk(LK_ERR, "%s:%d: Invalid rpc phandle\n", __PRETTY_FUNCTION__,
                __LINE__);
        return ERR_INVALID_ARGS;
    }

    if (of_device_get_int32(rpc_dev, "id", &service_id)) {
        free(state);
        printlk(LK_ERR,
                "%s:%d: Missing service-id property in device node. Failing..,\n",
                __PRETTY_FUNCTION__, __LINE__);
        return ERR_INVALID_ARGS;
    }

    state->rpc_service = ivshm_get_rpc_service(service_id);
    ASSERT(state->rpc_service);

    dev->state = state;

    int tries = 20;
    while (tries-- > 0 &&
           (ret = ivshm_rpc_cmd(state->rpc_service, RPC_ADC_INIT_ID)) != 0) {
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

status_t linux_rpc_adc_open(struct device *dev)
{
    status_t ret;
    struct linux_rpc_adc_state *state = dev->state;

    printlk(LK_INFO, "Open\n");

    ret = ivshm_rpc_cmd(state->rpc_service, RPC_ADC_OPEN_ID);

    return ret;
}

status_t linux_rpc_adc_close(struct device *dev)
{
    status_t ret;
    struct linux_rpc_adc_state *state = dev->state;

    printlk(LK_INFO, "Close\n");

    ret = ivshm_rpc_cmd(state->rpc_service, RPC_ADC_CLOSE_ID);

    return ret;
}

status_t linux_rpc_adc_get_capabilities(const struct device * dev,
                                                        uint32_t *capabilities)
{
    status_t ret;
    struct linux_rpc_adc_state *state = dev->state;

    printlk(LK_INFO, "Get capabilities\n");

    ret = ivshm_rpc_cmd_get(state->rpc_service, RPC_ADC_G_CAP_ID,
                                                capabilities, sizeof(uint32_t));

    return ret;
}

status_t linux_rpc_adc_set_format(struct device *dev,
                                            adc_audio_hw_params_t *hw_params)
{
    status_t ret;
    struct linux_rpc_adc_state *state = dev->state;
    struct rpc_adc_s_format_s format = {
        .pcm_fmt = hw_params->pcm_fmt,
        .fmt =  hw_params->fmt,
        .pkt =  hw_params->pkt,
        .num_ch =  hw_params->num_ch,
        .rate = hw_params->rate
    };

    printlk(LK_INFO, "Set audio format\n");

    ret = ivshm_rpc_cmd_set(state->rpc_service, RPC_ADC_S_FORMAT_ID,
                            &format, sizeof(struct rpc_adc_s_format_s));

    return ret;

}

static struct device_class linux_rpc_adc_device_class = {
    .name = "adc",
};
static struct adc_ops the_ops = {
        .std = {
            .device_class = &linux_rpc_adc_device_class,
            .init = linux_rpc_adc_init,
        },
        .open               = linux_rpc_adc_open,
        .close              = linux_rpc_adc_close,
        .get_capabilities   = linux_rpc_adc_get_capabilities,
        .set_format         = linux_rpc_adc_set_format,
};

DRIVER_EXPORT_WITH_LVL(linux_rpc_adc, &the_ops.std, DRIVER_INIT_HAL);
