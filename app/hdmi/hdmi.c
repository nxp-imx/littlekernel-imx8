/*
 * Copyright 2019-2020 NXP
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include <app.h>
#include <debug.h>
#include <err.h>
#include <string.h>
#include <stdlib.h>
#include <dev/class/hdmi.h>
#include <assert.h>
#include <kernel/thread.h>


#undef DEBUG_AUDIO_META_DATA
#undef  CONCURRENT_AUDIO_POLLING

#if WITH_LIB_CONSOLE

#include <lib/console.h>

static struct hdmi_app_state {
    struct device *dev;
    unsigned rate;
    hdmi_audio_pkt_t pkt_type;
    hdmi_audio_infoframe_pkt_t audio_infoframe_pkt;
    hdmi_channel_status_t cs;
} state;


static int hdmi_cb(hdmi_cb_evt_t evt, void *data, void *cookie)
{
    struct hdmi_app_state *_state = cookie;
    uint32_t *sample_rate = data;
    hdmi_audio_pkt_t *pkt = data;
    hdmi_audio_infoframe_pkt_t *audio_pkt = data;
    hdmi_channel_status_t *cs = data;

    ASSERT(_state == &state);

    switch(evt) {
    case HDMI_EVENT_AUDIO_SAMPLE_RATE:
        printlk(LK_NOTICE, "%s:%d: HDMI_EVENT_AUDIO_SAMPLE_RATE event, sample rate %d\n",
                __PRETTY_FUNCTION__, __LINE__, *sample_rate);
        _state->rate = *sample_rate;
        break;
    case HDMI_EVENT_AUDIO_STREAM_TYPE:
        printlk(LK_NOTICE, "%s:%d: HDMI_EVENT_AUDIO_STREAM_TYPE event, pkt %d\n",
                __PRETTY_FUNCTION__, __LINE__, *pkt);
        _state->pkt_type = *pkt;
        break;
    case HDMI_EVENT_AUDIO_INFOFRAME:
        printlk(LK_NOTICE, "%s:%d: HDMI_EVENT_AUDIO_INFOFRAME event\n",
                __PRETTY_FUNCTION__, __LINE__);
        _state->audio_infoframe_pkt = *audio_pkt;
#if DEBUG_AUDIO_META_DATA
        hdmi_print_infoframe(audio_pkt);
#endif
        break;
    case HDMI_EVENT_AUDIO_CHANNEL_STATUS:
        printlk(LK_NOTICE, "%s:%d: HDMI_EVENT_AUDIO_CHANNEL_STATUS event\n",
                __PRETTY_FUNCTION__, __LINE__);
        _state->cs = *cs;

#if DEBUG_AUDIO_META_DATA
        hdmi_print_channel_status(cs);
#endif
        break;
    case HDMI_EVENT_AUDIO_LINK:
        printlk(LK_NOTICE, "%s:%d: HDMI_EVENT_AUDIO_LINK notification\n",
                __PRETTY_FUNCTION__, __LINE__);
        break;
    case HDMI_EVENT_AUDIO_MCLK:
        printlk(LK_NOTICE, "%s:%d: HDMI_EVENT_AUDIO_MCLK notification\n",
                __PRETTY_FUNCTION__, __LINE__);
        break;
    case HDMI_EVENT_ERROR:
        printlk(LK_NOTICE, "%s:%d: HDMI_EVENT_ERROR notification\n",
                __PRETTY_FUNCTION__, __LINE__);
        break;
    default:
        printlk(LK_ERR, "%s:%d: Unknown HDMI_EVENT_AUDIO notification\n",
                __PRETTY_FUNCTION__, __LINE__);
    }


    return 0;
}

static int cmd_hdmi(int argc, const cmd_args *argv);

STATIC_COMMAND_START
STATIC_COMMAND("hdmi", "hdmi commands", &cmd_hdmi)
STATIC_COMMAND_END(hdmi);

#define DO_CMD_HDMI(func) \
    if (!strcmp(argv[2].str, #func)) { \
        err = class_hdmi_##func(dev); \
        printf("hdmi %s on bus %d returned  %#x\n", #func, bus, err); \
        return 0; \
    }

static int cmd_hdmi(int argc, const cmd_args *argv)
{
    int err;

    if (argc < 2) {
argument:
        printf("not enough arguments\n");
usage:
        printf("%s list\n", argv[0].str);
        printf("%s bus test\n", argv[0].str);
        printf("%s bus open\n", argv[0].str);
        printf("%s bus close\n", argv[0].str);
        printf("%s bus rpc\n", argv[0].str);
        return -1;
    }

    if (!strcmp(argv[1].str, "list")) {
        int i;
        for (i = 0; i < 8; i++) {
            struct device *dev = class_hdmi_get_device_by_id(i);
            if (dev)
                printf("%d: %s\n", i, dev->name);
        }
        return 1;
    }

    if (argc < 3)
        goto argument;

    int bus = argv[1].u;
    struct device *dev = class_hdmi_get_device_by_id(bus);
    if (dev == NULL) {
        printf("No device found on hdmi bus %d\n", bus);
        return 1;
    }

    state.dev = dev;

    DO_CMD_HDMI(open)
    DO_CMD_HDMI(close)

    if (!strcmp(argv[2].str, "test")) {
        err = class_hdmi_open(dev);
        printf("hdmi open on bus %d returned  %#x\n", bus, err);
        if (err) {
            return 1;
        }

        err = class_hdmi_set_callback(dev, hdmi_cb, &state);
        printf("hdmi set callback on bus %d returned  %#x\n", bus, err);
        if (err)
            return 1;
#ifdef CONCURRENT_AUDIO_POLLING
        hdmi_audio_infoframe_pkt_t audio_infoframe_pkt;
        hdmi_channel_status_t channel_status;
        while (true) {
            status_t ret = class_hdmi_get_audio_infoframe_pkt(dev, &audio_infoframe_pkt);
            if (ret == ERR_BUSY) {
                printf("Infoframe busy\n");
            } else {
                hexdump8_ex(&audio_infoframe_pkt,
                                    sizeof(hdmi_audio_infoframe_pkt_t), false);
            }
            ret = class_hdmi_get_channel_status(dev, &channel_status);
            if (ret == ERR_BUSY) {
                printf("Channel status busy\n");
            } else {
                hexdump8_ex(&channel_status,
                                    sizeof(hdmi_channel_status_t), false);
            }

            thread_sleep(2000);
        }
#endif
    }
    if (!strcmp(argv[2].str, "rpc")) {
        uint32_t capabilities;
        hdmi_audio_direction_t direction;
        hdmi_audio_fmt_t fmt;
        hdmi_audio_pkt_t pkt;
        hdmi_audio_if_t in, out;
        hdmi_channel_status_t channel_status;
        hdmi_audio_infoframe_pkt_t infoframe;
        hdmi_audio_pkt_layout_t pkt_layout;
        iec60958_custom_fmt_layout_t fmt_layout;

        err = class_hdmi_open(dev);
        printf("hdmi open on bus %d returned  %#x\n", bus, err);
        if (err) {
            return 1;
        }

        err = class_hdmi_set_callback(dev, hdmi_cb, &state);
        printf("hdmi set callback on bus %d returned  %#x\n", bus, err);
        if (err)
            return 1;

        err = class_hdmi_get_capabilities(dev, &capabilities);
        if (err)
            return 1;
        printf("hdmi get capabilities returns: %#x\n", capabilities);

        direction = HDMI_AUDIO_SINK;
        fmt = HDMI_AUDIO_FMT_61937;
        err = class_hdmi_set_audio_format(dev, direction, fmt);
        if (err)
            return 1;
        printf("hdmi set audio format done: direction/fmt [%x:%x]\n",
                                                            direction, fmt);

        direction = HDMI_AUDIO_SOURCE;
        pkt = HDMI_AUDIO_PKT_STD;
        err = class_hdmi_set_audio_packet(dev, direction, pkt);
        if (err)
            return 1;
        printf("hdmi set audio packet done: direction/pkt [%x:%x]\n",
                                                            direction, pkt);

        direction = HDMI_AUDIO_SINK;
        in = HDMI_AUDIO_IF_HDMI;
        out = HDMI_AUDIO_IF_ARC;
        err = class_hdmi_set_audio_interface(dev, direction, in, out);
        if (err)
            return 1;
        printf("hdmi set audio interface done: direction/in/out [%x:%x:%x]\n",
                                                        direction, in, out);

        err = class_hdmi_get_channel_status(dev, &channel_status);
        if (err)
            return 1;
        printf("hdmi get channel status done:\n");
        hexdump8_ex(&channel_status, sizeof(hdmi_channel_status_t), false);

        err = class_hdmi_get_audio_infoframe_pkt(dev, &infoframe);
        if (err)
            return 1;
        printf("hdmi get audio infoframe done:\n");
        hexdump8_ex(&infoframe, sizeof(hdmi_audio_infoframe_pkt_t), false);

        err = class_hdmi_get_audio_pkt_type(dev, &pkt);
        if (err)
            return 1;
        printf("hdmi get audio pkt type done: [%x]\n", pkt);

        err = class_hdmi_get_audio_pkt_layout(dev, &pkt_layout);
        if (err)
            return 1;
        printf("hdmi get audio pkt layout done: [%x]\n", pkt_layout);

        err = class_hdmi_get_audio_custom_fmt_layout(dev, &fmt_layout);
        if (err)
            return 1;
        printf("hdmi get audio fmt layout done:\n");
        hexdump8_ex(&fmt_layout, sizeof(iec60958_custom_fmt_layout_t), false);

        err = class_hdmi_close(dev);
        printf("hdmi close on bus %d returned  %#x\n", bus, err);
        if (err)
            return 1;

        return 0;
    }

    printf("unrecognized subcommand\n");
    goto usage;
}

APP_START(hdmi)
    .flags = 0,
APP_END

#endif // WITH_APP_CONSOLE


