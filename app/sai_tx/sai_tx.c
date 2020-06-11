/*
 * Copyright 2020-2021 NXP
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
#include <err.h>
#include <string.h>
#include <dev/class/sai.h>
#include <dev/class/dac.h>
#include <kernel/spinlock.h>
#include <kernel/event.h>

#include <dev/interrupt.h>
#include <trace.h>
#include "music.h"

#define LOCAL_TRACE 5

#define APP_SAI_BUSID 3
#define APP_DAC_BUSID 1
#define APP_AUDIO_SRATE 48000
#define APP_AUDIO_CHANNELS 2
#define APP_AUDIO_BITWIDTH 32
#define APP_FRAME_BYTES (APP_AUDIO_CHANNELS * APP_AUDIO_BITWIDTH / 8)
#define APP_PERIOD_SIZE 128
#define APP_DMA_PERIOD_SIZE (APP_PERIOD_SIZE * APP_FRAME_BYTES)

#if WITH_LIB_CONSOLE

#include <lib/console.h>

static int cmd_sai_tx(int argc, const cmd_args *argv);

STATIC_COMMAND_START
STATIC_COMMAND("sai_tx", "sai transmit commands", &cmd_sai_tx)
STATIC_COMMAND_END(sai_tx);

/* spin lock for callback */
static spin_lock_t cb_lock;
static event_t cb_event;
static int done_bytes;

/* callback for SAI -> DAC */
static int dac_sai_cb(sai_cb_evt_t evt, void *data, void *cookie)
{
    u64 period_cnt;

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&cb_lock, lock_state);

    switch(evt) {
    case SAI_EVENT_PERIOD_ELAPSED:
        period_cnt = *(u64*)data;
        if (period_cnt >= MUSIC_LEN/APP_DMA_PERIOD_SIZE)
            event_signal(&cb_event, false);
        break;
    case SAI_EVENT_ACTIVE:
        printlk(LK_INFO, "SAI_EVENT_ACTIVE\n");
        break;
    case SAI_EVENT_ERROR:
        printlk(LK_INFO, "SAI_EVENT_ERROR\n");
        break;
    case SAI_EVENT_DRAIN:
        printlk(LK_INFO, "SAI_EVENT_DRAIN\n");
        break;
    default:
        printlk(LK_NOTICE, "%s:%d: Unhandled event (%d) received\n",
                __PRETTY_FUNCTION__, __LINE__, evt);
    };

    spin_unlock_irqrestore(&cb_lock, lock_state);
    return 0;
}

static int cmd_sai_tx(int argc, const cmd_args *argv)
{
    status_t ret;
    struct device *sai_dev, *dac_dev;
    sai_format_t sai_fmt;


    /* Initialize sync objects */
    spin_lock_init(&cb_lock);
    event_init(&cb_event, false, 0);

    /* get SAI3 dev by bus id */
    sai_dev = class_sai_get_device_by_id(APP_SAI_BUSID);
    ASSERT(sai_dev != NULL);
    /* get the WM8524 dev by bus id */
    dac_dev = class_dac_get_device_by_id(APP_DAC_BUSID);
    ASSERT(dac_dev != NULL);

    ret = class_sai_open(sai_dev, false /* is_rx */);
    ASSERT(ret == 0);

    /* codec (dac) setup, wm8524 does not have cfg  interfaces */
    ret = class_dac_open(dac_dev);
    ASSERT(ret == 0);

    /* setup the callback for tx */
    ret = class_sai_set_callback(sai_dev, dac_sai_cb, NULL, false);
    ASSERT(ret == 0);

    /* SAI3 setup */
    sai_fmt.sai_protocol = 2; /* I2S */
    sai_fmt.sampleRate_Hz = APP_AUDIO_SRATE; /* sample rate */
    sai_fmt.bitWidth = 32; /* 24bits only */
    sai_fmt.num_channels = 2; /* stereo */
    sai_fmt.num_slots = 2;
    sai_fmt.period_size = APP_PERIOD_SIZE; /* frames per period */
    sai_fmt.polarity = SAI_BITCLOCK_POLARITY_ACTIVE_LOW;
    sai_fmt.master_slave = ksai_Master,
    sai_fmt.bitclock_source = kSAI_BClkSourceMclkDiv,

    ret = class_sai_setup(sai_dev, false, &sai_fmt);
    if (ret)
        return ret;

    /* write music buffer to cbuf */
    ret = class_sai_write(sai_dev, music, MUSIC_LEN);
    if (ret == 0) {
        printf("Write music data to cbuf done\n");
    } else {
        printf("Write music data failed:%d\n", ret);
        return ret;
    }

    done_bytes = 0;
    /* start SAI and dma, data filling in the cb */
    ret = class_sai_start(sai_dev, false);
    ASSERT(ret == 0);

    /* wait for all data sent */
    event_wait(&cb_event);

    /* stop SAI and dma */
    ret = class_sai_stop(sai_dev, false);
    ASSERT(ret == 0);

    event_unsignal(&cb_event);

    /* close SAI and dac */
    ret = class_sai_close(sai_dev, false);
    ASSERT(ret == 0);

    ret = class_dac_close(dac_dev);
    ASSERT(ret == 0);

    return 0;
}


APP_START(sai_tx)
    .flags = 0,
APP_END

#endif // WITH_APP_CONSOLE
