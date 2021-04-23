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
#include <dev/class/pdm.h>
#include <kernel/spinlock.h>
#include <kernel/event.h>

#include <lib/appargs.h>
#include <dev/interrupt.h>
#include <trace.h>

#define LOCAL_TRACE 5

#define APP_SAI_BUSID 3
#define APP_DAC_BUSID 1
#define APP_AUDIO_SRATE 48000
#define APP_AUDIO_CHANNELS 2
#define APP_AUDIO_BITWIDTH 16
#define APP_FRAME_BYTES (APP_AUDIO_CHANNELS * APP_AUDIO_BITWIDTH / 8)
#define APP_PERIOD_SIZE 128
#define APP_DMA_PERIOD_BYTES (APP_PERIOD_SIZE * APP_FRAME_BYTES)
#define APP_PDM_PERIOD_BYTES APP_DMA_PERIOD_BYTES

#if WITH_LIB_CONSOLE

#include <lib/console.h>

static int cmd_pdm_play(int argc, const cmd_args *argv);

STATIC_COMMAND_START
STATIC_COMMAND("pdm_play", "PDM recording and playback to DAC demo", &cmd_pdm_play)
STATIC_COMMAND_END(pdm_play);

/* spin lock for callback */
static spin_lock_t cb_lock;
static event_t cb_event;

/* callback for SAI -> DAC */
static int dac_sai_cb(sai_cb_evt_t evt, void *data, void *cookie)
{
    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&cb_lock, lock_state);

    switch(evt) {
    case SAI_EVENT_PERIOD_ELAPSED:
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

static int cmd_pdm_play(int argc, const cmd_args *argv)
{
    status_t ret;
    struct device *sai_dev, *dac_dev, *pdm_dev;
    sai_format_t sai_fmt;
    pdm_hw_params_t pdm_fmt;
    char pdm_buf[APP_PDM_PERIOD_BYTES];

    /* Initialize sync objects */
    spin_lock_init(&cb_lock);
    event_init(&cb_event, false, 0);

    /* get SAI3 dev by bus id */
    sai_dev = class_sai_get_device_by_id(APP_SAI_BUSID);
    ASSERT(sai_dev != NULL);
    /* get the WM8524 dev by bus id */
    dac_dev = class_dac_get_device_by_id(APP_DAC_BUSID);
    ASSERT(dac_dev != NULL);

    pdm_dev = of_get_device_by_class_and_id("pdm", 0);
    ASSERT(pdm_dev != NULL);

    ret = class_sai_open(sai_dev, false /* is_rx */);
    ASSERT(ret == 0);

    /* codec (dac) setup, wm8524 does not have cfg  interfaces */
    ret = class_dac_open(dac_dev);
    ASSERT(ret == 0);

    /* pdm open and setup */
    ret = class_pdm_open(pdm_dev, true);
    ASSERT(ret == 0);

    /* setup the callback for tx */
    ret = class_sai_set_callback(sai_dev, dac_sai_cb, NULL, false);
    ASSERT(ret == 0);

    /* SAI3 setup */
    sai_fmt.sai_protocol = 2; /* I2S */
    sai_fmt.sampleRate_Hz = APP_AUDIO_SRATE; /* sample rate */
    sai_fmt.bitWidth = APP_AUDIO_BITWIDTH;
    sai_fmt.num_channels = 2; /* stereo */
    sai_fmt.num_slots = 2;
    sai_fmt.period_size = APP_PERIOD_SIZE; /* frames per period */
    sai_fmt.polarity = SAI_BITCLOCK_POLARITY_ACTIVE_LOW;
    sai_fmt.master_slave = ksai_Master,
    sai_fmt.bitclock_source = kSAI_BClkSourceMclkDiv,

    ret = class_sai_setup(sai_dev, false, &sai_fmt);
    if (ret)
        return ret;

    /* PDM setup and start */
    pdm_fmt.sampleRate_Hz = APP_AUDIO_SRATE;
    pdm_fmt.bitWidth = APP_AUDIO_BITWIDTH;
    pdm_fmt.num_channels = 2;
    pdm_fmt.start_channel = 0; /* RXD0 BITSTREAM0 */

    ret = class_pdm_setup(pdm_dev, true, &pdm_fmt);
    ASSERT(ret == 0);

    class_pdm_start(pdm_dev, true);

    /* start SAI and dma, data filling in the cb */
    ret = class_sai_start(sai_dev, false);
    ASSERT(ret == 0);

    while (true) {
        ret = class_pdm_read(pdm_dev, pdm_buf, APP_PDM_PERIOD_BYTES);
        if (ret == 0) {
            /* write pdm recording buffer to cbuf */
            class_sai_write(sai_dev, pdm_buf, APP_PDM_PERIOD_BYTES);
            //printlk(LK_INFO, "pdm read out and send to sai\n");
        } else {
            printlk(LK_INFO, "pdm read error:%d\n", ret);
        }
        /* wait for all data sent */
        //event_wait(&cb_event);
        //event_unsignal(&cb_event);
    }

    /* stop SAI and dma */
    ret = class_sai_stop(sai_dev, false);
    ASSERT(ret == 0);

    /* stop PDM */
    ret = class_pdm_stop(sai_dev, true);
    ASSERT(ret == 0);

    /* close SAI and dac */
    ret = class_sai_close(sai_dev, false);
    ASSERT(ret == 0);

    ret = class_dac_close(dac_dev);
    ASSERT(ret == 0);

    ret = class_pdm_close(pdm_dev, true);
    ASSERT(ret == 0);

    return 0;
}


APP_START(pdm_play)
    .flags = 0,
APP_END

#endif // WITH_APP_CONSOLE
