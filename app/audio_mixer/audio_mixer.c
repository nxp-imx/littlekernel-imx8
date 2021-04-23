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
#include <dev/class/adc.h>
#include <dev/class/pdm.h>
#include <kernel/spinlock.h>
#include <kernel/event.h>
#include <kernel/thread.h>

#include <lib/appargs.h>
#include <dev/interrupt.h>
#include <trace.h>
#include "music_data.h"

#define LOCAL_TRACE 5

#define PDM_INPUT

#define DAC_BUSID		1
#define SAI_TX_BUSID		3
#define SAI_TX_AUDIO_CHANNELS	2
#ifdef PDM_INPUT
#define SAI_TX_AUDIO_BITWIDTH	16
#else
#define SAI_TX_AUDIO_BITWIDTH	32
#endif
#define SAI_TX_SAMPLE_BYTES	(SAI_TX_AUDIO_BITWIDTH / 8)
#define SAI_TX_FRAME_BYTES	(SAI_TX_AUDIO_CHANNELS * SAI_TX_SAMPLE_BYTES)
#define SAI_TX_PERIOD_SIZE	4
#define SAI_TX_DMA_PERIOD_BYTES	(SAI_TX_PERIOD_SIZE * SAI_TX_FRAME_BYTES)

#define ADC_BUSID		1
#define SAI_RX_BUSID		5
#define SAI_RX_AUDIO_CHANNELS	8
#define SAI_RX_AUDIO_BITWIDTH	32
#define SAI_RX_SAMPLE_BYTES	(SAI_RX_AUDIO_BITWIDTH / 8)
#define SAI_RX_FRAME_BYTES	(SAI_RX_AUDIO_CHANNELS * SAI_RX_SAMPLE_BYTES)
#define SAI_RX_PERIOD_SIZE	4
#define SAI_RX_DMA_PERIOD_BYTES	(SAI_RX_PERIOD_SIZE * SAI_RX_FRAME_BYTES)

#define PDM_AUDIO_CHANNELS	8
#define PDM_AUDIO_BITWIDTH	16
#define PDM_SAMPLE_BYTES	(PDM_AUDIO_BITWIDTH / 8)
#define PDM_FRAME_BYTES		(PDM_AUDIO_CHANNELS * PDM_SAMPLE_BYTES)
#define PDM_PERIOD_SIZE		4
#define PDM_PERIOD_BYTES	(PDM_PERIOD_SIZE * PDM_FRAME_BYTES)

#define LAT_EVT_MIXER    6000
extern void BOARD_LatencySetEvent(int event);

#if WITH_LIB_CONSOLE

#include <lib/console.h>

static int cmd_audio_mixer(int argc, const cmd_args *argv);

STATIC_COMMAND_START
STATIC_COMMAND("mixer", "audio mixer commands", &cmd_audio_mixer)
STATIC_COMMAND_END(audio_mixer);

static spin_lock_t cb_lock;
static event_t cb_event;

static int dac_sai_cb(sai_cb_evt_t evt, void *data, void *cookie)
{
    u64 period_cnt;

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&cb_lock, lock_state);

    switch(evt) {
    case SAI_EVENT_PERIOD_ELAPSED:
        period_cnt = *(u64*)data;
        if (period_cnt >= MUSIC_DATA_LEN/SAI_TX_DMA_PERIOD_BYTES)
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

static void volume(unsigned char *buf, unsigned int length)
{
#ifdef PDM_INPUT
    u16 *tmp;

    tmp = (u16 *)buf;
    while (length) {
        *tmp = *tmp << 2; //volume * 4
        length = length - sizeof(u16);
        tmp++;
    }
#else
    u32 *tmp;

    tmp = (u32 *)buf;
    while (length) {
        *tmp = *tmp << 2; //volume * 4
        length = length - sizeof(u32);
        tmp++;
    }
#endif
}

static void mixer(unsigned char *rx, unsigned char *tx, unsigned int len)
{
    unsigned int i;

#ifdef PDM_INPUT
    u16 *in = (u16 *)rx;
    u16 *out = (u16 *)tx;

    if (PDM_AUDIO_CHANNELS == 2) {
        for (i = 0; i < len / PDM_SAMPLE_BYTES; ) {
            *out++ = in[i];
            *out++ = in[i + 1];

            i = i + PDM_AUDIO_CHANNELS;
        }
    } else if (PDM_AUDIO_CHANNELS == 8) {
        for (i = 0; i < len / PDM_SAMPLE_BYTES; ) {
            *out++ = (in[i] + in[i + 2] + in[i + 4] + in[i + 6]);
            *out++ = (in[i + 1] + in[i + 3] + in[i + 5] + in[i + 7]);

            i = i + PDM_AUDIO_CHANNELS;
        }
    }
#else
    u32 *in = (u32 *)rx;
    u32 *out = (u32 *)tx;

    if (SAI_RX_AUDIO_CHANNELS == 2) {
        for (i = 0; i < len / SAI_RX_SAMPLE_BYTES; ) {
            *out++ = in[i];
            *out++ = in[i + 1];

            i = i + SAI_RX_AUDIO_CHANNELS;
        }
    } else if (SAI_RX_AUDIO_CHANNELS == 8) {
        for (i = 0; i < len / SAI_RX_SAMPLE_BYTES; ) {
            *out++ = (in[i] + in[i + 2] + in[i + 4] + in[i + 6]);
            *out++ = (in[i + 1] + in[i + 3] + in[i + 5] + in[i + 7]);

            i = i + SAI_RX_AUDIO_CHANNELS;
        }
    }
#endif
}

static int cmd_audio_mixer(int argc, const cmd_args *argv)
{
    status_t ret;
    struct device *sai_tx_dev, *dac_dev;
    sai_format_t sai_tx_fmt;
    unsigned char sai_tx_buf[SAI_TX_DMA_PERIOD_BYTES];
    unsigned int first = 1;

#ifdef PDM_INPUT
    struct device *pdm_dev;
    pdm_hw_params_t pdm_fmt;
    unsigned char pdm_buf[PDM_PERIOD_BYTES];
#else
    struct device *adc_dev, *sai_rx_dev;
    sai_format_t sai_rx_fmt;
    unsigned char sai_rx_buf[SAI_RX_DMA_PERIOD_BYTES];
#endif

    spin_lock_init(&cb_lock);
    event_init(&cb_event, false, 0);

    sai_tx_dev = class_sai_get_device_by_id(SAI_TX_BUSID);
    ASSERT(sai_tx_dev != NULL);
    /* get the WM8524 dev by bus id */
    dac_dev = class_dac_get_device_by_id(DAC_BUSID);
    ASSERT(dac_dev != NULL);

#ifdef PDM_INPUT
    pdm_dev = of_get_device_by_class_and_id("pdm", 0);
    ASSERT(pdm_dev != NULL);
#else
    sai_rx_dev = class_sai_get_device_by_id(SAI_RX_BUSID);
    ASSERT(sai_rx_dev != NULL);

    adc_dev = class_adc_get_device_by_id(ADC_BUSID);
    ASSERT(adc_dev != NULL);
#endif

    ret = class_sai_open(sai_tx_dev, false /* is_rx */);
    ASSERT(ret == 0);

    /* codec (dac) setup, wm8524 does not have cfg  interfaces */
    ret = class_dac_open(dac_dev);
    ASSERT(ret == 0);


#ifdef PDM_INPUT
    ret = class_pdm_open(pdm_dev, true);
    ASSERT(ret == 0);
#else
    ret = class_sai_open(sai_rx_dev, true /* is_rx */);
    ASSERT(ret == 0);

    ret = class_adc_open(adc_dev);
    ASSERT(ret == 0);
#endif

    ret = class_sai_set_callback(sai_tx_dev, dac_sai_cb, NULL, false);
    ASSERT(ret == 0);

    /* SAI3 setup */
    sai_tx_fmt.sai_protocol = 2; /* I2S */
    sai_tx_fmt.sampleRate_Hz = 48000; /* sample rate */
    sai_tx_fmt.bitWidth = SAI_TX_AUDIO_BITWIDTH; /* 24bits only */
    sai_tx_fmt.num_channels = SAI_TX_AUDIO_CHANNELS; /* stereo */
    sai_tx_fmt.num_slots = 2;
    sai_tx_fmt.period_size = SAI_TX_PERIOD_SIZE; /* frames per period */
    sai_tx_fmt.polarity = SAI_BITCLOCK_POLARITY_ACTIVE_LOW;
    sai_tx_fmt.master_slave = ksai_Master;
    sai_tx_fmt.bitclock_source = kSAI_BClkSourceMclkDiv;

    ret = class_sai_setup(sai_tx_dev, false, &sai_tx_fmt);
    if (ret)
        return ret;

#ifdef PDM_INPUT
    pdm_fmt.sampleRate_Hz = 48000;
    pdm_fmt.bitWidth = PDM_AUDIO_BITWIDTH;
    pdm_fmt.num_channels = PDM_AUDIO_CHANNELS;
    pdm_fmt.start_channel = 0; /* RXD0 BITSTREAM0 */

    ret = class_pdm_setup(pdm_dev, true, &pdm_fmt);
    ASSERT(ret == 0);

    class_pdm_start(pdm_dev, true);
#else
    adc_audio_hw_params_t hw_params = {
        .pcm_fmt = ADC_AUDIO_PCM_FMT_32,
        .fmt = ADC_AUDIO_FMT_I2S,
        .pkt = ADC_AUDIO_PKT_PCM,
        .num_ch = SAI_RX_AUDIO_CHANNELS,
        .rate = 48000,
    };
    class_adc_set_format(adc_dev, &hw_params);

    sai_rx_fmt.sai_protocol = 2;
    sai_rx_fmt.sampleRate_Hz = 48000;
    sai_rx_fmt.bitWidth = SAI_RX_AUDIO_BITWIDTH;
    sai_rx_fmt.num_channels = SAI_RX_AUDIO_CHANNELS; /* stereo */
    sai_rx_fmt.num_slots = 2;
    sai_rx_fmt.period_size = SAI_RX_PERIOD_SIZE; /* frames per period */
    sai_rx_fmt.polarity = SAI_BITCLOCK_POLARITY_ACTIVE_LOW;
    sai_rx_fmt.master_slave = ksai_Master;
    sai_rx_fmt.bitclock_source = kSAI_BClkSourceMclkDiv;
    ret = class_sai_setup(sai_rx_dev, true, &sai_rx_fmt);
    if (ret)
        return ret;

    ret = class_sai_start(sai_rx_dev, true);
    ASSERT(ret == 0);
#endif

    /* start SAI and dma, data filling in the cb */
    ret = class_sai_start(sai_tx_dev, false);
    ASSERT(ret == 0);

    while (true) {
#ifdef LATENCY_MEASUREMENTS
        BOARD_LatencySetEvent(LAT_EVT_MIXER);
#endif
#ifdef PDM_INPUT
        ret = class_pdm_read(pdm_dev, pdm_buf, PDM_PERIOD_BYTES);
#else
        ret = class_sai_read(sai_rx_dev, sai_rx_buf, SAI_RX_DMA_PERIOD_BYTES);
#endif
        if (ret == 0) {
//            volume(pdm_buf, PDM_PERIOD_BYTES);

            if (first) {
                memset(sai_tx_buf, 0xaa, SAI_TX_DMA_PERIOD_BYTES);
                first = 0;
            } else {
//                memset(sai_tx_buf, 0x10, SAI_TX_DMA_PERIOD_BYTES);
#ifdef PDM_INPUT
                mixer(pdm_buf, sai_tx_buf, PDM_PERIOD_BYTES);
#else
                mixer(sai_rx_buf, sai_tx_buf, SAI_RX_DMA_PERIOD_BYTES);
#endif
            }

#ifdef LATENCY_MEASUREMENTS
            BOARD_LatencySetEvent(LAT_EVT_MIXER);
#endif
            class_sai_write(sai_tx_dev, sai_tx_buf, SAI_TX_DMA_PERIOD_BYTES);
        } else {
            printlk(LK_INFO, "sai read error:%d\n", ret);
        }
    }

    /* wait for all data sent */
//    event_wait(&cb_event);

    /* stop SAI and dma */
    ret = class_sai_stop(sai_tx_dev, false);
    ASSERT(ret == 0);

#ifdef PDM_INPUT
    ret = class_pdm_stop(pdm_dev, true);
    ASSERT(ret == 0);
#else
    ret = class_sai_stop(sai_rx_dev, true);
    ASSERT(ret == 0);
#endif

//    event_unsignal(&cb_event);

    ret = class_sai_close(sai_tx_dev, false);
    ASSERT(ret == 0);

    ret = class_dac_close(dac_dev);
    ASSERT(ret == 0);

#ifdef PDM_INPUT
    ret = class_pdm_close(pdm_dev, true);
    ASSERT(ret == 0);
#else
    ret = class_sai_close(sai_rx_dev, true);
    ASSERT(ret == 0);

    ret = class_adc_close(adc_dev);
    ASSERT(ret == 0);
#endif

    return 0;
}


APP_START(audio_mixer)
    .flags = 0,
APP_END

#endif // WITH_APP_CONSOLE
