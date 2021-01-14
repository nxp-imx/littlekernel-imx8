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

#include <stdlib.h>
#include <string.h>
#include <err.h>
#include <assert.h>
#include <math.h>
#include <debug.h>

#include <dev/driver.h>
#include <dev/class/sai.h>
#include <kernel/thread.h>
#include <kernel/semaphore.h>
#include <kernel/event.h>
#include <lib/appargs.h>
#include <lib/dpc.h>
#include <platform/spdif_cs.h>


#include "spdif_extract.h"

/* ---------------------------------------------------------------------------- */
/* Defines                                                                      */
/* ---------------------------------------------------------------------------- */
#define SPDIF_CS_DEFAULT_POLLING_RATE 100

#define SPDIF_CS_STACK_SIZE    DEFAULT_STACK_SIZE
#define SPDIF_CS_THREAD_PRIO   LOW_PRIORITY


#define SPDIF_CS_IEC_BLOCK_SZ 192
#define SPDIF_CS_IEC_BLOCK_SZ_BYTES (192 * 2 * sizeof(uint32_t))

#define SPDIF_CS_DATA_SZ_BYTES 24

/*
 * SAI RX oversampled buffer sized for 2 IEC blocks OSR=8
 *     2 * 192 * 2 * 4 * 8 = 24576 bytes
 * rounded to upper power of 2
 */
#define SPDIF_CS_OSR_BUFFER_SZ_BYTES (32 * 1024)

/*
 * Biphase buffer size for 2 IEC blocks : 2 biphase bits per IEC bit
 */
#define SPDIF_CS_BIPHASE_BUFFER_SZ_BYTES (2 * SPDIF_CS_IEC_BLOCK_SZ_BYTES * 2)

#define SPDIF_CS_DATA_BUFFERS 2

#define SPDIF_CS_DEFAULT_SPDIF_RATE   48000

/* 2.6 ms at 48 kHz (OSR=4), 1.3 ms @ 96k (OSR=4) and 192k (OSR=2)*/
#define SPDIF_CS_DEFAULT_PERIOD_SIZE  2048

/* number of equal CS data to be received before reporting */
#define SPDIF_CS_DEFAULT_REDUNDANCY 1

#undef SPDIF_CS_AUTOSTART
#undef SPDIF_CS_PRINT_STATS


/* ---------------------------------------------------------------------------- */
/* Struct                                                                       */
/* ---------------------------------------------------------------------------- */


struct spdif_cs_s {
    thread_t *thread;
    event_t event;
    mutex_t mlock;
    unsigned state;
    unsigned polling_rate_ms;
    unsigned spdif_rate;
    unsigned sai_rate;
    unsigned osr;
    unsigned redundancy;
    unsigned redundancy_count;
    struct device *dev_sai;
    bool running;
    spdif_cs_event_cb callback;
    void *cookie;
    char osr_buff[SPDIF_CS_OSR_BUFFER_SZ_BYTES]
        __attribute__((aligned(32)));
    char biphase_buff[SPDIF_CS_BIPHASE_BUFFER_SZ_BYTES]
        __attribute__((aligned(32)));
    size_t cs_current;
    char cs_buff[SPDIF_CS_DATA_BUFFERS][SPDIF_CS_DATA_SZ_BYTES];
    char cs_data[SPDIF_CS_DATA_SZ_BYTES];

    unsigned count_error;
    unsigned count_decode;
    unsigned count_update;
};


/* ---------------------------------------------------------------------------- */
/* Enum                                                                         */
/* ---------------------------------------------------------------------------- */

enum spdif_cs_state {
    SPDIF_CS_ST_ACTIVE    = (1UL << 1),  /*!< Started   */
    SPDIF_CS_ST_SETUP     = (1UL << 2),  /*!< HAL setup */
};


/* ---------------------------------------------------------------------------- */
/* Global var                                                                   */
/* ---------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------- */
/* API Functions                                                                */
/* ---------------------------------------------------------------------------- */


static void _spdif_cs_enable(struct spdif_cs_s *cs)
{
    event_signal(&cs->event, false);
}

static void _spdif_cs_disable(struct spdif_cs_s *cs)
{
    event_unsignal(&cs->event);
}

/* state functions to be called with lock held */
static inline void spdif_cs_change_state(unsigned *state,
                                         enum spdif_cs_state value, bool set)
{
    if (set)
        *state |= value;
    else
        *state &= ~value;
    smp_wmb();
}

static inline void spdif_cs_set_state(unsigned *state, enum spdif_cs_state value)
{
    spdif_cs_change_state(state, value, true);
}

static inline void spdif_cs_clear_state(unsigned *state, enum spdif_cs_state value)
{
    spdif_cs_change_state(state, value, false);
}

static inline bool spdif_cs_is_state(unsigned state, enum spdif_cs_state value)
{
    return !!(state & value);
}

static void *spdif_cs_get_current_cs_buff(struct spdif_cs_s *cs)
{
    return (void *)cs->cs_buff[cs->cs_current];
}

static void *spdif_cs_get_next_cs_buff(struct spdif_cs_s *cs)
{
    size_t next = (cs->cs_current + 1) % SPDIF_CS_DATA_BUFFERS;
    return (void *)cs->cs_buff[next];
}

static void spdif_cs_flip_cs_buff(struct spdif_cs_s *cs)
{
    size_t next = (cs->cs_current + 1) % SPDIF_CS_DATA_BUFFERS;
    cs->cs_current = next;
}

static void spdif_cs_clear_cs_buff(struct spdif_cs_s *cs)
{
    char *cs_data;
    cs_data = spdif_cs_get_current_cs_buff(cs);
    memset(cs_data, 0, SPDIF_CS_DATA_SZ_BYTES);
    cs_data = spdif_cs_get_next_cs_buff(cs);
    memset(cs_data, 0, SPDIF_CS_DATA_SZ_BYTES);
}

static int spdif_cs_read_sai(struct device *dev_sai,
                             char *buff, size_t len)
{
    int ret;

    ret = class_sai_start(dev_sai, true /* is read */);
    if (ret) {
        printlk(LK_ERR, "%s:%d: start error %d\n",
                __PRETTY_FUNCTION__, __LINE__, ret);
        return ret;
    }

    ret = class_sai_read(dev_sai, buff, len);
    if (ret) {
        printlk(LK_ERR, "%s:%d: read error %d\n",
                __PRETTY_FUNCTION__, __LINE__, ret);
        class_sai_stop(dev_sai, true /* is read */);
        return ret;
    }

    ret = class_sai_stop(dev_sai, true /* is read */);
    if (ret) {
        printlk(LK_ERR, "%s:%d: stop error %d\n",
                __PRETTY_FUNCTION__, __LINE__, ret);
        return ret;
    }

    return NO_ERROR;
}

static int spdif_cs_sai_open(struct device *dev_sai)
{
    return class_sai_open(dev_sai, true /* is read */);
}

static int spdif_cs_sai_close(struct device *dev_sai)
{
    return class_sai_close(dev_sai, true /* is read */);
}

static int spdif_cs_sai_setup(struct device *dev_sai, sai_format_t *sai_format)
{
    return class_sai_setup(dev_sai, true /* is read */, sai_format);
}

static const struct {
    unsigned spdif_rate; /* frequency detected by SPDIF */
    unsigned sai_rate;   /* spdif_rate * 2 (biphase) * osr */
    unsigned osr;        /* biphase over sampling rate */
} rate_table[] = {
#if 0 /* TODO: crashes - skip those rates for now */
    { 32000,  256000, 4 },
#endif
    { 44100,  352800, 4 },
    { 48000,  384000, 4 },
    { 88200,  705600, 4 },
    { 96000,  768000, 4 },
    { 176400, 705600, 2 },
    { 192000, 768000, 2 },
};

static int spdif_cs_rate_params(unsigned spdif_rate_raw, unsigned *spdif_rate_round,
                                unsigned *sai_rate, unsigned *osr)
{
    size_t i;
    int ret = ERR_NOT_FOUND;
    float ratio, val;

/* SPDIF driver reports approximate rate - tolerance in ppm */
#define SPDIF_CS_RATIO_PPM 50000

    for (i = 0; i < ARRAY_SIZE(rate_table); i++) {
        val = (float)rate_table[i].spdif_rate;
        ratio = fabs((val - spdif_rate_raw) / val * 1.0e6f);
        if ((int)ratio < SPDIF_CS_RATIO_PPM) {
            *sai_rate         = rate_table[i].sai_rate;
            *spdif_rate_round = rate_table[i].spdif_rate;
            *osr              = rate_table[i].osr;
            ret = NO_ERROR;
            break;
        }
    }

    if (ret)
        printlk(LK_INFO, "no match found:%d\n", spdif_rate_raw);
    else
        printlk(LK_INFO, "match raw:%d rate:%d ratio:%d\n",
                             spdif_rate_raw, *spdif_rate_round, (int)ratio);

    return ret;
}


int spdif_cs_register(struct device *dev, void *cookie, spdif_cs_event_cb cb)
{
    struct spdif_cs_s *cs = dev->state;

    if (!cs)
        return ERR_NOT_READY;

    mutex_acquire(&cs->mlock);
    cs->callback = cb;
    cs->cookie   = cookie;
    mutex_release(&cs->mlock);

    return NO_ERROR;
}

int spdif_cs_start(struct device *dev, unsigned spdif_rate)
{
    struct spdif_cs_s *cs = dev->state;
    unsigned raw = spdif_rate, round, sai_rate, osr;
    int ret = NO_ERROR;

    printlk(LK_INFO, "start rate:%d\n", spdif_rate);

    if (!cs)
        return ERR_NOT_READY;

    ret = spdif_cs_rate_params(raw, &round, &sai_rate, &osr);
    if (ret != NO_ERROR)
        return ERR_INVALID_ARGS;

    mutex_acquire(&cs->mlock);
    if (spdif_cs_is_state(cs->state, SPDIF_CS_ST_ACTIVE)) {
        printlk(LK_DEBUG, "already started\n");
        ret = ERR_BAD_STATE;
        goto done;
    }
    spdif_cs_set_state(&cs->state, SPDIF_CS_ST_ACTIVE);

    /* clear channel status buffers */
    spdif_cs_clear_cs_buff(cs);
    memset(cs->cs_data, 0, SPDIF_CS_DATA_SZ_BYTES);
    cs->redundancy_count = 0;

    /* SAI device to be opened */
    ret = spdif_cs_sai_open(cs->dev_sai);
    if (ret) {
        printlk(LK_ERR, "%s:%d: error opening device %d\n",
                __PRETTY_FUNCTION__, __LINE__, ret);
        goto done;
    }

    /* SAI to be configured according to SPDIF operation */
    cs->spdif_rate = round;
    cs->sai_rate   = sai_rate;
    cs->osr        = osr;
    spdif_cs_clear_state(&cs->state, SPDIF_CS_ST_SETUP);

done:
    mutex_release(&cs->mlock);

    /* start cs monitoring loop */
    if (ret == NO_ERROR)
        _spdif_cs_enable(cs);

    return ret;
}

int spdif_cs_stop(struct device *dev)
{
    struct spdif_cs_s *cs = dev->state;
    int ret = NO_ERROR;

    printlk(LK_INFO, "stop\n");

    if (!cs)
        return ERR_NOT_READY;

    mutex_acquire(&cs->mlock);
    if (!spdif_cs_is_state(cs->state, SPDIF_CS_ST_ACTIVE)) {
        printlk(LK_DEBUG, "already stopped\n");
        ret = ERR_BAD_STATE;
        goto done;
    }
    spdif_cs_clear_state(&cs->state, SPDIF_CS_ST_ACTIVE);

    /* stop cs monitoring loop */
    _spdif_cs_disable(cs);

    /* SAI device to be closed now */
    ret = spdif_cs_sai_close(cs->dev_sai);
    if (ret) {
        printlk(LK_ERR, "%s:%d: error closing device %d\n",
                __PRETTY_FUNCTION__, __LINE__, ret);
    }

done:
    mutex_release(&cs->mlock);

    return ret;
}


static int spdif_cs_sai_configure(struct device *dev_sai,
                                  unsigned sai_rate)
{
    int ret;

    sai_format_t sai_format = {
        .sai_protocol = 2, /* kSAI_BusI2S */
        .bitWidth = 32,
        .num_channels = 2,
        .num_slots = 2,
        .period_size = SPDIF_CS_DEFAULT_PERIOD_SIZE,
        .polarity = SAI_BITCLOCK_POLARITY_UNUSED,
        .master_slave = ksai_Master,
        .bitclock_source = kSAI_BClkSourceMclkDiv,
    };
    sai_format.sampleRate_Hz = sai_rate;

    ret = spdif_cs_sai_setup(dev_sai, &sai_format);
    if (ret) {
        printlk(LK_ERR, "%s:%d: sai setup failed (%d)\n",
                __PRETTY_FUNCTION__, __LINE__, ret);
        return ret;
    }

    return ret;
}

static int spdif_cs_osr_to_iec(char *osr_buff, size_t osr_sz,
                               char *biphase_buff, size_t biphase_sz,
                               char *cs_data_buff, size_t cs_data_sz,
                               unsigned osr)
{
    int biphase_cnt, index;
    uint8_t *biphase_start;

    /* Oversampled SAI RX to biphase conversion via edge detection */
    biphase_cnt = oversampleToBiphase((uint8_t *)osr_buff,
                                      osr_sz,
                                      (uint8_t *)biphase_buff,
                                      biphase_sz,
                                      osr);
    if (biphase_cnt < 0) {
        printlk(LK_INFO, "osr to biphase decoding error\n");
        return ERR_INVALID_ARGS;
    } else
        printlk(LK_DEBUG, "osr to biphase decoding:%d\n", biphase_cnt);

    /* Align biphase stream on a subframe bit 0 boundary (X/Y/Z preamble) */
    index = biphaseRecovery((uint8_t *)biphase_buff, biphase_cnt,
                                  &biphase_start);
    if (index < 0) {
        printlk(LK_INFO, "biphase preambles detection error\n");
        return ERR_INVALID_ARGS;
    } else
        printlk(LK_DEBUG, "biphase preambles detection: %d\n", index);


    /* Sanity check valid IEC samples available for full IEC block decoding */
    if (((char *)biphase_start - biphase_buff) + (SPDIF_CS_IEC_BLOCK_SZ_BYTES * 2) >
                 biphase_sz) {
        printlk(LK_INFO, "Not enough biphase samples after recovery, consummed = %ld\n",
                ((char *)biphase_start - biphase_buff));

        return ERR_INVALID_ARGS;
    }

    ASSERT(osr_sz >= SPDIF_CS_IEC_BLOCK_SZ_BYTES * 2); //iec_buff

    char *iec_buff = osr_buff;

    biphaseToSPDIF((uint8_t *)biphase_start, SPDIF_CS_IEC_BLOCK_SZ_BYTES * 2 /* biphase */,
                   (uint8_t *)iec_buff, SPDIF_CS_IEC_BLOCK_SZ_BYTES);

    /* Channel Status extraction from IEC block */
    spdifSubFrameChannelStatus((uint8_t *)iec_buff, SPDIF_CS_IEC_BLOCK_SZ_BYTES,
                               (uint8_t *)cs_data_buff, cs_data_sz);

    return NO_ERROR;
}

static bool spdif_cs_process_cs_buff(struct spdif_cs_s *cs) {
    char *next, *current;
    next    = spdif_cs_get_next_cs_buff(cs);
    current = spdif_cs_get_current_cs_buff(cs);
    bool update = false;

    /* compare new CS data with previous one */
    if (memcmp(next, current, SPDIF_CS_DATA_SZ_BYTES)) {
        update = true;
        spdif_cs_flip_cs_buff(cs);
    }

    return update;
}

static void spdif_cs_data_cb(void *arg)
{
    struct spdif_cs_s *cs = (struct spdif_cs_s *)arg;

    mutex_acquire(&cs->mlock);

    if (!spdif_cs_is_state(cs->state, SPDIF_CS_ST_ACTIVE))
        goto skip;

    if (cs->callback)
        cs->callback(cs->cookie, SPDIF_CS_ST_CHANNEL_STATUS_UPDATE,
                     cs->cs_data, SPDIF_CS_DATA_SZ_BYTES);

    cs->count_update++;
skip:
    mutex_release(&cs->mlock);
}

static int spdif_cs_thread(void *arg)
{
    printlk(LK_DEBUG, "Entering spdif_cs thread\n");

    int ret;
    size_t osr_bytes, biphase_bytes;
    struct spdif_cs_s *cs;
    cs = (struct spdif_cs_s *)arg;
    bool update, notify;
    void *current;

    cs->running = true;

    while (cs->running) {

        event_wait(&cs->event);

        /*
         * SAI setup operated within locked in case driver is concurrently
         * stopped and SAI driver closed.
         */
        mutex_acquire(&cs->mlock);
        if (!spdif_cs_is_state(cs->state, SPDIF_CS_ST_ACTIVE)) {
            mutex_release(&cs->mlock);
            goto done;
        }
        /* TODO: SAI bug - setup() needed before (re)start() */
        if (true) /*(!spdif_cs_is_state(cs->state, SPDIF_CS_ST_SETUP))*/ {
            ret = spdif_cs_sai_configure(cs->dev_sai, cs->sai_rate);
            if (ret) {
                printlk(LK_ERR, "%s:%d: sai configuration failed (%d)\n",
                        __PRETTY_FUNCTION__, __LINE__, ret);
                mutex_release(&cs->mlock);
                goto done;
            }
            spdif_cs_set_state(&cs->state, SPDIF_CS_ST_SETUP);
        }
        mutex_release(&cs->mlock);

        /* oversampled SPDIF RX line acquisition - read up-rounded number to match DMA grain*/
        printlk(LK_DEBUG, "SPDIF RX sample rate:%d osr:%d\n", cs->spdif_rate, cs->osr);
        osr_bytes = SPDIF_CS_OSR_BUFFER_SZ_BYTES;

        /*
         * SAI acquisition operated within locked in case driver is concurrently
         * stopped and SAI driver closed.
         */
        mutex_acquire(&cs->mlock);
        if (!spdif_cs_is_state(cs->state, SPDIF_CS_ST_ACTIVE)) {
            mutex_release(&cs->mlock);
            goto done;
        }
        ret = spdif_cs_read_sai(cs->dev_sai, cs->osr_buff, osr_bytes);
        if (ret) {
            printlk(LK_ERR, "%s:%d: sai read failed (%d)\n",
                    __PRETTY_FUNCTION__, __LINE__, ret);
            mutex_release(&cs->mlock);
            goto done;
        }
        mutex_release(&cs->mlock);

        /* clip number of samples to actual need (2 blocks) by current osr */
        osr_bytes     = 2 * SPDIF_CS_IEC_BLOCK_SZ_BYTES * 2 * cs->osr;
        ASSERT(osr_bytes <= SPDIF_CS_OSR_BUFFER_SZ_BYTES);
        biphase_bytes = 2 * SPDIF_CS_IEC_BLOCK_SZ_BYTES * 2;
        ASSERT(biphase_bytes <= SPDIF_CS_BIPHASE_BUFFER_SZ_BYTES);

        char *cs_buff = spdif_cs_get_next_cs_buff(cs);
        ret = spdif_cs_osr_to_iec(cs->osr_buff, osr_bytes,
                                  cs->biphase_buff, biphase_bytes,
                                  cs_buff, SPDIF_CS_DATA_SZ_BYTES,
                                  cs->osr);
        if (ret) {
            printlk(LK_DEBUG, "decoding error\n");
            cs->count_error++;
            goto done;
        }

        /* Check for channel status data update */
        update = spdif_cs_process_cs_buff(cs);
        current = spdif_cs_get_current_cs_buff(cs);
        notify = false;
        if (update)
            cs->redundancy_count = 0;

        if (cs->redundancy_count < cs->redundancy) {
            cs->redundancy_count++;
            if ((cs->redundancy_count == cs->redundancy) &&
                (memcmp(cs->cs_data, current, SPDIF_CS_DATA_SZ_BYTES)))
                notify = true;
        }

        if (notify) {
            mutex_acquire(&cs->mlock);
            memcpy(cs->cs_data,
                   current,
                   SPDIF_CS_DATA_SZ_BYTES);
            mutex_release(&cs->mlock);
            dpc_queue(spdif_cs_data_cb, cs, 0);
        }

        cs->count_decode++;

done:
#ifdef SPDIF_CS_PRINT_STATS
        printlk(LK_NOTICE, "%s:%d: decoded:%d updates:%d errors:%d\n",
                __PRETTY_FUNCTION__, __LINE__,
                cs->count_decode, cs->count_update, cs->count_error);
#endif
        thread_sleep(cs->polling_rate_ms);
    }

    return 0;
}


/* default event callback - used only in autostart (test) mode */
void spdif_cs_autostart_cb(void *cookie, enum spdif_cs_event evt,
                                  void *data, size_t data_size)
{
    ASSERT(evt == SPDIF_CS_ST_CHANNEL_STATUS_UPDATE);
    ASSERT(data_size == SPDIF_CS_DATA_SZ_BYTES);
    printlk(LK_NOTICE, "%s:%d: cs update:\n", __PRETTY_FUNCTION__, __LINE__);
    hexdump(data, data_size);
}


static status_t spdif_cs_init(struct device *dev)
{
    struct spdif_cs_s *cs;

    printlk(LK_NOTICE, "%s:%d: Initializing spdif_cs\n",
            __PRETTY_FUNCTION__, __LINE__);

    cs = calloc(1, sizeof(struct spdif_cs_s));
    if (!cs)
        return ERR_NO_MEMORY;

    /* Get the polling rate of the SPDIF CS decoding algorithm */
    cs->polling_rate_ms = SPDIF_CS_DEFAULT_POLLING_RATE;
    of_device_get_int32(dev, "polling-rate-ms", &cs->polling_rate_ms);

    /* Get number of redundant CS data decoding before reporting */
    cs->redundancy = SPDIF_CS_DEFAULT_REDUNDANCY;
    of_device_get_int32(dev, "redundancy", &cs->redundancy);
    cs->redundancy_count = 0;

    /* SAI device configuration */
    cs->dev_sai = of_device_lookup_device(dev, "sai-dev");
    if (!cs->dev_sai) {
        printlk(LK_ERR, "%s:%d: no valid sai device\n",
                __PRETTY_FUNCTION__, __LINE__);
        free(cs);
        return ERR_INVALID_ARGS;
    }

    mutex_init(&cs->mlock);
    event_init(&cs->event, false, 0);
    cs->thread = thread_create("spdif_cs", spdif_cs_thread,
                        cs, SPDIF_CS_THREAD_PRIO, SPDIF_CS_STACK_SIZE);

    thread_detach_and_resume(cs->thread);

    dev->state = cs;

#ifdef SPDIF_CS_AUTOSTART
    spdif_cs_register(dev, cs, spdif_cs_autostart_cb);
    spdif_cs_start(dev, SPDIF_CS_DEFAULT_SPDIF_RATE);
#endif

    return 0;
}

static struct driver_ops the_ops = {
    .device_class = NULL,
    .init = spdif_cs_init,
};

DRIVER_EXPORT_WITH_LVL(spdif_cs, &the_ops, DRIVER_INIT_TARGET);
