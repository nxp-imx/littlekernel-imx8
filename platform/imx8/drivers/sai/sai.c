/*
 * The Clear BSD License
 * Copyright 2018-2021 NXP
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

#include <compiler.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "sai_hw.h"
#include "math.h"

#include <kernel/spinlock.h>
#include <kernel/semaphore.h>
#include <kernel/event.h>
#include <kernel/vm.h>

#include <lib/appargs.h>
#include <dev/interrupt.h>
#include <dev/class/sai.h>

#include <lib/cbuf.h>
#include <assert.h>
#include <err.h>

#include <dev/dma.h>

#include <sys/ioctl.h>
#include <dev/sai_ioctl.h>
#include <platform/clock.h>
#include <platform.h>
#include <math.h>
#include <debug.h>

#include <platform/imx_common.h>

#define SAI_DMA_PAYLOAD_PATTERN 0x7F
#define SAI_DMA_PREFILL_PATTERN 0x0
#define SAI_FIFO_PREFILL_PATTERN 0
#define SAI_HW_MAX_LANE 8

#define IMX_SAI_FIFO_PREFILL_PIO_DEFAULT 512

#if !defined(IMX_SAI_WARMUP_NR_PERIODS) || IMX_SAI_WARMUP_NR_PERIODS == 0
#define IMX_SAI_WARMUP_NR_PERIODS 0

#ifndef IMX_SAI_PREFILL_WITH_ZERO
#define IMX_SAI_PREFILL_WITH_ZERO 0
// we are here if IMX_SAI_WARMUP_NR_PERIODS=0 && IMX_SAI_WARMUP_NR_PERIODS_FRAC=0 in imx8mm-evk-af-base.mk
#endif

#else
/*
 * Frames in fifo is half of its depth, 2 channels per fifo
 * Assuming a samples per fifo stage
 * */
#define IMX_SAI_PREFILL_WITH_ZERO (FSL_FEATURE_SAI_FIFO_COUNT / 2)
#endif

#ifndef IMX_SAI_WARMUP_NR_PERIODS_FRAC
#define IMX_SAI_WARMUP_NR_PERIODS_FRAC 0
#endif

#ifndef IMX_SAI_READ_SILENT_RETRY
#define IMX_SAI_READ_SILENT_RETRY 1
#endif

#ifndef IMX_SAI_PERMISSIVE
#define IMX_SAI_PERMISSIVE 0
#endif

#ifndef IMX_SAI_READ_AUTOSTART
#define IMX_SAI_READ_AUTOSTART 1
#endif

#define IMX_SAI_SILENCE_INJECT_PRINT_RATE 1000
#define IMX_SAI_SILENCE_INJECTION_TIMEOUT_MS 4 /* 2 > x  > 1 OM periods*/

#define IMX_SAI_DMA_NR_PERIOD (8)   /* number of dma buffers */
#define IMX_SAI_DMA_PERIOD_RATIO (4) /* one dma period = fmt_period / ratio */

#define DMA_BUFFER_ALIGNMENT  (128)

/* use vmm_alloc_contiguous for DMA buffers */
#define DMA_BUFFER_MEMALIGN_TYPE 0

#ifndef DMA_BUFFER_MEMALIGN_TYPE
#define DMA_BUFFER_MEMALIGN_TYPE 1
#endif

/*
 *  If IMX_SAI_EMPTY_RXFIFO is defined to 1,
 *  the driver will empty the FIFO while reading it..
 *  Not only reading the watermark size.
 *
 */
#ifndef IMX_SAI_EMPTY_RXFIFO
#define IMX_SAI_EMPTY_RXFIFO 1
#endif

/*
 * Send by default, 4 OM periods (~10ms)
 * of silence at end of flush sequence
 *
 */
#ifndef IMX_SAI_DMA_SILENCE_N_PERIOD_ON_FLUSH
#define IMX_SAI_DMA_SILENCE_N_PERIOD_ON_FLUSH 4
#endif

struct dma_driver_data {
    semaphore_t dma_period_sem;
    thread_t *dma_thread;
    struct dma_descriptor *dma_desc;
    uint64_t dma_period_elapsed;
    void *dma_buffer;
    paddr_t dma_buffer_phys;
    unsigned dma_nr_periods;
    unsigned dma_period_size; /* current period size */
#define SAI_FRMT_MAX_PERIOD 512
/* DMA_MAX_PERIOD = SAI_FRMT_MAX_PERIOD * 32 bits * max channels */
#define SAI_DMA_RX_MAX_PERIOD (SAI_FRMT_MAX_PERIOD * 4 * 8) / IMX_SAI_DMA_PERIOD_RATIO
#define SAI_DMA_TX_MAX_PERIOD (SAI_FRMT_MAX_PERIOD * 4 * 32) / IMX_SAI_DMA_PERIOD_RATIO
    unsigned dma_period_size_max; /* max period size */
};

struct imx_period_s {
    u64 xfer_done;
    u64 cnt;
    unsigned size;
    unsigned frame_size;
    unsigned warmup_size;
    unsigned frames;
    float warmup_ratio;
    unsigned fifo_prefill;
    unsigned dma_period_prefill;
    u64 ts;
    u64 duration;
    uint32_t erate; /* estimated rate */
};

enum imx_sai_hw_state {
    kSAI_hw_Up           = (1UL << 1),   /*!< HW is powered up. */
    kSAI_hw_Prepared     = (1UL << 2),   /*!< HW is prepared  */
    kSAI_hw_Opened       = (1UL << 3),   /*!< HW is on. */
    kSAI_hw_Started      = (1UL << 4),   /*!< HW is on. IRQ should be flowing */
    kSAI_hw_Counting     = (1UL << 5),   /*!< HW is on. bit counting */
    kSAI_hw_Flushing     = (1UL << 6),   /*!< HW is on. IRQ flowing, flushing */
    kSAI_hw_Flushed      = (1UL << 7),   /*!< HW is on. IRQ stopped, flushed */
    kSAI_hw_Running      = (1UL << 8),   /*!< HW is on. IRQ flowing */
};

struct imx_sai_state {
    int bus_id;
    bool is_dummy_rx;
    bool is_dummy_tx;
    I2S_Type *io_base;
    spin_lock_t lock;
    struct device *device;
    struct device *clock_dev;
    struct device *rx_sai_chained_dev;
    struct device *tx_sai_chained_dev;
    bool rx_is_chained;
    bool tx_is_chained;
    unsigned rx_lanes;
    unsigned tx_lanes;
    struct list_node node;
    uint32_t clk_mode;
    bool mclk_as_output;
    struct gpio_desc gpio_mclk_ctrl;
    uint32_t mclk_select_rx;
    uint32_t mclk_select_tx;
#define SAI_MCLK_CONFIG 3
#define SAI_MCLK_CONFIG_44K 0
#define SAI_MCLK_CONFIG_48K 1
#define SAI_MCLK_CONFIG_32K 2
    struct device_cfg_clk *clk_tx[SAI_MCLK_CONFIG];
    const char *mclk_tx_config[SAI_MCLK_CONFIG];
    struct device_cfg_clk *clk_rx[SAI_MCLK_CONFIG];
    const char *mclk_rx_config[SAI_MCLK_CONFIG];
    unsigned mclk_tx_rate;
    unsigned mclk_rx_rate;
    bool rx_is_slave;
    bool tx_is_slave;
    sai_sync_mode_t rx_sync_mode;
    sai_sync_mode_t tx_sync_mode;
    uint32_t rx_fcomb;
    uint32_t tx_fcomb;
    cbuf_t rx_circ_buf;
    cbuf_t tx_circ_buf;
#define SAI_N_PERIODS 256
/* Max bitwidth * FIFO COUNT * MAX CHANNEL */
#define SAI_RX_BUFFER_SIZE (4 * FSL_FEATURE_SAI_FIFO_COUNT * 8)
#define SAI_TX_BUFFER_SIZE (4 * FSL_FEATURE_SAI_FIFO_COUNT * 16)
/*
 * 512kB --> 85ms for 8ch/32bit/192kHz
 * */
#define SAI_MAX_RX_CIRC_BUF_SIZE (512 * 1024)
/*
 * 1MB --> 85.3 ms for 16ch/32bit/192kHz
 * Must be consistent with per default per channel delay buffer size (85ms)
 */
#define SAI_MAX_TX_CIRC_BUF_SIZE (1024 * 1024)
    size_t rx_circ_buf_size;
    size_t tx_circ_buf_size;
    size_t tx_circ_buf_max_size;
    struct imx_hw_state_s rx_hw_state;
    struct imx_hw_state_s tx_hw_state;
    spin_lock_t rx_lock;
    spin_lock_t rx_bitcnt_lock;
    spin_lock_t tx_lock;
    spin_lock_t tx_bitcnt_lock;
    mutex_t rx_mutex;
    mutex_t tx_mutex;
    event_t rx_wait;
    event_t tx_wait;
    /* stats */
    unsigned irq_stats;
    unsigned irq_stats_tx;
    unsigned irq_stats_rx;
    unsigned silence_accumulated_tx;
    bool silence_injection_ongoing;
    size_t silence_injection_len;
    unsigned silence_injection_count;
    lk_bigtime_t silence_last_ts;
    /* Interface with SAI HW */
    sai_handle_t sai_rx_handle;
    sai_transfer_format_t sai_rx_fmt;
    sai_handle_t sai_tx_handle;
    sai_transfer_format_t sai_tx_fmt;
    struct dma_driver_data dma_rx_driver_data;
    struct dma_driver_data dma_tx_driver_data;
    spin_lock_t rx_cb_lock;
    spin_lock_t tx_cb_lock;
    void * rx_cb_cookie;
    void * tx_cb_cookie;
    sai_cb_t rx_cb;
    sai_cb_t tx_cb;
    struct imx_period_s rx_period;
    struct imx_period_s tx_period;
    bool flushed_with_silence;
    uint32_t flush_silence_periods;
};

static status_t imx_sai_set_timestamp(struct device *dev,
                                        struct sai_ioc_cmd_timestamp *ts);

static inline void imx_sai_reset_period(struct imx_period_s *period, unsigned frame_size)
{
    period->xfer_done = 0;
    period->cnt = 0;
    period->warmup_size = 0;
    period->frame_size = frame_size;
    period->ts = 0;
    period->duration = 0;
    period->erate = 0;
}

static inline bool imx_sai_period_add_xfer(struct imx_period_s *period, unsigned len)
{
    u64 next_period = (period->cnt + 1 ) * period->size;
    period->xfer_done += len;
    if (period->xfer_done >= next_period) {
        period->cnt += 1;
        return true;
    }
    return false;
}

static unsigned imx_sai_get_warmup_frames(struct imx_period_s *period,
                                           void *buf, size_t xfer_len, bool fill)
{
#if defined IMX_SAI_WARMUP_NR_PERIODS && (IMX_SAI_WARMUP_NR_PERIODS > 0)
    if (unlikely(period->warmup_size == 0)) {
        period->warmup_size = IMX_SAI_WARMUP_NR_PERIODS * period->size;
#if defined(IMX_SAI_WARMUP_NR_PERIODS_FRAC)  && (IMX_SAI_WARMUP_NR_PERIODS_FRAC > 0)
        period->warmup_size += period->size / IMX_SAI_WARMUP_NR_PERIODS_FRAC;
#endif
        unsigned fifo_warmup_size = period->fifo_prefill * period->frame_size;
        printlk(LK_VERBOSE, "Warmup: (size / fifo / samples / periods / frac) [%d %d %d %d %d]\n",
            period->warmup_size, fifo_warmup_size, period->warmup_size / period->frame_size,
            IMX_SAI_WARMUP_NR_PERIODS, IMX_SAI_WARMUP_NR_PERIODS_FRAC);
        DEBUG_ASSERT(period->warmup_size >= fifo_warmup_size);
        period->warmup_size -= fifo_warmup_size;
    }

    if (likely(period->xfer_done > period->warmup_size))
        return 0;

    unsigned remaining = MIN(period->warmup_size - period->xfer_done, xfer_len);

    if (fill)
        memset(buf, 0, remaining);

    return remaining;
#else
    return 0;
#endif
}

static int imx_sai_notify_silence(struct imx_sai_state *state, uint64_t cnt)
{
    void *cookie = state->tx_cb_cookie;
    sai_cb_t cb = state->tx_cb;

    sai_evt_silence_t evt_silence = {
        .cnt = cnt,
    };

    printlk(LK_VERBOSE, "SAI%d: Silence injection (%llu)\n", state->bus_id, cnt);

    if (!cb)
        return 0;

    return cb(SAI_EVENT_SILENCE , &evt_silence, cookie);
}

static int imx_sai_notify_period(struct imx_sai_state *state, bool is_rx)
{
    void *cookie;
    struct imx_period_s *period;
    sai_cb_t cb;

    if (is_rx) {
        period = &state->rx_period;
        cookie = state->rx_cb_cookie;
        cb = state->rx_cb;
    } else {
        period = &state->tx_period;
        cookie = state->tx_cb_cookie;
        cb = state->tx_cb;
    }

    if (!cb)
        return 0;

    u64 period_elapsed_cnt = period->cnt;

    sai_evt_period_elapsed_t evt_period_elapsed = {
        .cnt = period_elapsed_cnt,
    };

    printlk(LK_VERBOSE, "Period elapsed (%s %d: %llu (%d)\n",
                            is_rx ? "RX" : "TX", state->bus_id,
                            period_elapsed_cnt, period->size);

    return cb(SAI_EVENT_PERIOD_ELAPSED , &evt_period_elapsed, cookie);
}

static inline unsigned imx_sai_tx_get_warmup_frames(struct imx_sai_state *state,
                                           void *buf, size_t xfer_len)
{
    if (state->sai_tx_handle.dma_enable == false)
        return imx_sai_get_warmup_frames(&state->tx_period,
                buf, xfer_len, true);

    return 0;
}

#define I2S_RX_FR_PENDING (I2S_RCSR_FRIE_MASK | I2S_RCSR_FRF_MASK)
#define I2S_RX_FW_PENDING (I2S_RCSR_FWIE_MASK | I2S_RCSR_FWF_MASK)
#define I2S_RX_FE_PENDING (I2S_RCSR_FEIE_MASK | I2S_RCSR_FEF_MASK)
#define I2S_RX_SE_PENDING (I2S_RCSR_SEIE_MASK | I2S_RCSR_SEF_MASK)
#define I2S_RX_WS_PENDING (I2S_RCSR_WSIE_MASK | I2S_RCSR_WSF_MASK)

#define I2S_RX_STATUS_MASK(x) (\
    (x & \
        ( \
            I2S_RCSR_FRF_MASK | I2S_RCSR_FWF_MASK \
            | I2S_RCSR_FEF_MASK | I2S_RCSR_SEF_MASK \
            | I2S_RCSR_WSF_MASK\
        ) \
    ) >> I2S_RCSR_FRF_SHIFT)

#define I2S_RX_ENABLE_MASK(x) (\
    (x & \
        ( \
            I2S_RCSR_FRIE_MASK | I2S_RCSR_FWIE_MASK \
            | I2S_RCSR_FEIE_MASK | I2S_RCSR_SEIE_MASK \
            | I2S_RCSR_WSIE_MASK\
        ) \
    ) >> I2S_RCSR_FRIE_SHIFT)

#define I2S_TX_FR_PENDING (I2S_TCSR_FRIE_MASK | I2S_TCSR_FRF_MASK)
#define I2S_TX_FW_PENDING (I2S_TCSR_FWIE_MASK | I2S_TCSR_FWF_MASK)
#define I2S_TX_FE_PENDING (I2S_TCSR_FEIE_MASK | I2S_TCSR_FEF_MASK)
#define I2S_TX_SE_PENDING (I2S_TCSR_SEIE_MASK | I2S_TCSR_SEF_MASK)
#define I2S_TX_WS_PENDING (I2S_TCSR_WSIE_MASK | I2S_TCSR_WSF_MASK)


#define I2S_TX_STATUS_MASK(x) (\
    (x & \
        ( \
            I2S_TCSR_FRF_MASK | I2S_TCSR_FWF_MASK \
            | I2S_TCSR_FEF_MASK | I2S_TCSR_SEF_MASK \
            | I2S_TCSR_WSF_MASK\
        ) \
    ) >> I2S_TCSR_FRF_SHIFT)

#define I2S_TX_ENABLE_MASK(x) (\
    (x & \
        ( \
            I2S_TCSR_FRIE_MASK | I2S_TCSR_FWIE_MASK \
            | I2S_TCSR_FEIE_MASK | I2S_TCSR_SEIE_MASK \
            | I2S_TCSR_WSIE_MASK\
        ) \
    ) >> I2S_TCSR_FRIE_SHIFT)

#define IMX_SAI_FIFO_WATERMARK_RX   (FSL_FEATURE_SAI_FIFO_COUNT / 2)
#define IMX_SAI_FIFO_WATERMARK_TX   (FSL_FEATURE_SAI_FIFO_COUNT / 2)
#define IMX_SAI_PERIOD_SIZE_TX      (IMX_SAI_FIFO_WATERMARK_TX / 2)

static bool imx_sai_pop_from_tx_circ_buffer(struct imx_sai_state *state,
                                           void *buf, size_t xfer_len)
{
    sai_handle_t *handle = &state->sai_tx_handle;
    bool reschedule = false;
    size_t read = 0;
    static unsigned prev_state = 0;
    struct imx_hw_state_s *hw_state = &state->tx_hw_state;
    bool inject_silence;
    size_t cbuf_level, injection_threshold;


    if (prev_state != hw_state->state) {
        printlk(LK_VERBOSE, "sai%d hw state: (%x) -> (%x)\n",
                state->bus_id, prev_state, hw_state->state);
        prev_state = hw_state->state;
    }

    /* DMA case : Inject silence if original dma prefill amount is not anymore present in circular buffer */
    cbuf_level = cbuf_space_used(&state->tx_circ_buf);
    injection_threshold = state->tx_period.dma_period_prefill
            * state->dma_tx_driver_data.dma_period_size;
    inject_silence = (cbuf_level < injection_threshold)
            && (state->sai_tx_handle.dma_enable)
            && (handle->dma_mode != kSAI_DMA_Private_Buffer);

    read = imx_sai_tx_get_warmup_frames(state, buf, xfer_len);
    if (read < xfer_len) {
        printlk(LK_VERBOSE, "sai%d hw state: (%x), Poping %ld from tx circular buffer\n",
                state->bus_id, handle->state, xfer_len);
        read += cbuf_read(&state->tx_circ_buf, buf + read, xfer_len - read, false);
    }

    if ((read != xfer_len) || inject_silence) {
        if (!state->silence_injection_ongoing) {
            ASSERT(!state->silence_injection_len);
            ASSERT(!state->silence_injection_count);
            state->silence_injection_ongoing = true;
        }

        //FIXME: Flush should be done before warmup
        struct imx_hw_state_s *hw_state = &state->tx_hw_state;
        if (imx_is_hw_state(hw_state, kSAI_hw_Flushing)) {

            if (state->flushed_with_silence == false) {
                /*
                 * Some speakers (might ?)requires significant amount of silences
                 * before to switch down the frame clk.
                 *
                 * Customer reported those injected silences to clean a pop
                 * noise at end of stream. Nevertheless, the fade out is likely
                 * not applied because of IMXAF-2547 which might be the root
                 * cause.
                 * Compiling with IMX_SAI_DMA_SILENCE_N_PERIOD_ON_FLUSH set to 0
                 * will not apply any silence at end of flush.
                 *
                 * */
                size_t zeros_flushed;
                if (state->sai_tx_handle.dma_enable)
                    zeros_flushed = IMX_SAI_DMA_PERIOD_RATIO
                            * state->dma_tx_driver_data.dma_period_size;
                else
                    zeros_flushed = state->tx_period.size;

                zeros_flushed *= state->flush_silence_periods;

                printlk(LK_NOTICE, "SAI%d: Injecting %zu Bytes as ending zeros\n",
                                                state->bus_id, zeros_flushed);
                cbuf_write(&state->tx_circ_buf, NULL, zeros_flushed, false);
            } else {
                printlk(LK_NOTICE, "%s:%d: SAI%d: Initiating flushing sequence, missing (%ld)\n",
                        __PRETTY_FUNCTION__, __LINE__, state->bus_id,
                        xfer_len - read);
                /* Disable interrupt here */
                I2S_Type *base = state->io_base;
                /* FSL_FEATURE_SAI_FIFO_COUNT */
    #if defined(FSL_FEATURE_SAI_FIFO_COUNT) && (FSL_FEATURE_SAI_FIFO_COUNT > 1)
                SAI_TxDisableInterrupts(base, kSAI_FIFORequestInterruptEnable);
    #else
                SAI_TxDisableInterrupts(base, kSAI_FIFOWarningInterruptEnable);
    #endif

                if (state->sai_tx_handle.dma_enable) {
                    printlk(LK_INFO, "SAI%d: Disable DMA Tx FifoRequest\n",
                                                        state->bus_id);
                    SAI_TxEnableDMA(base, kSAI_FIFORequestDMAEnable, false);
                    dma_pause_channel(handle->dma_chan);
                }

                printlk(LK_INFO, "SAI%d: Injected %d silence during stream\n",
                                                        state->bus_id, state->silence_accumulated_tx);
                state->silence_accumulated_tx = 0;
                reschedule = true;
            }

            state->flushed_with_silence = true;
        } else {
#if defined(IMX_SAI_PERMISSIVE) && (IMX_SAI_PERMISSIVE == 1)
            size_t inject_len;
            size_t skip_len = xfer_len - read;

            /* Determine silence injection length */
            if ( inject_silence )
                inject_len = IMX_SAI_DMA_PERIOD_RATIO * state->dma_tx_driver_data.dma_period_size;
            else
                inject_len = skip_len;

            /* Keep track of injected silence amount */
            state->silence_accumulated_tx++;
            state->silence_injection_count++;
            state->silence_injection_len += inject_len;
            state->silence_last_ts = current_time_hires();

            if (0 == state->silence_injection_count % IMX_SAI_SILENCE_INJECT_PRINT_RATE) {
                printlk(LK_WARNING, "SAI%d: silence injection on-going... %d injections for %zuB already.\n",
                       state->bus_id,
                       state->silence_injection_count,
                       state->silence_injection_len);
            }

            switch (handle->dma_mode) {
            case kSAI_DMA_Zerocopy_Cached:
            case kSAI_DMA_Zerocopy_Uncached:
                ASSERT(!(skip_len % state->dma_tx_driver_data.dma_period_size));

                /* Write (SW) */
                cbuf_write(&state->tx_circ_buf, NULL, inject_len, false);
                /* Read (DMA) */
                if (skip_len) {
                    cbuf_skip(&state->tx_circ_buf, false, skip_len);
                    printlk(LK_NOTICE, "%s:%d: Realigned DMA by skipping %zu bytes.\n",
                            __PRETTY_FUNCTION__, __LINE__, skip_len);
                }
                break;
            default:
                memset(buf + read, 0, inject_len);
                break;
            }

            reschedule = true;
            imx_sai_notify_silence(state, inject_len);
#else
            ASSERT(read == xfer_len);
#endif
        }
    } else {
        if ( (state->silence_injection_ongoing) &&
             (state->silence_injection_len) &&
             (current_time_hires() > (state->silence_last_ts + IMX_SAI_SILENCE_INJECTION_TIMEOUT_MS * 1000)) ) {
            printlk(LK_WARNING, "SAI%d: Injected a total length of %zuB silence for %d fifo request\n",
                                        state->bus_id,
                                        state->silence_injection_len,
                                        state->silence_injection_count);
            /* Clear data for next round */
            state->silence_injection_ongoing = false;
            state->silence_injection_count = 0;
            state->silence_injection_len = 0;
        }
    }

    return reschedule;
}


static bool imx_sai_push_to_rx_circ_buffer(struct imx_sai_state *state,
                                           void *buf, size_t xfer_len)
{
    sai_handle_t *handle = &state->sai_rx_handle;
    bool reschedule = false;
    size_t written = 0;
    printlk(LK_VERBOSE, "hw state: (%x), Pushing %ld to rx circular buffer\n",
            handle->state, xfer_len);

#if defined(IMX_SAI_PERMISSIVE) && (IMX_SAI_PERMISSIVE == 1)
    size_t avail = cbuf_space_avail(&state->rx_circ_buf);
    if (xfer_len > avail) {
        printlk(LK_NOTICE, "%s:%d: SAI%d: No more room in circ buffer (avail %ld). Dropping %ld bytes\n",
                __PRETTY_FUNCTION__, __LINE__, state->bus_id, avail, xfer_len);
            cbuf_reset(&state->rx_circ_buf);
        }
    written =  cbuf_write(&state->rx_circ_buf, buf, xfer_len, false);
#else
    written =  cbuf_write(&state->rx_circ_buf, buf, xfer_len, false);
    ASSERT(written == xfer_len);
#endif

    if (handle->xfer_remaining > 0) {
        handle->xfer_remaining -= written;
        if ((handle->xfer_remaining <= 0) ||
            (cbuf_space_used(&state->rx_circ_buf) > handle->cbuf_watermark)) {
            handle->xfer_remaining = MAX(0, handle->xfer_remaining);
            event_signal(&state->rx_wait, false);
            reschedule = true;
        }
    }
    return reschedule;
}

static inline bool is_imx_sai_isr_xx_pending(unsigned status, unsigned flags)
{
    return ((status & flags) == flags);
}

static status_t imx_sai_rx_stop_hw(struct imx_sai_state *state)
{
    I2S_Type *base = state->io_base;
    ASSERT(base);

    sai_handle_t *handle = &state->sai_rx_handle;
    ASSERT(handle);

    struct imx_hw_state_s *hw_state = &state->rx_hw_state;

    if (!imx_is_hw_state(hw_state, kSAI_hw_Running))
        return 0;

    printlk(LK_INFO, "Stopping Sai rx hardware\n");

    SAI_TransferAbortReceive(base, handle);
    SAI_RxSoftwareReset(base, kSAI_ResetTypeSoftware);

    imx_clear_hw_state(hw_state, kSAI_hw_Running);

    return 0;
}

static bool imx_sai_isr_tx(struct imx_sai_state *state)
{
    int reschedule = 0;
    size_t xfer_len = 0;

    I2S_Type *base = state->io_base;
    ASSERT(base);

    sai_handle_t *handle = &state->sai_tx_handle;
    ASSERT(handle);

    unsigned pending = 0;

    struct imx_hw_state_s *hw_state = &state->tx_hw_state;
    spin_lock(&state->tx_lock);

    unsigned status = base->TCSR;

    printlk(LK_VERBOSE, "ISR TX status: %x\n", status);

    /* Do we have a TX interrupt enabled pending */
    if (!(I2S_TX_ENABLE_MASK(status) & I2S_TX_STATUS_MASK(status))) {
        reschedule = 0;
        goto exit_tx_isr;
    }

    state->irq_stats_tx++;

    if (is_imx_sai_isr_xx_pending(status, I2S_TX_WS_PENDING)) {
        printlk(LK_NOTICE, "%s:%d: SAI%d: Word Start Interrupt pending\n",
                __PRETTY_FUNCTION__, __LINE__, state->bus_id);
        pending |= I2S_TCSR_WSF_MASK;
    }

    if (is_imx_sai_isr_xx_pending(status, I2S_TX_SE_PENDING)) {
        printlk(LK_ERR, "%s:%d: SAI%d: Sync Error Interrupt pending\n",
                __PRETTY_FUNCTION__, __LINE__, state->bus_id);
        pending |= I2S_TCSR_SEF_MASK;
    }

    if (is_imx_sai_isr_xx_pending(status, I2S_TX_FE_PENDING)) {
        pending |= I2S_TCSR_FEF_MASK;
        if (imx_is_hw_state(hw_state, kSAI_hw_Flushing)) {
            /* kSAI_FIFOWarningInterruptEnable */
            SAI_TxDisableInterrupts(base, kSAI_FIFOErrorInterruptEnable);

            printlk(LK_NOTICE, "%s:%d: SAI%d: Flushing completed\n",
                    __PRETTY_FUNCTION__, __LINE__, state->bus_id);
            if (state->tx_cb) {
                reschedule |= state->tx_cb(SAI_EVENT_DRAIN,
                                            NULL, state->tx_cb_cookie);
            }
            imx_clear_hw_state(hw_state, kSAI_hw_Flushing);
//            imx_set_hw_state(hw_state, kSAI_hw_Flushed);
        } else {
            u64 timestamp = current_time_hires();
            printlk(LK_ERR, "%s:%d: [%05d.%06d] SAI%d: FIFO Error Underrun (%d/%d/%d/%d)\n",
                __PRETTY_FUNCTION__,
                __LINE__,
                (int) (timestamp / 1000000ULL),
                (int) (timestamp % 1000000ULL),
                state->bus_id,
                arch_curr_cpu_num(), state->irq_stats,
                state->irq_stats_rx, state->irq_stats_tx);
        }
    }

    /* Clear the interrupt pending */
    SAI_TxClearStatusFlags(base, pending);
    if (pending & I2S_TCSR_FEF_MASK) {
        SAI_TxSoftwareReset(base, I2S_TCSR_FR_MASK);
        goto exit_tx_isr;
    }

    /* FIXME: Notify client through callback in case of error ? */

    if (state->sai_tx_handle.dma_enable) {
        printlk(LK_ERR, "%s:%d: SAI%d: Got unexpected error\n",
                __PRETTY_FUNCTION__, __LINE__, state->bus_id);
        goto exit_tx_isr;
    }

    /* Handle the DATA */
    /* We assume the period size to be modulo the watermak */
#if defined(FSL_FEATURE_SAI_FIFO_COUNT) && (FSL_FEATURE_SAI_FIFO_COUNT > 1)
    if (is_imx_sai_isr_xx_pending(status, I2S_TX_FR_PENDING)) {
        xfer_len = handle->burst_length;
    }
#else
    if (is_imx_sai_isr_xx_pending(status, I2S_TX_FW_PENDING)) {
        xfer_len = handle->bitWidth / 8U;
    }
#endif
    ASSERT(xfer_len <= handle->buf_size);

    bool period_elapsed = imx_sai_period_add_xfer(&state->tx_period, xfer_len);
    if (period_elapsed)
        reschedule |= imx_sai_notify_period(state, false);

    if (xfer_len) {
        reschedule |= (int) imx_sai_pop_from_tx_circ_buffer(state, handle->buf, xfer_len);
        SAI_WriteNonBlocking(base, handle->channel, handle->bitWidth, handle->buf, xfer_len);
    }

exit_tx_isr:
    spin_unlock(&state->tx_lock);

    return (reschedule != 0);
}

static bool imx_sai_isr_rx(struct imx_sai_state *state)
{
    bool reschedule = false;
    size_t xfer_len = 0;

    I2S_Type *base = state->io_base;
    ASSERT(base);

    sai_handle_t *handle = &state->sai_rx_handle;
    ASSERT(handle);

    struct imx_hw_state_s *hw_state = &state->rx_hw_state;
    unsigned pending = 0;

    spin_lock(&state->rx_lock);
    unsigned status = base->RCSR;
    printlk(LK_VERBOSE, "ISR RX status: %x\n", status);
    /* Do we have an RX interrupt enabled pending */
    if (!(I2S_RX_ENABLE_MASK(status) & I2S_RX_STATUS_MASK(status))) {
        reschedule = false;
        goto exit_rx_isr;
    }

    state->irq_stats_rx++;

    if (is_imx_sai_isr_xx_pending(status, I2S_RX_WS_PENDING)) {
        printlk(LK_NOTICE, "%s:%d: SAI%d: Word Start Interrupt pending\n",
                __PRETTY_FUNCTION__, __LINE__, state->bus_id);
        pending |= I2S_RCSR_WSF_MASK;
    }

    if (is_imx_sai_isr_xx_pending(status, I2S_RX_SE_PENDING)) {
        printlk(LK_ERR, "%s:%d: SAI%d: Sync Error Interrupt pending\n",
                __PRETTY_FUNCTION__, __LINE__, state->bus_id);
        pending |= I2S_RCSR_SEF_MASK;
    }

    if (is_imx_sai_isr_xx_pending(status, I2S_RX_FE_PENDING)) {
        u64 timestamp = current_time_hires();
        printlk(LK_ERR, "%s:%d: [%05d.%06d] SAI%d: FIFO Error Overflow (%d/%d/%d/%d)\n",
                __PRETTY_FUNCTION__,
                __LINE__,
                (int) (timestamp / 1000000ULL),
                (int) (timestamp % 1000000ULL),
                state->bus_id,
                arch_curr_cpu_num(), state->irq_stats,
                state->irq_stats_rx, state->irq_stats_tx);
        pending |= I2S_RCSR_FEF_MASK;
    }

    /* Clear the interrupt pending */
    SAI_RxClearStatusFlags(base, pending);
    if (pending & I2S_RCSR_FEF_MASK) {
        imx_sai_rx_stop_hw(state);

        handle->xfer_remaining = ERR_IO;
        printlk(LK_NOTICE, "%s:%d: SAI%d: Aborting transfer(%d) (%ld)\n",
            __PRETTY_FUNCTION__,
            __LINE__,
            state->bus_id,
            state->irq_stats_rx, handle->xfer_remaining);
        smp_wmb();
        reschedule = true;
        event_signal(&state->rx_wait, false);
        goto exit_rx_isr;
    }

    /* FIXME: Notify client through callback in case of error ? */

    if (handle->dma_enable) {
        printlk(LK_ERR, "%s:%d: SAI%d: Got unexpected error\n",
                __PRETTY_FUNCTION__, __LINE__, state->bus_id);
        goto exit_rx_isr;
    }

    /* Handle the DATA */
    /* We assume the period size to be modulo the watermak */
#if defined(FSL_FEATURE_SAI_FIFO_COUNT) && (FSL_FEATURE_SAI_FIFO_COUNT > 1)
    if (is_imx_sai_isr_xx_pending(status, I2S_RX_FR_PENDING)) {

#if defined(IMX_SAI_EMPTY_RXFIFO) && (IMX_SAI_EMPTY_RXFIFO == 1)
        xfer_len = handle->bitWidth / 8U;
        xfer_len *= (handle->channel + 1);
        xfer_len *= SAI_GetRxFifoLevel(base);
        printlk(LK_VERBOSE, "Attempting to read %lx chars\n", xfer_len);
#else
        xfer_len = handle->burst_length;
#endif
    }
#else
    if (is_imx_sai_isr_xx_pending(status, I2S_RX_FW_PENDING)) {
        xfer_len = handle->bitWidth / 8U;
    }
#endif
    ASSERT(xfer_len <= handle->buf_size);

    bool period_elapsed = imx_sai_period_add_xfer(&state->rx_period, xfer_len);
    if (period_elapsed)
        imx_sai_notify_period(state, true);

    if (xfer_len) {
        SAI_ReadNonBlocking(base, handle->channel, handle->bitWidth, handle->buf, xfer_len);

        if (imx_is_hw_state(hw_state, kSAI_hw_Flushing)) {
            printlk(LK_NOTICE, "Flushing request detected \n");

            if (handle->xfer_remaining) {
                handle->xfer_remaining = ERR_CANCELLED;
                reschedule = true;
                event_signal(&state->rx_wait, false);
            }
        } else {
            reschedule = imx_sai_push_to_rx_circ_buffer(state,
                handle->buf, xfer_len);
        }
    }

exit_rx_isr:
    spin_unlock(&state->rx_lock);
    smp_wmb();

    return reschedule;
}

static enum handler_return imx_sai_isr(void *arg)
{
    struct device *dev = (struct device *)arg;
    ASSERT(dev);

    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    bool reschedule = false;

    state->irq_stats++;

    if (imx_sai_isr_rx(state))
        reschedule = true;

    if (imx_sai_isr_tx(state))
        reschedule = true;

    if (reschedule)
        return INT_RESCHEDULE;
    else
        return INT_NO_RESCHEDULE;
}

static unsigned imx_sai_set_mclk(struct imx_sai_state *state, unsigned rate,
                                 bool is_tx, unsigned *pll_k_value)
{
    struct device_cfg_clk **clk_configs;
    const char **mclk_configs_names;
    struct device_cfg_clk * clk;
    unsigned mclk_cfg;
    unsigned *mclk_rate_ptr;
    bool is_dummy = is_tx ? state->is_dummy_tx : state->is_dummy_rx;

    if ( is_tx ) {
        clk_configs = state->clk_tx;
        mclk_configs_names = state->mclk_tx_config;
        mclk_rate_ptr = &state->mclk_tx_rate;
    } else {
        clk_configs = state->clk_rx;
        mclk_configs_names = state->mclk_rx_config;
        mclk_rate_ptr = &state->mclk_rx_rate;
    }

    if ((rate == 32000) || (rate == 16000) || (rate == 8000) || (rate == 64000)) {
        /* Fs: 8kHz, 16kHz, 32 kHz, 64kHz */
        mclk_cfg = SAI_MCLK_CONFIG_32K;
        clk = clk_configs[SAI_MCLK_CONFIG_32K];
        if (clk == NULL)
            clk = clk_configs[SAI_MCLK_CONFIG_48K];
    } else if (((rate % 12000) == 0)) {
        /* Fs: 12kHz, 24kHz, 48 kHz, 96kHz, 192kHz, 384kHz  */
        mclk_cfg = SAI_MCLK_CONFIG_48K;
        clk = clk_configs[SAI_MCLK_CONFIG_48K];
    } else if ((rate % 11025) == 0) {
        /* Fs: 11.025kHz, 22.05kHz, 44.1 kHz, 88.2kHz, 176.4kHz, 352.8kHz  */
        mclk_cfg = SAI_MCLK_CONFIG_44K;
        clk = clk_configs[SAI_MCLK_CONFIG_44K];
    } else {
        printlk(LK_INFO, "%s:%d: format->sampleRate_Hz: %d is invalid\n",
                __PRETTY_FUNCTION__, __LINE__, rate);
        ASSERT(0);
    }

    if (state->clk_mode == 1 && *mclk_rate_ptr) {
        *mclk_rate_ptr = clk->rate;
        return 0;
    }

    printlk(LK_INFO, "%s:%d: Switch to %d compatible mclk\n",
            __PRETTY_FUNCTION__, __LINE__, rate);
    int ret = 1;

    if (pll_k_value) {
        printlk(LK_INFO, "%s:%d: Switching PLL k factor to %u\n",
                __PRETTY_FUNCTION__, __LINE__, *pll_k_value);
    }

    if ((is_dummy == false) && (state->clock_dev && mclk_configs_names[mclk_cfg]))
        ret = clock_audio_set_sai_mclk_config(state->clock_dev, state->bus_id,
                                              mclk_configs_names[mclk_cfg],
                                              mclk_rate_ptr, pll_k_value);

    /* Fallback solution */
    if (clk && ret) {
        printlk(LK_NOTICE, "%s:%d: SAI%d: Using direct access to root clock mux as fallback\n",
                __PRETTY_FUNCTION__, __LINE__, state->bus_id);
        devcfg_set_clock(clk);
        *mclk_rate_ptr = clk->rate;
    }

    float mclk = *mclk_rate_ptr;
    mclk /= 1000;
    mclk = roundf(mclk);
    mclk *= 1000;
    *mclk_rate_ptr = (unsigned) mclk;

    printlk(LK_INFO, "SAI%d: Mode %d: Switching to %s mclk rate %d - harmonic of %d\n",
           state->bus_id, state->mclk_as_output, is_tx ? "tx" : "rx",
           *mclk_rate_ptr, rate);

    return *mclk_rate_ptr;
}

static int imx_sai_rx_dma_worker_thread(void *arg)
{
    struct imx_sai_state *state = arg;
    struct imx_hw_state_s *hw_state = &state->rx_hw_state;
    sai_handle_t *handle = &state->sai_rx_handle;
    ASSERT(handle);

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    I2S_Type *base = state->io_base;
    ASSERT(base);

    while (true) {
        bool resched = false;
        /* A reschedule will be trigger if semaphore is not ready */
        sem_wait(&state->dma_rx_driver_data.dma_period_sem);
        struct dma_descriptor *desc = state->dma_rx_driver_data.dma_desc;
        printlk(LK_VERBOSE, "Period elapsed (DMA/Worker): [%llu:%llu]\n",
                      desc->period_elapsed,
                      state->dma_rx_driver_data.dma_period_elapsed);
        uint64_t period_elapsed =
            desc->period_elapsed - state->dma_rx_driver_data.dma_period_elapsed;

        if (period_elapsed > state->dma_rx_driver_data.dma_nr_periods) {
            printlk(LK_NOTICE,
                "ERROR period_elapsed > dma_nr_periods (%lld/%d/%lld)\n",
                period_elapsed,
                state->dma_rx_driver_data.dma_nr_periods,
                state->dma_rx_driver_data.dma_period_elapsed);

#ifdef ENABLE_PERMISSIVE_MODE
            printlk(LK_WARNING, "SAI%d: WARNING PERMISSIVE: period elapsed not expected (%lld vs. %d)\n",
                    __PRETTY_FUNCTION__, __LINE__, state->bus_id, period_elapsed, state->dma_rx_driver_data.dma_nr_periods);

            state->dma_rx_driver_data.dma_period_elapsed +=
                ((period_elapsed/state->dma_rx_driver_data.dma_nr_periods - 1) *
                 state->dma_rx_driver_data.dma_nr_periods + 1 +
                 period_elapsed % state->dma_rx_driver_data.dma_nr_periods);

            period_elapsed = state->dma_rx_driver_data.dma_nr_periods - 1;
#endif
        }

#ifndef ENABLE_PERMISSIVE_MODE
        /* FIXME: we see sometimes one startup with period_elapsed = 5 and  dma_nr_periods = 4 */
        ASSERT(period_elapsed <= state->dma_rx_driver_data.dma_nr_periods);
#endif

        spin_lock_saved_state_t lock_state;
        spin_lock_irqsave(&state->rx_lock, lock_state);
        unsigned i, index;
        size_t len = state->dma_rx_driver_data.dma_period_size;

        for (i = 0; i < period_elapsed; i++) {
            index =
                state->dma_rx_driver_data.dma_period_elapsed %
                state->dma_rx_driver_data.dma_nr_periods;
            void *buf = state->dma_rx_driver_data.dma_buffer;
            buf += len * index;
            if (imx_is_hw_state(hw_state, kSAI_hw_Flushing)) {
                printlk(LK_INFO, "Flushing request detected (DMA mode)\n");
                if (handle->xfer_remaining  > 0 ) {
                    handle->xfer_remaining = ERR_CANCELLED;
                    event_signal(&state->rx_wait, false);
                    resched = true;
                }
            } else {
#if defined(DMA_BUFFER_MEMALIGN_TYPE) && (DMA_BUFFER_MEMALIGN_TYPE == 1)
                arch_invalidate_cache_range((vaddr_t) buf, len);
#endif /* DMA_BUFFER_MEMALIGN_TYPE */
                resched = imx_sai_push_to_rx_circ_buffer(state, buf, len);
            }
            state->dma_rx_driver_data.dma_period_elapsed++;
            smp_wmb();
        }

        if (likely(state->rx_period.duration)) {
            uint32_t new, previous = state->rx_period.erate;
            sai_transfer_format_t *format = &state->sai_rx_fmt;
            u64 delta = state->rx_period.duration;

            new = len * 1e6 / delta /
                (format->audio_channel * format->bitWidth / 8);

            /* Check freq is still in range +/- 5% */
            if ((new < (previous * 95 / 100) || new > (previous * 105 / 100)) &&
                 state->rx_cb) {
                resched += state->rx_cb(SAI_EVENT_ERATE_CHANGE, &new, state->rx_cb_cookie);
                printlk(LK_DEBUG, "new rate %u - channels %d bitwidth %d\n", new,
                       format->audio_channel, format->bitWidth);
            }

            state->rx_period.erate = new;
        }

        spin_unlock_irqrestore(&state->rx_lock, lock_state);

        smp_wmb();
        if (resched)
            thread_preempt();
    }
    return 0;
}

static bool imx_sai_rx_dma_cb(struct dma_descriptor *desc)
{
    struct imx_sai_state *state = desc->cookie;

    printlk(LK_VERBOSE,
                  "Descriptor RX completed with status %d, period elapsed %llu, bytes transferred %llu \n",
                  desc->status, desc->period_elapsed, desc->bytes_transferred);

    struct imx_period_s *period = &state->rx_period;
    u64 now = current_time_hires();
    if (period->ts)
        period->duration = now - period->ts;
    period->ts = now;

    sem_post(&state->dma_rx_driver_data.dma_period_sem, false);

    return true;
}

static bool imx_sai_rx_dma_zerocopy_cb(struct dma_descriptor *desc,
                                       uint32_t *addr, uint32_t *len)
{
    struct imx_sai_state *state = desc->cookie;

    uint32_t offset = ((desc->period_elapsed - 1
            + state->dma_rx_driver_data.dma_nr_periods)
            * state->dma_rx_driver_data.dma_period_size)
            % state->rx_circ_buf_size;

    /* TODO: Improve with a variable circular buffer size */
    ASSERT(!(state->rx_circ_buf_size % state->dma_rx_driver_data.dma_period_size));

    if (kSAI_DMA_Zerocopy_Cached == desc->dma_mode) {
        /* Invalidate cache fed by DMA */
        void *buffer =
            *addr - state->dma_rx_driver_data.dma_buffer_phys
                + state->dma_rx_driver_data.dma_buffer;
        arch_invalidate_cache_range((vaddr_t) buffer, (size_t)*len);
    }

    /* TODO: this could be improved by a relative offset computation instead of
     * going through vaddr_to_paddr()
     */
    *addr = (uint32_t) state->dma_rx_driver_data.dma_buffer_phys + offset;

    return true;
}

static int imx_sai_tx_dma_worker_thread(void *arg)
{
    struct imx_sai_state *state = arg;
    struct imx_hw_state_s *hw_state = &state->tx_hw_state;

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    I2S_Type *base = state->io_base;
    ASSERT(base);

    while (true) {
        int resched = 0;
        /* A reschedule will be trigger if semaphore is not ready */
        sem_wait(&state->dma_tx_driver_data.dma_period_sem);
        smp_mb();
        struct dma_descriptor *desc = state->dma_tx_driver_data.dma_desc;
        printlk(LK_VERBOSE, "Period elapsed (DMA/Worker): [%llu:%llu]\n",
                      desc->period_elapsed,
                      state->dma_tx_driver_data.dma_period_elapsed);

        spin_lock_saved_state_t lock_state;
        spin_lock_irqsave(&state->tx_lock, lock_state);
        unsigned index;
        void *buf = state->dma_tx_driver_data.dma_buffer;
        size_t len = state->dma_tx_driver_data.dma_period_size;
        bool period_elapsed;

        if (!imx_is_hw_state(hw_state, kSAI_hw_Opened)) {
            printlk(
                    LK_INFO,
                    "SAI%d: Warning - worker signaled while TX closed. Discarding.\n",
                    state->bus_id);
            spin_unlock_irqrestore(&state->tx_lock, lock_state);
            continue;
        }

        index =
            state->dma_tx_driver_data.dma_period_elapsed %
            state->dma_tx_driver_data.dma_nr_periods;
        buf += len * index;
        period_elapsed = imx_sai_period_add_xfer(&state->tx_period, len);

        resched |= imx_sai_pop_from_tx_circ_buffer(state, buf, len);
#if defined(DMA_BUFFER_MEMALIGN_TYPE) && (DMA_BUFFER_MEMALIGN_TYPE == 1)
        arch_clean_cache_range((vaddr_t) buf, len);
#endif /* DMA_BUFFER_MEMALIGN_TYPE */
        state->dma_tx_driver_data.dma_period_elapsed++;

        smp_mb();
        spin_unlock_irqrestore(&state->tx_lock, lock_state);

        if (period_elapsed)
            resched = imx_sai_notify_period(state, false);

        if (resched)
            thread_preempt();
    }
    return 0;
}

static bool imx_sai_tx_dma_cb(struct dma_descriptor *desc)
{
    struct imx_sai_state *state = desc->cookie;
    struct imx_hw_state_s *hw_state = &state->tx_hw_state;

    if ((!imx_is_hw_state(hw_state, kSAI_hw_Started))
        && (!spin_lock_held(&state->tx_lock))) {
        printlk(LK_ERR, "%s:%d: SAI%d: ERROR: TX DMA works while TX is not started....\n",
            __PRETTY_FUNCTION__, __LINE__, state->bus_id);
    }

    printlk(LK_VERBOSE,
                  "Desc TX done with status %d, period elapsed %llu, bytes transferred %llu, cbuf_content=%lu \n",
                  desc->status, desc->period_elapsed, desc->bytes_transferred, cbuf_space_used(&state->tx_circ_buf));

    sem_post(&state->dma_tx_driver_data.dma_period_sem, false);

    return true;
}

static bool imx_sai_tx_dma_zerocopy_cb(struct dma_descriptor *desc,
                                       uint32_t *addr, uint32_t *len)
{
    struct imx_sai_state *state = desc->cookie;

    uint32_t offset = ((desc->period_elapsed - 1
            + state->dma_tx_driver_data.dma_nr_periods)
            * state->dma_tx_driver_data.dma_period_size)
            % state->tx_circ_buf_size;

    *addr = (uint32_t) state->dma_tx_driver_data.dma_buffer_phys + offset;

    return true;
}

static void imx_sai_get_clk_config(struct device *dev, bool is_tx)
{
    const struct device_config_data *config = dev->config;
    struct imx_sai_state *state = dev->state;
    struct device_cfg_clk **cfgs;
    const char **cfg_names;
    const char *cfg_name_defaults[] = {
        "mclk-44k",
        "mclk-48k",
        "mclk-32k"
    };

    if (is_tx) {
        cfgs = state->clk_tx;
        cfg_names = state->mclk_tx_config;
    } else {
        cfgs = state->clk_rx;
        cfg_names = state->mclk_rx_config;
    }

    if (!*cfg_names)
        cfg_names = cfg_name_defaults;

    for(int i = 0; i < SAI_MCLK_CONFIG; i++) {
        cfgs[i] = device_config_get_clk_by_name(
                config, cfg_names[i]);
        if (NULL == cfgs[i]) {
            /* Fallback to default config name */
            printlk(LK_INFO, "%s:%d: %s configuration does not exist. Falling back to default config name %s...\n",
                    __PRETTY_FUNCTION__, __LINE__, cfg_names[i],
                    cfg_name_defaults[i]);
            cfgs[i] = device_config_get_clk_by_name(config, cfg_name_defaults[i]);
            if (NULL == cfgs[i]) {
                printlk(LK_INFO, "%s:%d: %s configuration does not exist either...\n",
                        __PRETTY_FUNCTION__, __LINE__, cfg_name_defaults[i]);
                ASSERT(0);
            }
        }
    }
}

static uint32_t imx_sai_get_max_lanes(struct device *dev, bool is_rx, bool recurse)
{
    struct device *next_dev;
    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    I2S_Type *base = state->io_base;
    ASSERT(base);

    uint32_t nlanes;
    uint32_t val, mask;
    uint32_t restricted_max_lanes;

    if (is_rx) {
        restricted_max_lanes = state->rx_lanes;
        next_dev = state->rx_sai_chained_dev;
        val = base->RCR3;
        SAI_RxSetChannelFIFOMask(base, 0xff);
        mask = base->RCR3;
        mask &= I2S_RCR3_RCE_MASK;
        mask >>= I2S_RCR3_RCE_SHIFT;
        base->RCR3 = val;
    } else {
        restricted_max_lanes = state->tx_lanes;
        next_dev = state->tx_sai_chained_dev;
        val = base->TCR3;
        SAI_TxSetChannelFIFOMask(base, 0xff);
        mask = base->TCR3;
        mask &= I2S_TCR3_TCE_MASK;
        mask >>= I2S_TCR3_TCE_SHIFT;
        base->TCR3 = val;
    }

    /* Count number of 1's */
    nlanes = (uint32_t)__builtin_popcount(mask);

    /* Clamp number of detected lanes to restricted_max_lanes,
     * when non-zero (i.e. specified via dts) */
    if (restricted_max_lanes && restricted_max_lanes < nlanes) {
        nlanes = restricted_max_lanes;
    }

    /* If current SAI is chained with another one, its lanes must be
     * taken into account */
    if (recurse && next_dev) {
        nlanes += imx_sai_get_max_lanes(next_dev, is_rx, recurse);
    }

    printlk(LK_VERBOSE, "SAI%d %s supports up to %d lanes (chained=%s)\n",
            state->bus_id, (is_rx) ? "RX" : "TX", nlanes, (recurse && next_dev)?"yes":"no");

    return nlanes;
}

static struct list_node imx_sai_list = LIST_INITIAL_VALUE(imx_sai_list);
static spin_lock_t imx_sai_list_lock = SPIN_LOCK_INITIAL_VALUE;

#define IMX_SAI_DEFAULT_MCLK_RATE 48000

static status_t imx_sai_init(struct device *dev)
{
    const struct device_config_data *config = dev->config;
    struct imx_sai_state *state;
    status_t ret = NO_ERROR;
    bool dma_tx_enable;
    bool dma_rx_enable;
    uint32_t dma_rx_mode;
    uint32_t dma_tx_mode;
    uint32_t sync_rx_mode;
    uint32_t sync_tx_mode;
    uint32_t rx_fcomb;
    uint32_t tx_fcomb;

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);
    state = malloc(sizeof(struct imx_sai_state));
    ASSERT(state);
    memset(state, 0, sizeof(*state));
    dev->state = state;
    state->device = dev;
    state->bus_id = config->bus_id;
    state->is_dummy_rx = state->is_dummy_tx = false;

    /* Clock device */
    state->clock_dev = of_device_lookup_device(dev, "clock_audio");

    /* Look for chained SAI devices */
    state->rx_sai_chained_dev = of_device_lookup_device(dev, "rx,sai_chained");
    if (state->rx_sai_chained_dev) {
        printlk(LK_NOTICE, "%s:%d: SAI%d RX setup as chained to another SAI...\n",  __PRETTY_FUNCTION__, __LINE__, state->bus_id);
    }
    state->tx_sai_chained_dev = of_device_lookup_device(dev, "tx,sai_chained");
    if (state->tx_sai_chained_dev) {
        printlk(LK_NOTICE, "%s:%d: SAI%d TX setup as chained to another SAI...\n",  __PRETTY_FUNCTION__, __LINE__, state->bus_id);
    }

    /* Get tx mclk configs */
    ret = of_device_get_strings(dev, "mclk-tx-config-names", state->mclk_tx_config, SAI_MCLK_CONFIG);
    printlk(LK_NOTICE, "%s:%d: mclk_tx_config ret code: %d\n",
            __PRETTY_FUNCTION__, __LINE__, ret);
    if (ret) {
        int i;
        for (i = 0; i < ret; i++)
            printlk(LK_NOTICE, "%s:%d: mclk tx config %d name: %s\n",
                    __PRETTY_FUNCTION__, __LINE__, i, state->mclk_tx_config[i]);
    }
    imx_sai_get_clk_config(dev, true);
    ASSERT(NULL != state->clk_tx[SAI_MCLK_CONFIG_48K]);
    ASSERT(NULL != state->clk_tx[SAI_MCLK_CONFIG_44K]);
    ASSERT(NULL != state->clk_tx[SAI_MCLK_CONFIG_32K]);

    /* Get rx mclk configs */
    ret = of_device_get_strings(dev, "mclk-rx-config-names", state->mclk_rx_config, SAI_MCLK_CONFIG);
    printlk(LK_NOTICE, "%s:%d: mclk_rx_config ret code: %d\n",
            __PRETTY_FUNCTION__, __LINE__, ret);
    if (ret) {
        int i;
        for (i = 0; i < ret; i++)
            printlk(LK_NOTICE, "%s:%d: mclk rx config %d name: %s\n",
                    __PRETTY_FUNCTION__, __LINE__, i, state->mclk_rx_config[i]);
    }
    imx_sai_get_clk_config(dev, false);
    ASSERT(NULL != state->clk_rx[SAI_MCLK_CONFIG_48K]);
    ASSERT(NULL != state->clk_rx[SAI_MCLK_CONFIG_44K]);
    ASSERT(NULL != state->clk_rx[SAI_MCLK_CONFIG_32K]);

    struct device_cfg_reg *reg =
                        device_config_get_register_by_name(config, "core");
    ASSERT(reg);

    struct device_cfg_irq *irq =
                        device_config_get_irq_by_name(config, "core");
    ASSERT(irq);

    spin_lock_init(&state->lock);
    spin_lock_init(&state->rx_lock);
    spin_lock_init(&state->tx_lock);
    spin_lock_init(&state->tx_cb_lock);
    spin_lock_init(&state->rx_cb_lock);
    spin_lock_init(&state->rx_bitcnt_lock);
    spin_lock_init(&state->tx_bitcnt_lock);
    imx_init_hw_state(&state->rx_hw_state);
    imx_init_hw_state(&state->tx_hw_state);

    imx_set_hw_state(&state->rx_hw_state, kSAI_hw_Up);
    imx_set_hw_state(&state->tx_hw_state, kSAI_hw_Up);

    mutex_init(&state->rx_mutex);
    mutex_init(&state->tx_mutex);
    /* Wake up any waiting threads when signaled */
    event_init(&state->rx_wait, false, 0);
    /* Wake up one thread and return */
    event_init(&state->tx_wait, false, EVENT_FLAG_AUTOUNSIGNAL);

    state->sai_rx_handle.dma_enable = false;
    state->sai_tx_handle.dma_enable = false;

    devcfg_set_pins_by_name(config->pins_cfg,
                            config->pins_cfg_cnt, "default");

    state->io_base = (I2S_Type *) reg->vbase;

    state->is_dummy_rx = of_device_get_bool(dev, "is-dummy")
                      || of_device_get_bool(dev, "is-dummy-rx");
    state->is_dummy_tx = of_device_get_bool(dev, "is-dummy")
                      || of_device_get_bool(dev, "is-dummy-tx");

    /* Check clock mode */
    state->mclk_as_output = !!of_device_get_bool(dev, "mclk,is-output");
    printlk(LK_NOTICE, "%s:%d: SAI%d mclk is output : %s\n",
           __PRETTY_FUNCTION__, __LINE__, state->bus_id, (state->mclk_as_output == true) ? "true" : "false");

    /* mclk select */
    if (of_device_get_int32(dev, "rx,mclk-select", &state->mclk_select_rx))
        state->mclk_select_rx = kSAI_MclkSourceSelect1;

    if (of_device_get_int32(dev, "tx,mclk-select", &state->mclk_select_tx))
        state->mclk_select_tx = kSAI_MclkSourceSelect1;

    /* Setup mclk TX */
    imx_sai_set_mclk(state, IMX_SAI_DEFAULT_MCLK_RATE, true, NULL);
    /* Setup mclk RX */
    imx_sai_set_mclk(state, IMX_SAI_DEFAULT_MCLK_RATE, false, NULL);

    /* Master/Slave mode (defaults to Master if not specified) */
    state->rx_is_slave = !!of_device_get_bool(dev, "rx,slave_mode");
    printlk(LK_NOTICE, "%s:%d: SAI%d RX configured for %s mode...\n",
            __PRETTY_FUNCTION__, __LINE__, state->bus_id,
           state->rx_is_slave ? "slave" : "master");
    state->tx_is_slave = !!of_device_get_bool(dev, "tx,slave_mode");
    printlk(LK_NOTICE, "%s:%d: SAI%d TX configured for %s mode...\n",
            __PRETTY_FUNCTION__, __LINE__, state->bus_id,
           state->tx_is_slave ? "slave" : "master");

    /* GPIO configuration, if present (used for external mclk routing) */
    if (!gpio_request_by_name(dev, "gpio,mclk-is-extern",
                              &(state->gpio_mclk_ctrl), 0)) {
        printlk(LK_INFO, "%s:%d: Apply GPIO configuration for external MCLK control...\n",
                __PRETTY_FUNCTION__, __LINE__);
        gpio_desc_set_direction(&state->gpio_mclk_ctrl);
    }

    /* Number of lanes, when specified */
    if (of_device_get_int32(dev, "rx,lanes", &state->rx_lanes))
        state->rx_lanes = 0;
    if (of_device_get_int32(dev, "tx,lanes", &state->tx_lanes))
        state->tx_lanes = 0;
    if (state->rx_lanes)
        printlk(LK_NOTICE, "SAI%d: RX lanes usage restriction: %u\n",
                state->bus_id, state->rx_lanes);
    if (state->tx_lanes)
        printlk(LK_NOTICE, "SAI%d: TX lanes usage restriction: %u\n",
                state->bus_id, state->tx_lanes);


    uint32_t sz;
    if (of_device_get_int32(dev, "tx-buf-length", &sz))
        sz = SAI_MAX_TX_CIRC_BUF_SIZE;
    state->tx_circ_buf_max_size = sz;
    printlk(LK_NOTICE, "SAI%d: TX max circ buf size: %zu\n",
                                    state->bus_id, state->tx_circ_buf_max_size);

    if (of_device_get_int32(dev, "rx-buf-length", &sz))
        sz = SAI_MAX_RX_CIRC_BUF_SIZE;
    state->rx_circ_buf_size = sz;
    printlk(LK_NOTICE, "SAI%d: RX max circ buf size: %zu\n",
                                    state->bus_id, state->rx_circ_buf_size );

    if (of_device_get_int32(dev, "tx,flush,silence-periods", &state->flush_silence_periods))
        state->flush_silence_periods = IMX_SAI_DMA_SILENCE_N_PERIOD_ON_FLUSH;

    SAI_RxReset(state->io_base);
    SAI_TxReset(state->io_base);

    dma_rx_enable = !!!of_device_get_bool(dev, "disable-dma-rx") && (false == state->is_dummy_rx);
    dma_tx_enable = !!!of_device_get_bool(dev, "disable-dma-tx") && (false == state->is_dummy_tx);

    if (!!of_device_get_bool(dev, "disable-dma")) {
        dma_rx_enable = false;
        dma_tx_enable = false;
    }

    /* Synchronization configuration (default: asynchronous) */
    if (of_device_get_int32(dev, "rx,sync-mode", &sync_rx_mode))
        sync_rx_mode = kSAI_ModeAsync;
    if (of_device_get_int32(dev, "tx,sync-mode", &sync_tx_mode))
        sync_tx_mode = kSAI_ModeAsync;
    state->rx_sync_mode = (sai_sync_mode_t)sync_rx_mode;
    state->tx_sync_mode = (sai_sync_mode_t)sync_tx_mode;
    printlk(LK_NOTICE, "%s:%d: SAI%d rx synchronization mode set to %u\n",
            __PRETTY_FUNCTION__, __LINE__, state->bus_id, sync_rx_mode);
    printlk(LK_NOTICE, "%s:%d: SAI%d tx synchronization mode set to %u\n",
            __PRETTY_FUNCTION__, __LINE__, state->bus_id, sync_tx_mode);

    /* FIFO Combine feature usage */
    if (of_device_get_int32(dev, "rx,fcomb", &rx_fcomb))
        rx_fcomb = 0;
    if (of_device_get_int32(dev, "tx,fcomb", &tx_fcomb))
        tx_fcomb = 0;
    printlk(LK_NOTICE, "%s:%d: SAI%d RX FCOMB configuration set to %u.\n",
            __PRETTY_FUNCTION__, __LINE__, state->bus_id, rx_fcomb);
    printlk(LK_NOTICE, "%s:%d: SAI%d TX FCOMB configuration set to %u.\n",
            __PRETTY_FUNCTION__, __LINE__, state->bus_id, tx_fcomb);
    state->rx_fcomb = rx_fcomb;
    state->tx_fcomb = tx_fcomb;

    /* DMA mode configuration (default: DMA to/from dedicated buffer) */
    if (of_device_get_int32(dev, "rx,dma-mode", &dma_rx_mode))
        dma_rx_mode = kSAI_DMA_Private_Buffer;
    if (of_device_get_int32(dev, "tx,dma-mode", &dma_tx_mode))
        dma_tx_mode = kSAI_DMA_Private_Buffer;

    printlk(LK_NOTICE, "%s:%d: SAI%d driver running with DMA RX mode %sbled (mode %d)\n",
                                        __PRETTY_FUNCTION__,
                                        __LINE__,
                                        state->bus_id,
                                        dma_rx_enable ? "ena": "disa",
                                        dma_rx_mode);

    printlk(LK_NOTICE, "%s:%d: SAI%d driver running with DMA TX mode %sbled (mode %d)\n",
                                        __PRETTY_FUNCTION__,
                                        __LINE__,
                                        state->bus_id,
                                        dma_tx_enable ? "ena": "disa",
                                        dma_tx_mode);

    if (dma_rx_enable) {
        unsigned dma_period_size_max = SAI_DMA_RX_MAX_PERIOD;
        unsigned dma_nr_periods;

        sem_init(&state->dma_rx_driver_data.dma_period_sem, 0);
        state->sai_rx_handle.dma_enable = true;
        state->sai_rx_handle.dma_mode = dma_rx_mode;
        state->dma_rx_driver_data.dma_period_size_max = dma_period_size_max;

        if (of_device_get_int32(dev, "dma-rx-nr-period", &dma_nr_periods))
            dma_nr_periods = IMX_SAI_DMA_NR_PERIOD;
        state->dma_rx_driver_data.dma_nr_periods = dma_nr_periods;

        printlk(LK_INFO, "%s:%d: RX: Using %d period buffers of length %d bytes\n",
                __PRETTY_FUNCTION__, __LINE__,
                state->dma_rx_driver_data.dma_nr_periods,
                state->dma_rx_driver_data.dma_period_size_max);

        if (kSAI_DMA_Private_Buffer != dma_rx_mode) {
            /* Nothing to do - zero-copy into circular buffer */
            printlk(LK_INFO,"%s:%d: Use zero-copy DMA buffer descriptors to write into rx circular buffer\n",
                    __PRETTY_FUNCTION__, __LINE__);
        } else {
            /* allocate non-cachable buffers for DMA transfers */
#if defined(DMA_BUFFER_MEMALIGN_TYPE) && (DMA_BUFFER_MEMALIGN_TYPE == 1)
            state->dma_rx_driver_data.dma_buffer = memalign(DMA_BUFFER_ALIGNMENT,
                dma_nr_periods * dma_period_size_max);
#else
            const char* buffer_name = "sai_rx";
            void **dma_buffer = &state->dma_rx_driver_data.dma_buffer;

            status_t ret_alloc;
            ret_alloc = vmm_alloc_contiguous(vmm_get_kernel_aspace(),
                                                buffer_name,
                                                dma_nr_periods * dma_period_size_max,
                                                dma_buffer,
                                                8 /* log2_align */,
                                                0 /* vmm flags */,
                                                ARCH_MMU_FLAG_UNCACHED);
            printlk(LK_VERBOSE, "%s: vmm_alloc_contig returns %d, ptr %p, 0x%lx\n",
                buffer_name,
                ret_alloc, *dma_buffer,
                vaddr_to_paddr((void *)*dma_buffer));
            if (ret_alloc < 0) {
                printlk(LK_ERR, "%s:%d: failed to allocate memory for %s.\n",
                        __PRETTY_FUNCTION__, __LINE__, buffer_name);
                ret = ERR_NO_MEMORY;
                return ret;
            }
#endif /* DMA_BUFFER_MEMALIGN_TYPE */
            ASSERT(state->dma_rx_driver_data.dma_buffer);
        }

        /* create SDMA workers */
        char thread_name[64];
        snprintf(thread_name, 64, "sai%d_rx dma worker",
             state->bus_id);

        state->dma_rx_driver_data.dma_thread = thread_create(
            thread_name,
            imx_sai_rx_dma_worker_thread, state,
            HIGH_PRIORITY, DEFAULT_STACK_SIZE);
        ASSERT(state->dma_rx_driver_data.dma_thread);
        thread_set_pinned_cpu(state->dma_rx_driver_data.dma_thread, -1);
        thread_detach_and_resume(state->dma_rx_driver_data.dma_thread);
    }

    if (dma_tx_enable) {
        unsigned dma_period_size_max = SAI_DMA_TX_MAX_PERIOD;
        unsigned dma_nr_periods;
        unsigned ret;

        sem_init(&state->dma_tx_driver_data.dma_period_sem, 0);
        state->sai_tx_handle.dma_enable = true;
        state->sai_tx_handle.dma_mode = dma_tx_mode;
        state->dma_tx_driver_data.dma_period_size_max = dma_period_size_max;

        if (of_device_get_int32(dev, "dma-tx-nr-period", &dma_nr_periods))
            dma_nr_periods = IMX_SAI_DMA_NR_PERIOD;
        state->dma_tx_driver_data.dma_nr_periods = dma_nr_periods;

        printlk(LK_INFO, "%s:%d: TX: Using %d period buffers of length %d bytes\n",
                __PRETTY_FUNCTION__, __LINE__,
                state->dma_tx_driver_data.dma_nr_periods,
                state->dma_tx_driver_data.dma_period_size_max);

        if (kSAI_DMA_Private_Buffer != dma_tx_mode) {
            /* Nothing to do - zero-copy into circular buffer */
            printlk(LK_INFO, "%s:%d: Use zero-copy DMA buffer descriptors to read from tx circular buffer\n",
                    __PRETTY_FUNCTION__, __LINE__);
        } else {
            /* allocate non-cachable buffers for DMA transfers */
#if defined(DMA_BUFFER_MEMALIGN_TYPE) && (DMA_BUFFER_MEMALIGN_TYPE == 1)
            state->dma_tx_driver_data.dma_buffer = memalign(DMA_BUFFER_ALIGNMENT,
                dma_nr_periods * dma_period_size_max);
#else
            const char* buffer_name = "sai_tx";
            void **dma_buffer = &state->dma_tx_driver_data.dma_buffer;

            status_t ret_alloc;
            ret_alloc = vmm_alloc_contiguous(vmm_get_kernel_aspace(),
                                                buffer_name,
                                                dma_nr_periods * dma_period_size_max,
                                                dma_buffer,
                                                8 /* log2_align */,
                                                0 /* vmm flags */,
                                                ARCH_MMU_FLAG_UNCACHED);
            printlk(LK_VERBOSE, "%s: vmm_alloc_contig returns %d, ptr %p, 0x%lx\n",
                buffer_name,
                ret_alloc, *dma_buffer,
                vaddr_to_paddr((void *)*dma_buffer));
            if (ret_alloc < 0) {
                printlk(LK_ERR, "%s:%d: failed to allocate memory for %s.\n",
                        __PRETTY_FUNCTION__, __LINE__, buffer_name);
                ret = ERR_NO_MEMORY;
                return ret;
            }
#endif /* DMA_BUFFER_MEMALIGN_TYPE */
            ASSERT(state->dma_tx_driver_data.dma_buffer);
        }

        /* create SDMA workers */
        char thread_name[64];
        snprintf(thread_name, 64, "sai%d_tx dma worker",
             state->bus_id);

        state->dma_tx_driver_data.dma_thread = thread_create(
            thread_name,
            imx_sai_tx_dma_worker_thread, state,
            HIGH_PRIORITY + 1, DEFAULT_STACK_SIZE);
        ASSERT(state->dma_tx_driver_data.dma_thread);

        thread_set_pinned_cpu(state->dma_tx_driver_data.dma_thread, 1);
        thread_detach_and_resume(state->dma_tx_driver_data.dma_thread);
    }

    if (!(state->is_dummy_rx && state->is_dummy_tx)) {
        /*
         * Register interrupt handler only if either of RX or TX
         * is not a dummy path.
         */
        register_int_handler(irq->irq, imx_sai_isr, dev);
        unmask_interrupt(irq->irq);
    }

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&imx_sai_list_lock, lock_state);

    list_add_tail(&imx_sai_list, &state->node);

    spin_unlock_irqrestore(&imx_sai_list_lock, lock_state);

    return ret;
}

static void imx_sai_tx_adjust_buffers(struct imx_sai_state *state, size_t length)
{
    sai_handle_t *tx_handle = &state->sai_tx_handle;
    ASSERT(length < state->tx_circ_buf_max_size);

    state->tx_circ_buf_size = length;
    cbuf_adjust_size(&state->tx_circ_buf, length);

    tx_handle->cbuf_watermark = length;
    tx_handle->cbuf_watermark -= TX_CBUF_N_FIFO_WATERMARK * tx_handle->buf_size;
}

static status_t imx_sai_tx_open(struct device *dev)
{
    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    void *buf, *circ_buf;
    size_t buf_size, circ_buf_size;
    const char * circ_buf_name = "sai tx circ buf";
    status_t ret_alloc;

    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    I2S_Type *base = state->io_base;
    ASSERT(base);

    sai_handle_t *tx_handle = &state->sai_tx_handle;
    ASSERT(tx_handle);

    buf_size = SAI_TX_BUFFER_SIZE;
    buf = malloc(buf_size);
    ASSERT(buf);
    tx_handle->buf = buf;
    tx_handle->buf_size = buf_size;

    /* By default, consider current SAI as not part of a multi-SAI configuration */
    state->tx_is_chained = false;

    circ_buf_size = state->tx_circ_buf_max_size;

    switch (tx_handle->dma_mode)
    {
    case kSAI_DMA_Zerocopy_Uncached:
        ret_alloc = vmm_alloc_contiguous(vmm_get_kernel_aspace(),
                                            circ_buf_name,
                                            circ_buf_size,
                                            &circ_buf,
                                            8 /* log2_align */,
                                            0 /* vmm flags */,
                                            ARCH_MMU_FLAG_UNCACHED);
        if (ret_alloc < 0) {
            printlk(LK_ERR, "%s:%d: failed to allocate memory for %s.\n",
                    __PRETTY_FUNCTION__, __LINE__, circ_buf_name);
            return ERR_NO_MEMORY;
        }
        /* Use circular buffer as DMA buffer */
        state->dma_tx_driver_data.dma_buffer = circ_buf;
        break;

    case kSAI_DMA_Zerocopy_Cached:
        circ_buf = memalign(DMA_BUFFER_ALIGNMENT, circ_buf_size);
        /* Use circular buffer as DMA buffer */
        state->dma_tx_driver_data.dma_buffer = circ_buf;
        break;

    default:
        /* Standard DMA mode with intermediate buffer */
        circ_buf = malloc(circ_buf_size);
        break;
    }
    ASSERT(circ_buf);

    printlk(LK_INFO, "%s:%d: SAI%d: TX dma mode %d, buffer = %p\n",
            __PRETTY_FUNCTION__, __LINE__,
            state->bus_id,
            tx_handle->dma_mode,
            state->dma_tx_driver_data.dma_buffer);

    cbuf_initialize_etc(&state->tx_circ_buf, circ_buf_size, circ_buf);

    cbuf_set_flag(&state->tx_circ_buf, CBUF_FLAG_USE_MAX_CHUNK_W);

    if (tx_handle->dma_mode == kSAI_DMA_Zerocopy_Uncached)
        cbuf_clear_flag(&state->tx_circ_buf, CBUF_FLAG_BUF_IS_CACHEABLE);
    else
        cbuf_set_flag(&state->tx_circ_buf, CBUF_FLAG_BUF_IS_CACHEABLE);

    if (tx_handle->dma_mode == kSAI_DMA_Private_Buffer)
        cbuf_set_flag(&state->tx_circ_buf, CBUF_FLAG_SW_IS_READER);
    else
        cbuf_clear_flag(&state->tx_circ_buf, CBUF_FLAG_SW_IS_READER);

    tx_handle->cbuf_watermark = circ_buf_size;
    tx_handle->cbuf_watermark -= TX_CBUF_N_FIFO_WATERMARK * buf_size;

    struct imx_hw_state_s *hw_state = &state->tx_hw_state;

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->tx_lock, lock_state);

    cbuf_reset_with_zero(&state->tx_circ_buf);

    imx_clear_hw_state(hw_state, kSAI_hw_Prepared | kSAI_hw_Started);
    imx_set_hw_state(hw_state, kSAI_hw_Opened);

    SAI_TxReset(base);

    smp_wmb();
    spin_unlock_irqrestore(&state->tx_lock, lock_state);

    /* Call open() for chained SAI(s) if any */
    if (state->tx_sai_chained_dev) {
        /* Set is_chained for current device */
        state->tx_is_chained = true;

        status_t ret = imx_sai_tx_open(state->tx_sai_chained_dev);
        if (ret)
            return ret;

        /* Set is_chained for next device in the chain (must be done AFTER open() call) */
        struct imx_sai_state *next_sai_state = state->tx_sai_chained_dev->state;
        next_sai_state->tx_is_chained = true;
    }

    return 0;
}

static status_t imx_sai_tx_prefill_fifo(struct device *dev)
{
    size_t xfer_len = 0;

    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    sai_handle_t *handle = &state->sai_tx_handle;
    ASSERT(handle);

    I2S_Type *base = state->io_base;
    ASSERT(base);

    printlk(LK_VERBOSE, "FIFO prefill on SAI%d\n", state->bus_id);

    /* We assume the period size to be modulo the watermark */
    unsigned watermark;
    watermark = state->tx_period.fifo_prefill * 2;
#if defined(IMX_SAI_PREFILL_WITH_ZERO) && (IMX_SAI_PREFILL_WITH_ZERO == 0)
    watermark -= handle->watermark;
#endif

#if defined(FSL_FEATURE_SAI_FIFO_COUNT) && (FSL_FEATURE_SAI_FIFO_COUNT > 1)
    xfer_len = handle->bitWidth / 8U;
    xfer_len *= watermark;
    xfer_len *= (handle->channel + 1);
    printlk(LK_VERBOSE, "FIFO watermark: %d (%d)\n", handle->watermark,
        (handle->channel + 1));
#else
    xfer_len = handle->bitWidth / 8U;
    xfer_len *= (handle->channel + 1);
#endif
    ASSERT(xfer_len <= handle->buf_size);

    if (xfer_len) {
        printlk(LK_NOTICE, "SAI%d: FIFO prefill length: %zd %u\n", state->bus_id, xfer_len, watermark);
#if defined(IMX_SAI_PREFILL_WITH_ZERO) && (IMX_SAI_PREFILL_WITH_ZERO == 0)
        size_t read;
        read = cbuf_read(&state->tx_circ_buf, handle->buf, xfer_len, false);
#if defined(IMX_SAI_PERMISSIVE) && (IMX_SAI_PERMISSIVE == 1)
        if (read != xfer_len) {
            printlk(LK_VERBOSE, "Requested from FIFO: %ld read:%ld - injecting silence\n",
                    xfer_len, read);
            memset(handle->buf + read, 0, xfer_len - read);
        }
#else
        ASSERT(read == xfer_len);
#endif
#else
        memset(handle->buf, SAI_FIFO_PREFILL_PATTERN, xfer_len);
#endif
        SAI_WriteNonBlocking(base, handle->channel, handle->bitWidth,
                handle->buf, xfer_len);
    }

    return 0;
}

static status_t imx_sai_tx_start(struct device *dev)
{
    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);
    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    I2S_Type *base = state->io_base;
    ASSERT(base);

    sai_handle_t *tx_handle = &state->sai_tx_handle;
    ASSERT(tx_handle);

    struct imx_hw_state_s *hw_state = &state->tx_hw_state;

    if (imx_is_hw_state(hw_state, kSAI_hw_Started))
        return 0;

    bool flushing_cb = false;
    bool active_cb = false;

    /* Call start() for chained SAI(s) if any
     *
     * IMPORTANT: Master SAI must be the last to start and first to stop
     * when used in a SAI chain.
     */
    if (state->tx_sai_chained_dev) {
        struct imx_sai_state *sai_chained_state =
                state->tx_sai_chained_dev->state;

        if (sai_chained_state->sai_tx_fmt.audio_channel) {
            status_t ret = imx_sai_tx_start(state->tx_sai_chained_dev);
            if (ret)
                return ret;
        }
    }

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->tx_lock, lock_state);

    state->silence_injection_ongoing = false;
    state->silence_injection_count = 0;
    state->silence_injection_len = 0;

    if (state->flush_silence_periods == 0)
        state->flushed_with_silence = true;
    else
        state->flushed_with_silence = false;

    DEBUG_ASSERT(imx_is_hw_state(hw_state, kSAI_hw_Prepared));

    size_t cbuf_level = cbuf_space_used(&state->tx_circ_buf);
    printlk(LK_INFO, "%s:%d: sai%d: starting tx with %zuB ready in circular buffer\n",
            __PRETTY_FUNCTION__, __LINE__, state->bus_id, cbuf_level);

    if (imx_is_hw_state(hw_state, kSAI_hw_Flushing)) {
        imx_clear_hw_state(hw_state, kSAI_hw_Flushing);
        flushing_cb = true;
        cbuf_reset(&state->tx_circ_buf);
        panic("Starting while still flushing\n");
    } else {
        sai_transfer_format_t *format = &state->sai_tx_fmt;
        unsigned frame_size = (format->audio_channel * format->bitWidth / 8);
        imx_sai_reset_period(&state->tx_period, frame_size);

        /* NOTE: Fill SAI-TX FIFO before Start SAI-TX Interrupts.
         * To fix SAI-TX data skipped due to race condition
         * on frame-sync signal of SAI-TX.
         * This race condition can be occur when FIFO has no data
         * while SAI-TX pending interrupt are comes. */
        imx_sai_tx_prefill_fifo(dev);

        if (state->sai_tx_handle.dma_enable) {
            /* Resume DMA channel, if allocated and configured */
            if (state->sai_tx_handle.dma_chan) {

                printlk(LK_INFO, "Resuming dma channel 0x%p...\n", state->sai_tx_handle.dma_chan);
                dma_resume_channel(state->sai_tx_handle.dma_chan);

#if defined(DMA_BUFFER_MEMALIGN_TYPE) && (DMA_BUFFER_MEMALIGN_TYPE == 1)
                arch_clean_cache_range((vaddr_t) buf, len);
#endif /* DMA_BUFFER_MEMALIGN_TYPE */
            }
        }

        SAI_TransferSendNonBlocking(base, tx_handle);

        if (state->is_dummy_tx) {
            printlk(LK_NOTICE, "%s:%d: SAI%d: Disable TX interrupts\n",
                    __PRETTY_FUNCTION__, __LINE__, state->bus_id);
            /* Disable all SAI tx interrupts for dummy TX */
            SAI_TxDisableInterrupts(
                    base,
                    kSAI_WordStartInterruptEnable
                  | kSAI_SyncErrorInterruptEnable
                  | kSAI_FIFOWarningInterruptEnable
#if defined(FSL_FEATURE_SAI_FIFO_COUNT) && (FSL_FEATURE_SAI_FIFO_COUNT > 1)
                  | kSAI_FIFORequestInterruptEnable
#endif
                  | kSAI_FIFOErrorInterruptEnable);
        }

        imx_set_hw_state(hw_state, kSAI_hw_Started);
        active_cb = true;
        smp_wmb();

    }
    spin_unlock_irqrestore(&state->tx_lock, lock_state);

    if (state->tx_cb) {
        int reschedule = 0;
        if (active_cb)
            reschedule |= state->tx_cb(SAI_EVENT_ACTIVE,
                                        NULL, state->tx_cb_cookie);
        if (flushing_cb)
            reschedule |= state->tx_cb(SAI_EVENT_DRAIN,
                                        NULL, state->tx_cb_cookie);
        if (reschedule)
            thread_preempt();
    }

    return 0;
}

static status_t imx_sai_tx_stop(struct device *dev)
{
    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    I2S_Type *base = state->io_base;
    ASSERT(base);

    sai_handle_t *tx_handle = &state->sai_tx_handle;
    ASSERT(tx_handle);

    struct imx_hw_state_s *hw_state = &state->tx_hw_state;

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->tx_lock, lock_state);

    SAI_TransferTerminateSend(base, tx_handle);

    SAI_TxSoftwareReset(base, kSAI_ResetTypeFIFO);

    imx_clear_hw_state(hw_state,
                        kSAI_hw_Started | kSAI_hw_Flushing | kSAI_hw_Flushed);

    /* Stop DMA channel, if enabled and configured */
    if (tx_handle->dma_enable && state->sai_tx_handle.dma_chan) {
        struct dma_descriptor *olddesc = state->sai_tx_handle.dma_chan->desc;
        dma_terminate_sync(state->sai_tx_handle.dma_chan);
        free(olddesc);
        state->sai_tx_handle.dma_chan = NULL;
    }
    smp_wmb();

    spin_unlock_irqrestore(&state->tx_lock, lock_state);
    /* reset circular buffer data while stopping */
    cbuf_reset_with_zero(&state->tx_circ_buf);

    if (tx_handle->dma_mode == kSAI_DMA_Private_Buffer) {
#if IMX_SAI_WARMUP_NR_PERIODS > 0 || IMX_SAI_WARMUP_NR_PERIODS_FRAC > 0
        void *dma_buffer = state->dma_tx_driver_data.dma_buffer;
        size_t length_to_fill = state->dma_tx_driver_data.dma_period_size
                            * state->dma_tx_driver_data.dma_nr_periods;
        memset(dma_buffer, SAI_DMA_PREFILL_PATTERN, length_to_fill);
#endif
    }


    /* Call stop() for chained SAI(s) if any
     *
     * IMPORTANT: Master SAI must be the last to start and first to stop
     * when used in a SAI chain.
     */
    if (state->tx_sai_chained_dev) {
        struct imx_sai_state *sai_chained_state =
                state->tx_sai_chained_dev->state;

        if (sai_chained_state->sai_tx_fmt.audio_channel) {
            status_t ret = imx_sai_tx_stop(state->tx_sai_chained_dev);
            if (ret)
                return ret;
        }
    }

    return 0;
}

static status_t imx_sai_tx_flush(struct device *dev)
{
    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    struct imx_hw_state_s *hw_state = &state->tx_hw_state;

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->tx_lock, lock_state);

    if (imx_is_hw_state(hw_state, kSAI_hw_Started))
        imx_set_hw_state(hw_state, kSAI_hw_Flushing);

    smp_wmb();
    spin_unlock_irqrestore(&state->tx_lock, lock_state);

    /* Call flush() for chained SAI(s) if any */
    if (state->tx_sai_chained_dev) {
        struct imx_sai_state *sai_chained_state =
                state->tx_sai_chained_dev->state;

        if (sai_chained_state->sai_tx_fmt.audio_channel) {
            status_t ret = imx_sai_tx_flush(state->tx_sai_chained_dev);
            if (ret)
                return ret;
        }
    }

    return 0;
}

static status_t imx_sai_tx_drop(struct device *dev)
{
    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    struct imx_hw_state_s *hw_state = &state->tx_hw_state;

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->tx_lock, lock_state);

    if (imx_is_hw_state(hw_state, kSAI_hw_Started))
        imx_set_hw_state(hw_state, kSAI_hw_Flushing);

    smp_wmb();

    /* Reset end of circular buffer */
    cbuf_rewind(&state->tx_circ_buf);

    spin_unlock_irqrestore(&state->tx_lock, lock_state);

    /* Call drop() for chained SAI(s) if any */
    if (state->tx_sai_chained_dev) {
        struct imx_sai_state *sai_chained_state =
                state->tx_sai_chained_dev->state;

        if (sai_chained_state->sai_tx_fmt.audio_channel) {
            status_t ret = imx_sai_tx_drop(state->tx_sai_chained_dev);
            if (ret)
                return ret;
        }
    }

    return 0;
}

static status_t imx_sai_tx_close(struct device *dev)
{
    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    I2S_Type *base = state->io_base;
    ASSERT(base);

    sai_handle_t *tx_handle = &state->sai_tx_handle;
    ASSERT(tx_handle);

    struct imx_hw_state_s *hw_state = &state->tx_hw_state;

    if (imx_is_hw_state(hw_state, kSAI_hw_Started))
        imx_sai_tx_stop(dev);

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->tx_lock, lock_state);

    free(tx_handle->buf);
    if (kSAI_DMA_Zerocopy_Uncached == tx_handle->dma_mode) {
        vmm_free_region(vmm_get_kernel_aspace(), (vaddr_t) state->tx_circ_buf.buf);
    } else {
        free(state->tx_circ_buf.buf);
    }
    tx_handle->buf = NULL;
    state->tx_circ_buf.buf = NULL;
    memset(&state->sai_tx_fmt, 0, sizeof(sai_transfer_format_t));

    /*
     * For sai master mode, after several open/close sai,
     * there will be no frame clock, and can't recover
     * anymore. Add software reset to fix this issue.
     * This is a hardware bug, and will be fix in the
     * next sai version.
     */
    SAI_TxReset(base);

    imx_clear_hw_state(hw_state, kSAI_hw_Opened);

    spin_unlock_irqrestore(&state->tx_lock, lock_state);

    /* Call close() for chained SAI(s) if any */
    if (state->tx_sai_chained_dev) {
        status_t ret = imx_sai_tx_close(state->tx_sai_chained_dev);
        if (ret)
            return ret;
    }

    return 0;
}

static status_t imx_sai_pio_setup(struct device *dev, bool is_rx)
{
    struct imx_sai_state *state = dev->state;
    sai_handle_t *sai_handle;
    sai_transfer_format_t *format;

    if (!is_rx) {
        sai_handle = &state->sai_tx_handle;
        format = &state->sai_tx_fmt;
        state->tx_period.fifo_prefill = (IMX_SAI_FIFO_PREFILL_PIO_DEFAULT * (sai_handle->channel + 1)) / format->bitWidth;
    }

    return 0;
}

static status_t imx_sai_dma_setup(struct device *dev, bool is_rx)
{
    struct imx_sai_state *state = dev->state;
    sai_handle_t *sai_handle;
    struct dma_driver_data *dma_driver_data;
    sai_transfer_format_t *format;
    struct imx_period_s *period;
    const char *channel_name;
    enum dma_data_direction dma_transfer_direction;
    unsigned bytes_per_request, total_audio_channels;
    bool ignore_dma_config;

    struct dma_chan *dma_chan = NULL;
    struct dma_descriptor *dma_desc = NULL;

    ASSERT(state);

    if (is_rx) {
        sai_handle = &state->sai_rx_handle;
        dma_driver_data = &state->dma_rx_driver_data;
        format = &state->sai_rx_fmt;
        period = &state->rx_period;
        channel_name = "rx";
        dma_transfer_direction = DMA_DEV_TO_MEM;
        ignore_dma_config = false;
    } else {
        sai_handle = &state->sai_tx_handle;
        dma_driver_data = &state->dma_tx_driver_data;
        format = &state->sai_tx_fmt;
        period = &state->tx_period;
        channel_name = "tx";
        dma_transfer_direction = DMA_MEM_TO_DEV;
        ignore_dma_config = state->tx_is_chained && state->tx_is_slave;
    }

    ASSERT(dma_driver_data->dma_buffer);


    /* FIXME: Reconfiguring the channel sometimes corrupts it,
     * so we first deallocate the one used before requesting/configuring a new one
     */
    smp_mb();
    if (sai_handle->dma_chan) {
        struct dma_descriptor *olddesc = sai_handle->dma_chan->desc;
        printlk(LK_DEBUG, "SAI%d: Terminating a pending dma channel\n", state->bus_id);
        dma_terminate_sync(sai_handle->dma_chan);
        free(olddesc);
        sai_handle->dma_chan = NULL;
    }

    /* configure DMA channel */
    struct dma_slave_config cfg_dma;
    unsigned ret;

    memset(&cfg_dma, 0, sizeof(struct dma_slave_config));

    uint32_t fifo_num = (format->audio_channel + format->slot - 1) / (format->slot);

    int slots = format->slot;

    printlk(LK_DEBUG, "SAI%d: DMA: format->audio_channel = 0x%x\n", state->bus_id, format->audio_channel);
    printlk(LK_DEBUG, "SAI%d: DMA: format->slot = 0x%x\n", state->bus_id, format->slot);
    printlk(LK_DEBUG, "SAI%d: DMA: format->channelMask = 0x%x\n", state->bus_id, format->channelMask);
    printlk(LK_DEBUG, "SAI%d: DMA: fifo_num = 0x%x\n", state->bus_id, fifo_num);
    printlk(LK_DEBUG, "SAI%d: DMA: sai_handle->channelMask = 0x%x\n", state->bus_id, sai_handle->channelMask);
    printlk(LK_DEBUG, "SAI%d: DMA: sai_handle->channelNums = 0x%x\n", state->bus_id, sai_handle->channelNums);
    printlk(LK_DEBUG, "SAI%d: DMA: sai_handle->endChannel = 0x%x\n", state->bus_id, sai_handle->endChannel);
    printlk(LK_DEBUG, "SAI%d: DMA: sai_handle->channel = 0x%x\n", state->bus_id, sai_handle->channel);
    printlk(LK_DEBUG, "SAI%d: DMA: sai_handle->burst_length = 0x%x\n", state->bus_id, sai_handle->burst_length);
    printlk(LK_DEBUG, "SAI%d: DMA: sai_handle->watermark = 0x%x\n", state->bus_id, sai_handle->watermark);

    /* Set bytes_per_request as the required amount of data to get all FIFOs full */
    bytes_per_request = sai_handle->burst_length;

    /* Set total_audio_channels as current SAI's number of channels */
    total_audio_channels = format->audio_channel;

    if (is_rx) {
        uint8_t start_lane = 0;

        for (unsigned i = 0; i < SAI_HW_MAX_LANE; i++) {
            if (format->channelMask & (1 << i)) {
                start_lane = i;
                break;
            }
        }
        printlk(LK_VERBOSE, "start_lane = %d\n", start_lane);

        cfg_dma.src_addr = SAI_RxGetDataRegisterAddress(state->io_base, start_lane);
        if (kSAI_BusI2SPDM == format->protocol)
            slots = 1; /* if PDMtoPCM is used */
    } else {
        cfg_dma.dst_addr = SAI_TxGetDataRegisterAddress(state->io_base, 0);

        if (state->tx_is_chained && !state->tx_is_slave) {
            struct device *ldev = dev;
            uint32_t multi_sai_addr = 0;

            /* Reset bytes_per_request/total_audio_channels as we will recompute them now */
            bytes_per_request = 0;
            total_audio_channels = 0;

            for (unsigned shift = 0 ; shift < 4 ; shift++) {
                struct imx_sai_state *lstate;
                uint32_t lsaicfg;

                /* Break if no more SAI in chain */
                if (!ldev)
                    break;

                /* Set current sai id */
                lstate = (struct imx_sai_state *)ldev->state;

                /* Add this SAI entry only if it actually handle channels */
                if (lstate->sai_tx_fmt.audio_channel) {
                    /* Set current sai id */
                    lsaicfg = (lstate->bus_id);

                    /* Set number of lanes */
                    lsaicfg |= (lstate->sai_tx_fmt.audio_channel / lstate->sai_tx_fmt.slot) << 4;

                    /* Finally aggregate with previous configs */
                    multi_sai_addr |= (lsaicfg << (shift<<3));

                    /* Increment bytes_per_request */
                    bytes_per_request += lstate->sai_tx_handle.burst_length;

                    /* Increment total_audio_channels */
                    total_audio_channels += lstate->sai_tx_fmt.audio_channel;
                }

                /* Move to next chained SAI */
                ldev = lstate->tx_sai_chained_dev;
            }

            printlk(LK_INFO,
                    "SAI%d: Setting up DMA destination address to 0x%x\n",
                    state->bus_id, multi_sai_addr);

            cfg_dma.dst_addr = (paddr_t)multi_sai_addr;
        }
    }
    cfg_dma.direction = dma_transfer_direction;
    if ((!is_rx) && (format->bitWidth == 16))
        cfg_dma.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
    else
        cfg_dma.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
    cfg_dma.dst_addr_width = cfg_dma.src_addr_width;
    cfg_dma.src_fifo_num = fifo_num;
    cfg_dma.dst_fifo_num = fifo_num;
    cfg_dma.src_sample_num = slots;
    cfg_dma.dst_sample_num = slots;
    cfg_dma.src_maxburst = bytes_per_request;
    cfg_dma.dst_maxburst = cfg_dma.src_maxburst;
    printlk(LK_DEBUG, "cfg_dma.src_addr_width=%d\n", cfg_dma.src_addr_width);
    printlk(LK_DEBUG, "cfg_dma.src_maxburst=%d\n", cfg_dma.src_maxburst);

    dma_driver_data->dma_period_elapsed = 0;

    /* adjust DMA buffer length to a multiple of DMA transfer size */
    unsigned dma_period_size = period->size / IMX_SAI_DMA_PERIOD_RATIO;
    unsigned remainder = dma_period_size % cfg_dma.src_maxburst;
    dma_driver_data->dma_period_size =
        remainder ? dma_period_size + cfg_dma.src_maxburst - remainder : dma_period_size;

    printlk(LK_DEBUG,
            "sai%d: period->size=%d, dma_period_size=%d, cfg_dma.src_maxburst=%d, remainder=%d, dma_driver_data->dma_period_size=%d\n",
            state->bus_id, period->size, dma_period_size, cfg_dma.src_maxburst,
            remainder, dma_driver_data->dma_period_size);

    if (dma_driver_data->dma_period_size > dma_driver_data->dma_period_size_max) {
        dma_driver_data->dma_period_size = dma_driver_data->dma_period_size_max;
    }
    printlk(LK_DEBUG, "sai%d: %s dma_period_size = %d\n", state->bus_id,
                  channel_name, dma_driver_data->dma_period_size);
    ASSERT(dma_driver_data->dma_period_size != 0);

    /* Bypass DMA configuration if requested (typically slave SAIs when chained) */
    if (ignore_dma_config) {
        printlk(LK_INFO, "SAI%d: bypass dma channel setup...\n", state->bus_id);
    } else {
        /* request a DMA channel */

        dma_chan = dma_request_chan(dev, channel_name);
        ASSERT(dma_chan);
        sai_handle->dma_chan = dma_chan;

        printlk(LK_NOTICE, "SAI%d: Configure DMA channel %p\n", state->bus_id, dma_chan);

        /* set DMA controller specific parameters */
        ret = dma_slave_config(sai_handle->dma_chan, &cfg_dma);
        if (ret) {
            printlk(LK_ERR, "%s:%d: sai%d: Failed to configure DMA %s channel.\n",
                    __PRETTY_FUNCTION__, __LINE__, state->bus_id,
                   channel_name);
            return ret;
        }

        /* Prepare and submit Back-End DMA channel */
        dma_driver_data->dma_buffer_phys =
            phys_to_dma(vaddr_to_paddr(dma_driver_data->dma_buffer));

        printlk(LK_DEBUG, "sai%d: dma_prep_dma_cyclic: paddr=%lx\n",
                    state->bus_id, dma_driver_data->dma_buffer_phys);

        dma_desc = dma_prep_dma_cyclic(
                sai_handle->dma_chan,                       /* dma channel handle */
                dma_driver_data->dma_buffer_phys,           /* buf_addr */
                (dma_driver_data->dma_period_size
                        * dma_driver_data->dma_nr_periods), /* buf_len */
                dma_driver_data->dma_period_size,           /* period_len */
                dma_transfer_direction                      /* direction */
        );

        dma_desc->dma_mode = sai_handle->dma_mode;
        dma_desc->cookie = state;
        dma_driver_data->dma_desc = dma_desc;

        if (is_rx) {
            /* RX DMA callbacks */
            dma_desc->zerocopy_cb = imx_sai_rx_dma_zerocopy_cb;
            dma_desc->cb = imx_sai_rx_dma_cb;
        } else {
            /* TX DMA callbacks */
            dma_desc->zerocopy_cb = imx_sai_tx_dma_zerocopy_cb;
            dma_desc->cb = imx_sai_tx_dma_cb;
        }
    }

    /* TX specific initialization (prefill of SAI FIFO and circular buffer) */
    if (!is_rx) {
        /* TX specific */
        cbuf_reset_indexes(&state->tx_circ_buf);

        float ratio = state->tx_period.warmup_ratio;

        state->dma_tx_driver_data.dma_period_elapsed = ratio *
            (state->tx_period.size / state->dma_tx_driver_data.dma_period_size);

        unsigned sample_size = total_audio_channels * format->bitWidth / 8;
        state->tx_period.fifo_prefill =
                ((ratio * state->tx_period.size) -
                        (state->dma_tx_driver_data.dma_period_elapsed
                                * state->dma_tx_driver_data.dma_period_size));

        state->tx_period.fifo_prefill /= sample_size;

        if (state->tx_period.fifo_prefill == 0) {
            state->tx_period.fifo_prefill = bytes_per_request / sample_size;
            printlk(LK_DEBUG, "sai%d: Inserting unnecessary samples: %d\n", state->bus_id,
                                            state->tx_period.fifo_prefill);
        }

        if (state->tx_period.fifo_prefill > IMX_SAI_PREFILL_WITH_ZERO) {
            printlk(LK_DEBUG, "sai%d: Clamping to %d samples from %d\n", state->bus_id,
                                            IMX_SAI_PREFILL_WITH_ZERO,
                                            state->tx_period.fifo_prefill);
            state->tx_period.fifo_prefill = IMX_SAI_PREFILL_WITH_ZERO;
        }

        state->tx_period.dma_period_prefill = state->dma_tx_driver_data.dma_period_elapsed;

        printlk(LK_DEBUG, "sai%d: tx om period prefill ratio = %f\n",
                state->bus_id, ratio);
        printlk(LK_DEBUG, "sai%d: tx dma periods prefill = %d\n",
                state->bus_id, state->tx_period.dma_period_prefill);
        printlk(LK_DEBUG, "sai%d: state->dma_tx_driver_data.dma_period_size=%d\n",
                state->bus_id,state->dma_tx_driver_data.dma_period_size);
        printlk(LK_DEBUG, "sai%d: state->dma_tx_driver_data.dma_nr_periods=%d\n",
                state->bus_id,state->dma_tx_driver_data.dma_nr_periods);
        printlk(LK_DEBUG, "sai%d: state->dma_tx_driver_data.dma_buffer_phys=%lx\n",
                state->bus_id,state->dma_tx_driver_data.dma_buffer_phys);
        printlk(LK_DEBUG, "sai%d: state->dma_tx_driver_data.dma_buffer=%p\n",
                state->bus_id,state->dma_tx_driver_data.dma_buffer);
        printlk(LK_DEBUG, "sai%d: state->tx_period.size=%d\n",
                state->bus_id, state->tx_period.size);
        printlk(LK_DEBUG, "sai%d: 1 om period for %d dma periods\n",
                state->bus_id,
                state->tx_period.size / state->dma_tx_driver_data.dma_period_size);

    }

    /* Bypass DMA configuration if requested (typically slave SAIs when chained) */
    if (ignore_dma_config) {
        printlk(LK_INFO, "SAI%d: bypass dma channel setup...\n", state->bus_id);
        return 0;
    }

    if (!is_rx) {
        switch (dma_desc->dma_mode) {
        case kSAI_DMA_Zerocopy_Cached:
        case kSAI_DMA_Zerocopy_Uncached: {
            unsigned nr_period = state->tx_circ_buf_max_size / state->tx_period.size;
            state->tx_circ_buf_size = nr_period * state->tx_period.size;
            if (state->tx_circ_buf_size != cbuf_size(&state->tx_circ_buf)) {
                printlk(LK_DEBUG, "sai%d: Adjusting circ buf size to %zu\n",
                    state->bus_id, state->tx_circ_buf_size);
                imx_sai_tx_adjust_buffers(state,state->tx_circ_buf_size);
            }

            DEBUG_ASSERT(cbuf_size(&state->tx_circ_buf) == state->tx_circ_buf_size);

            printlk(LK_DEBUG, "sai%d: %f dma buffer in circ buffer\n", state->bus_id,
                ((float) cbuf_size(&state->tx_circ_buf)) / state->dma_tx_driver_data.dma_period_size);

            ASSERT((state->tx_circ_buf_size % state->dma_tx_driver_data.dma_period_size) == 0);

            size_t len_adv = state->dma_tx_driver_data.dma_period_size;
            len_adv *= state->tx_period.dma_period_prefill;
            printlk(LK_DEBUG, "sai%d: Advance TX circ wr pointer (+%zu)\n",
                state->bus_id, len_adv);
            cbuf_skip(&state->tx_circ_buf, true, len_adv);
            break;
        }
        case kSAI_DMA_Private_Buffer:
        default:
            printlk(LK_DEBUG, "sai%d: zero-init dma private buffer\n",state->bus_id);
            memset(state->dma_tx_driver_data.dma_buffer, SAI_DMA_PREFILL_PATTERN,
                        state->dma_tx_driver_data.dma_period_size
                            * state->dma_tx_driver_data.dma_nr_periods);
            break;
        }
        printlk(LK_DEBUG, "sai%d: TX Init DMA period_elapsed [sw/hw] %llu/%u, fifo prefill: %d\n",
                        state->bus_id,
                        state->dma_tx_driver_data.dma_desc->period_elapsed,
                        state->tx_period.dma_period_prefill,
                        state->tx_period.fifo_prefill);

    }

    dma_submit(dma_driver_data->dma_desc);

    return 0;
}

static status_t imx_sai_tx_setup(struct device *dev, sai_format_t *pfmt)
{
    uint32_t local_num_channels;

    status_t ret;

    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    I2S_Type *base = state->io_base;
    ASSERT(base);

    sai_handle_t *tx_handle = &state->sai_tx_handle;
    ASSERT(tx_handle);

    sai_transfer_format_t *tx_format = &state->sai_tx_fmt;
    ASSERT(tx_format);

    sai_config_t config;

    int slots, channel;

    struct imx_hw_state_s *hw_state = &state->tx_hw_state;
    ASSERT(!imx_is_hw_state(hw_state, kSAI_hw_Started));

    printlk(LK_VERBOSE, "%s: SAI%d: entry\n", __PRETTY_FUNCTION__, state->bus_id);

    slots = (int)pfmt->num_slots;

    /* Configuration coherency check */
    ASSERT(state->is_dummy_tx ||
           (unsigned)(pfmt->num_channels / slots) <= imx_sai_get_max_lanes(dev, false, true));

    /* Compute number of channels which can be handled by current SAI */
    local_num_channels = MIN(pfmt->num_channels, imx_sai_get_max_lanes(dev, false, false) * slots);

    printlk(LK_NOTICE, "%s:%d: SAI%d: Request for %d audio channels. Will handle %d on current SAI\n",
            __PRETTY_FUNCTION__, __LINE__, state->bus_id, pfmt->num_channels, local_num_channels);

    /* Call setup() for chained SAI(s) if any */
    if (state->tx_sai_chained_dev) {
        sai_format_t next_sai_format = *pfmt;

        /* Leaving remaining audio channels to next SAI(s) in chain */
        next_sai_format.num_channels -= local_num_channels;

        if (next_sai_format.num_channels) {
            ret = imx_sai_tx_setup(state->tx_sai_chained_dev, &next_sai_format);
            if (ret)
                return ret;
        }
    }

    channel = (((local_num_channels) + (slots) - 1) / (slots)); //0U;
    if (channel > 0)
        channel -= 1;

    if (slots > 2) {
        printlk(LK_NOTICE, "%s:%d: SAI%d TX TDM%d (aka TDM%d) configuration requested.\n",
                __PRETTY_FUNCTION__, __LINE__, state->bus_id,
                slots, slots * 32);
    } else {
        printlk(LK_NOTICE, "%s:%d: SAI%d TX TDM feature not used.\n",
                __PRETTY_FUNCTION__, __LINE__, state->bus_id);
    }


    /* Master/ Slave mode selection */
    if ( (state->tx_is_slave ? ksai_Slave : ksai_Master) != pfmt->master_slave) {
        printlk(LK_INFO, "%s:%d: SAI%d TX configuration (%s) overrides HAL configuration (%s)\n",
                __PRETTY_FUNCTION__, __LINE__,
                state->bus_id, state->tx_is_slave ? "slave" : "master",
                pfmt->master_slave == ksai_Slave ? "slave" : "master");
        pfmt->master_slave = state->tx_is_slave ? ksai_Slave : ksai_Master;
    }

#ifdef IMX_SAI_TX_SETUP_LOCK
    /*
     * Even though this routine is not supposed to be call with the stream
     * enabled, handle this within a critical section
     * Do the memory allocation not in the critical section
     */

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->tx_lock, lock_state);
#endif

    /*
     * config->bclkSource = kSAI_BclkSourceMclkDiv;
     * config->masterSlave = kSAI_Master;
     * config->mclkSource = kSAI_MclkSourceSysclk;
     * config->protocol = kSAI_BusLeftJustified;
     * config->syncMode = kSAI_ModeAsync;
     */
    SAI_TxGetDefaultConfig(&config);
    config.protocol = pfmt->sai_protocol;
#if defined(FSL_FEATURE_SAI_HAS_MCR) && (FSL_FEATURE_SAI_HAS_MCR)
    config.mclkOutputEnable = (state->mclk_as_output == true);
    if ((1 == state->clk_mode) && (NULL != state->gpio_mclk_ctrl.dev)) {
        config.mclkOutputEnable = false;
    }
#endif /* FSL_FEATURE_SAI_HAS_MCR */
    config.mclkSource = state->mclk_select_tx;
    config.syncMode = state->tx_sync_mode;
    config.masterSlave = pfmt->master_slave;

    /* Control MCLK external source (if present) */
    /* If mclk is controlled by a GPIO pin... */
    if (NULL != state->gpio_mclk_ctrl.dev) {
        /* Temporarily force mclk output to be disabled to prevent electrical
         * conflict on MCLK pad while switching external control */
        SAI_SetMclkOutputEnable(base, false);

        /* ... just clear that pin when clk_mode is set (i.e. external mclk) */
        gpio_desc_set_value(&state->gpio_mclk_ctrl, !(state->clk_mode));
        printlk(LK_INFO, "%s:%d: sai%d: External mclk control GPIO turned %s\n",
                __PRETTY_FUNCTION__, __LINE__, state->bus_id,
                (state->clk_mode) ? "off" : "on");
    }

    /* Configure SAI TX registers (includes updating mclk output configuration) */
    SAI_TxInit(base, &config);

    /* Change polarity if needed */
    if (pfmt->polarity != SAI_BITCLOCK_POLARITY_UNUSED)
        SAI_TxSetBitClockPolarity(base, pfmt->polarity);

    /* Configure the audio format */
    tx_format->bitWidth = pfmt->bitWidth;
    tx_format->channel = channel;

    tx_format->sampleRate_Hz = pfmt->sampleRate_Hz;
    tx_format->masterClockHz = state->mclk_tx_rate;
    tx_format->protocol = config.protocol;
    if (pfmt->num_channels >= 2)
        tx_format->stereo = kSAI_Stereo;
    else
        tx_format->stereo = kSAI_MonoRight;

    if (local_num_channels != pfmt->num_channels) {
        printlk(LK_NOTICE, "%s:%d: SAI%d: Set audio_channel to %d for current SAI\n",
            __PRETTY_FUNCTION__, __LINE__, state->bus_id, local_num_channels);
    }
    tx_format->audio_channel = local_num_channels;

    tx_format->slot = slots;

    printlk(LK_INFO, "%s:%d: \nsai%d: ---format->audio_channel : %d \n",
            __PRETTY_FUNCTION__, __LINE__, state->bus_id, tx_format->audio_channel);
    printlk(LK_INFO, "%s:%d: \nsai%d: ---DATA Channel : %d \n",
            __PRETTY_FUNCTION__, __LINE__, state->bus_id, tx_format->channel);

#if defined(FSL_FEATURE_SAI_FIFO_COUNT) && (FSL_FEATURE_SAI_FIFO_COUNT > 1)
    if (pfmt->period_size > IMX_SAI_PERIOD_SIZE_TX) {
        printlk(LK_NOTICE, "%s:%d: sai%d: TX Period size (%d) must be greater than %d frames\n",
               __PRETTY_FUNCTION__, __LINE__, state->bus_id, pfmt->period_size, IMX_SAI_PERIOD_SIZE_TX);
    }

    tx_format->watermark = MIN(
                    pfmt->period_size / IMX_SAI_DMA_PERIOD_RATIO,
                    IMX_SAI_FIFO_WATERMARK_TX);
    tx_format->watermark = FSL_FEATURE_SAI_FIFO_COUNT - tx_format->watermark;
    printlk(LK_INFO, "%s:%d: ---sai%d: TX Watermark: %d \n",
            __PRETTY_FUNCTION__, __LINE__, state->bus_id, tx_format->watermark);
#endif

    tx_format->isFrameSyncCompact = true;

    state->tx_period.size =
        pfmt->period_size * pfmt->num_channels * tx_format->bitWidth / 8;

    state->tx_period.frames = pfmt->period_size;
#if IMX_SAI_WARMUP_NR_PERIODS_FRAC > 0
    state->tx_period.warmup_ratio = IMX_SAI_WARMUP_NR_PERIODS
                        + (1.0 / IMX_SAI_WARMUP_NR_PERIODS_FRAC);
#else
    state->tx_period.warmup_ratio = IMX_SAI_WARMUP_NR_PERIODS;
#endif

    /* set CCM SAI ROOT MCLK input */

    imx_sai_set_mclk(state, tx_format->sampleRate_Hz, true, NULL);

    tx_format->masterClockHz = state->mclk_tx_rate;

    /* FIXME: Translate kstatus to LK status */
    ret = SAI_TransferTxSetFormat(
              base,
              tx_handle,
              tx_format,
              state->mclk_tx_rate,
              tx_format->masterClockHz);
    printlk(LK_VERBOSE, "sai%d: FIFO watermark: %d\n", state->bus_id, tx_handle->watermark);

    /* adjust dataline mask */
    uint32_t fifo_num = (tx_format->audio_channel) / (tx_format->slot);
    uint32_t fifo_mask = (1 << fifo_num) - 1;
    printlk(LK_VERBOSE, "sai%d: Setting TX FIFO mask to 0x%x\n", state->bus_id, fifo_mask);
    SAI_TxSetChannelFIFOMask(base, fifo_mask);

    /* set FCOMB according to device tree */
    base->TCR4 &= ~I2S_TCR4_FCOMB_MASK;
    base->TCR4 |= (state->tx_fcomb << I2S_TCR4_FCOMB_SHIFT);

    if (state->sai_tx_handle.dma_enable)
        imx_sai_dma_setup(dev, false);
    else
        imx_sai_pio_setup(dev, false);

    imx_set_hw_state(hw_state, kSAI_hw_Prepared);

    smp_wmb();

#ifdef IMX_SAI_TX_SETUP_LOCK
    spin_unlock_irqrestore(&state->tx_lock, lock_state);
#endif

    printlk(LK_DEBUG, "Tx period: (%d frames)(%d Bytes)\n",
        pfmt->period_size, state->tx_period.size);
    printlk(LK_DEBUG, "TX circular buffer size: %ld\n", state->tx_circ_buf_size);
    printlk(LK_DEBUG, "SAI_Tx init Done: %d/%d (i2s/audio ch)"
           ", %d (sample rate).\n",
           tx_format->channel,
           tx_format->audio_channel,
           tx_format->sampleRate_Hz);

    return ret;
}

static status_t imx_sai_write(struct device *dev, const void *buf, size_t len)
{
    size_t avail, remaining, written;

    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    mutex_acquire(&state->tx_mutex);
    remaining = len;

    printlk(LK_VERBOSE, "sai%d: len: %ld\n", state->bus_id, len);

    while (remaining) {
        avail = cbuf_space_avail(&state->tx_circ_buf);
        if (remaining > avail) {
            printlk(LK_NOTICE, "%s:%d: SAI%d: No more room in tx circ buffer (avail %ld)."
                " Dropping %ld bytes\n",
                __PRETTY_FUNCTION__, __LINE__, state->bus_id, avail, remaining);
            written = remaining;
        } else {
            written = cbuf_write(&state->tx_circ_buf, buf, remaining, false);
        }
        /* FIXME: Caller is likely doing wrong if it expects the driver to start
         * the stream on its own
         * */
        remaining -= written;
        buf += written;
    }

    printlk(LK_VERBOSE, "sai%d: len: %ld..done\n", state->bus_id, len);
    mutex_release(&state->tx_mutex);

    return 0;
}

static status_t imx_sai_rx_stop(struct device *dev)
{
    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);
    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    I2S_Type *base = state->io_base;
    ASSERT(base);

    sai_handle_t *rx_handle = &state->sai_rx_handle;
    ASSERT(rx_handle);

    struct imx_hw_state_s *hw_state = &state->rx_hw_state;

    /* Save the pointers we are going to free later */
    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->rx_lock, lock_state);

    rx_handle->xfer_remaining = ERR_IO;

    imx_sai_rx_stop_hw(state);
    struct sai_ioc_cmd_timestamp ts = {0, 0, 0};
    ts.stop = current_time_hires();
    imx_sai_set_timestamp(dev, &ts);
    SAI_RxSoftwareReset(base, kSAI_ResetTypeFIFO);
    SAI_RxReset(base);

    imx_clear_hw_state(hw_state, kSAI_hw_Started);

    if (rx_handle->dma_enable) {
        SAI_RxEnableDMA(base, kSAI_FIFORequestDMAEnable, false);
        dma_pause_channel(rx_handle->dma_chan);
    }

    smp_wmb();
    spin_unlock_irqrestore(&state->rx_lock, lock_state);

    event_signal(&state->rx_wait, false);

    return 0;
}

static status_t imx_sai_rx_flush(struct device *dev)
{
    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    sai_handle_t *rx_handle = &state->sai_rx_handle;
    ASSERT(rx_handle);

    struct imx_hw_state_s *hw_state = &state->rx_hw_state;

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->rx_lock, lock_state);

    if (rx_handle->xfer_remaining > 0)
        imx_set_hw_state(hw_state, kSAI_hw_Flushing);
    else
        cbuf_reset(&state->rx_circ_buf);

    smp_wmb();

    spin_unlock_irqrestore(&state->rx_lock, lock_state);

    return 0;
}

static status_t imx_sai_rx_open(struct device *dev)
{
    void *buf, *circ_buf;
    size_t buf_size, circ_buf_size;
    const char * circ_buf_name = "sai rx circ buf";
    status_t ret_alloc;

    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    sai_handle_t *rx_handle = &state->sai_rx_handle;
    ASSERT(rx_handle);

    struct imx_hw_state_s *hw_state = &state->rx_hw_state;

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    buf_size = SAI_RX_BUFFER_SIZE;
    buf = malloc(buf_size);
    ASSERT(buf);
    rx_handle->buf = buf;
    rx_handle->buf_size = buf_size;

    circ_buf_size = state->rx_circ_buf_size;

    switch (rx_handle->dma_mode)
    {
    case kSAI_DMA_Zerocopy_Uncached:
        ret_alloc = vmm_alloc_contiguous(vmm_get_kernel_aspace(),
                                            circ_buf_name,
                                            circ_buf_size,
                                            &circ_buf,
                                            8 /* log2_align */,
                                            0 /* vmm flags */,
                                            ARCH_MMU_FLAG_UNCACHED);
        if (ret_alloc < 0) {
            printlk(LK_ERR, "%s:%d: failed to allocate memory for %s.\n",
                    __PRETTY_FUNCTION__, __LINE__, circ_buf_name);
            return ERR_NO_MEMORY;
        }
        /* Use circular buffer as DMA buffer */
        state->dma_rx_driver_data.dma_buffer = circ_buf;
        break;

    case kSAI_DMA_Zerocopy_Cached:
        circ_buf = memalign(DMA_BUFFER_ALIGNMENT, circ_buf_size);
        /* Use circular buffer as DMA buffer */
        state->dma_rx_driver_data.dma_buffer = circ_buf;
        break;

    default:
        /* Standard DMA mode with intermediate buffer */
        circ_buf = malloc(circ_buf_size);
        break;
    }

    ASSERT(circ_buf);

    printlk(LK_INFO, "rx circ buf (size / address) : %ldbytes / 0x%p\n",
                  circ_buf_size,
                  circ_buf);

    cbuf_initialize_etc(&state->rx_circ_buf, circ_buf_size, circ_buf);

    cbuf_set_flag(&state->rx_circ_buf, CBUF_FLAG_USE_MAX_CHUNK_R);

    if (rx_handle->dma_mode == kSAI_DMA_Private_Buffer)
        cbuf_set_flag(&state->rx_circ_buf, CBUF_FLAG_SW_IS_WRITER);
    else
        cbuf_clear_flag(&state->rx_circ_buf, CBUF_FLAG_SW_IS_WRITER);

    rx_handle->cbuf_watermark = circ_buf_size;
    rx_handle->cbuf_watermark -= RX_CBUF_N_FIFO_WATERMARK * buf_size;

    imx_set_hw_state(hw_state, kSAI_hw_Opened);
    return 0;
}

static status_t imx_sai_rx_close(struct device *dev)
{
    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);
    struct imx_hw_state_s *hw_state = &state->rx_hw_state;

    if (imx_is_hw_state(hw_state, kSAI_hw_Started | kSAI_hw_Running))
        imx_sai_rx_stop(dev);

    sai_handle_t *rx_handle = &state->sai_rx_handle;
    ASSERT(rx_handle);
    free(rx_handle->buf);
    ASSERT(state->rx_circ_buf.buf);
    if (kSAI_DMA_Zerocopy_Uncached == rx_handle->dma_mode) {
        vmm_free_region(vmm_get_kernel_aspace(), (vaddr_t) state->rx_circ_buf.buf);
    } else {
        free(state->rx_circ_buf.buf);
    }
    rx_handle->buf = NULL;
    state->rx_circ_buf.buf = NULL;
    memset(&state->sai_rx_fmt, 0, sizeof(sai_transfer_format_t));
    imx_clear_hw_state(hw_state, kSAI_hw_Prepared | kSAI_hw_Opened);

    return 0;
}

static status_t imx_sai_rx_start_hw(struct imx_sai_state *state)
{
    status_t ret;
    I2S_Type *base = state->io_base;
    ASSERT(base);

    sai_handle_t *handle = &state->sai_rx_handle;
    ASSERT(handle);

    struct imx_hw_state_s *hw_state = &state->rx_hw_state;

    if (!imx_is_hw_state(hw_state, kSAI_hw_Started))
        return 0;

    if (imx_is_hw_state(hw_state, kSAI_hw_Running))
        return 0;

    printlk(LK_INFO, "Starting Sai rx hardware\n");
    ret = SAI_TransferReceiveNonBlocking(base, handle);

    if (state->is_dummy_rx) {
        printlk(LK_NOTICE, "%s:%d: SAI%d: Disable RX interrupts",
                __PRETTY_FUNCTION__, __LINE__, state->bus_id);
        /* Disable all SAI rx interrupts for dummy RX */
        SAI_RxDisableInterrupts(
                base,
                kSAI_WordStartInterruptEnable
              | kSAI_SyncErrorInterruptEnable
              | kSAI_FIFOWarningInterruptEnable
#if defined(FSL_FEATURE_SAI_FIFO_COUNT) && (FSL_FEATURE_SAI_FIFO_COUNT > 1)
              | kSAI_FIFORequestInterruptEnable
#endif
              | kSAI_FIFOErrorInterruptEnable);
    }

    if (ret == 0)
        imx_set_hw_state(hw_state, kSAI_hw_Running);

    return ret;
}

static status_t imx_sai_rx_start(struct device *dev)
{
    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    I2S_Type *base = state->io_base;
    ASSERT(base);

    sai_handle_t *rx_handle = &state->sai_rx_handle;
    struct imx_hw_state_s *hw_state = &state->rx_hw_state;

    ASSERT(rx_handle);
    DEBUG_ASSERT(imx_is_hw_state(hw_state, kSAI_hw_Prepared));

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->rx_lock, lock_state);

    sai_transfer_format_t *format = &state->sai_rx_fmt;
    unsigned frame_size = (format->audio_channel * format->bitWidth / 8);
    imx_sai_reset_period(&state->rx_period, frame_size);

    struct sai_ioc_cmd_timestamp tstamp = {0, 0, 0};
    tstamp.start = current_time_hires();
    imx_sai_set_timestamp(dev, &tstamp);
    if (rx_handle->dma_enable) {
        switch (rx_handle->dma_mode)
        {
        case kSAI_DMA_Zerocopy_Uncached:
        case kSAI_DMA_Zerocopy_Cached:
            cbuf_reset_indexes(&state->rx_circ_buf);
            break;
        case kSAI_DMA_Private_Buffer:
        default:
            break;
        }

        dma_resume_channel(state->sai_rx_handle.dma_chan);
    }

    smp_wmb();
    imx_set_hw_state(hw_state, kSAI_hw_Started);

    /* Reset RX FIFOs */
    SAI_RxSoftwareReset(state->io_base, kSAI_ResetTypeFIFO);

    imx_sai_rx_start_hw(state);

    spin_unlock_irqrestore(&state->rx_lock, lock_state);

    return 0;
}

static status_t imx_sai_rx_setup(struct device *dev, sai_format_t *pfmt)
{
    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);
    status_t ret;
    uint8_t i;

    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    I2S_Type *base = state->io_base;
    ASSERT(base);

    sai_handle_t *rx_handle = &state->sai_rx_handle;
    ASSERT(rx_handle);

    sai_transfer_format_t *rx_format = &state->sai_rx_fmt;
    ASSERT(rx_format);

    struct imx_hw_state_s *hw_state = &state->rx_hw_state;

    sai_config_t config;
    int slots, channel;

    /*
     * Even though this routine is not supposed to be called with the stream
     * enabled, handle this within a critical section.
     * Do the memory allocation not in the critical section
     */

    ASSERT(!imx_is_hw_state(hw_state, kSAI_hw_Started));

    slots = (int)pfmt->num_slots;

    channel = (((pfmt->num_channels) + (slots) - 1) / (slots)); //0U;
    if (channel > 0)
        channel -= 1;

    if (slots > 2) {
        printlk(LK_NOTICE, "%s:%d: SAI%d RX TDM%d (aka TDM%d) configuration requested.\n",
                __PRETTY_FUNCTION__, __LINE__, state->bus_id,
                slots, slots * 32);
    } else {
        printlk(LK_NOTICE, "%s:%d: SAI%d RX TDM feature not used.\n",
                __PRETTY_FUNCTION__, __LINE__, state->bus_id);
    }

    ASSERT(state->is_dummy_rx ||
           (unsigned)(pfmt->num_channels / slots) <= imx_sai_get_max_lanes(dev, true, true));

    /* Master/ Slave mode selection */
    if ( (state->rx_is_slave ? ksai_Slave : ksai_Master) != pfmt->master_slave) {
        printlk(LK_INFO, "%s:%d: SAI%d RX configuration (%s) overrides HAL configuration (%s)\n",
                __PRETTY_FUNCTION__, __LINE__,
                state->bus_id, state->rx_is_slave ? "slave" : "master",
                pfmt->master_slave == ksai_Slave ? "slave" : "master");
        pfmt->master_slave = state->rx_is_slave ? ksai_Slave : ksai_Master;
    }

#ifdef IMX_SAI_RX_SETUP_LOCK
    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->rx_lock, lock_state);
#endif

    SAI_RxReset(base);
    SAI_RxGetDefaultConfig(&config);
    /* FIXME: Async/Sync Mode should be defined at platform level, so
     * configurable
     * */

    config.masterSlave = pfmt->master_slave;
    config.syncMode = state->rx_sync_mode;
    config.bclkSource = pfmt->bitclock_source;
    config.protocol = pfmt->sai_protocol;
#if defined(FSL_FEATURE_SAI_HAS_MCR) && (FSL_FEATURE_SAI_HAS_MCR)
    config.mclkOutputEnable = (state->mclk_as_output == true);
    if ((1 == state->clk_mode) && (NULL != state->gpio_mclk_ctrl.dev)) {
        config.mclkOutputEnable = false;
    }
#endif /* FSL_FEATURE_SAI_HAS_MCR */
    config.mclkSource = state->mclk_select_rx;

    /* Control MCLK external source (if present) */
    /* If mclk is controlled by a GPIO pin... */
    if (NULL != state->gpio_mclk_ctrl.dev) {
        /* Temporarily force mclk output to be disabled to prevent electrical
         * conflict on MCLK pad while switching external control */
        SAI_SetMclkOutputEnable(base, false);
        /* ... just clear that pin when clk_mode is set (i.e. external mclk) */
        gpio_desc_set_value(&state->gpio_mclk_ctrl, !(state->clk_mode));
        printlk(LK_INFO, "%s:%d: External mclk control GPIO turned %s\n",
                __PRETTY_FUNCTION__, __LINE__,
                (state->clk_mode) ? "off" : "on");
    }

    /* Configure SAI RX registers (includes updating mclk output configuration) */
    SAI_RxInit(base, &config);

    /* Change polarity if needed */
    if (pfmt->polarity != SAI_BITCLOCK_POLARITY_UNUSED)
        SAI_RxSetBitClockPolarity(base, pfmt->polarity);

    /* Configure the audio format */
    rx_format->bitWidth = pfmt->bitWidth;
    /*
    To set format-> channel with 0,1,2,3 ( data lines) with
    audio channel 2,4,6 and 8 respectively.
    pfmt->num_channels -> number of audio channels
    format->channel -> Data channel used in transfer
    */
    rx_format->channel = channel;
    rx_format->sampleRate_Hz = pfmt->sampleRate_Hz;

    /*
     * Set mclk rate and associated to pll/clock config.
     * This is required here as mclk setup done during
     * initialization phase can be done while pll configuration
     * is still not active, leading to a corrupted mclk rate
     */
    imx_sai_set_mclk(state, rx_format->sampleRate_Hz, false, NULL);

#if (defined FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER && FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER) || \
    (defined FSL_FEATURE_PCC_HAS_SAI_DIVIDER && FSL_FEATURE_PCC_HAS_SAI_DIVIDER)
#error "Untested path: FIXME"
    rx_format->masterClockHz = OVER_SAMPLE_RATE * format->sampleRate_Hz;
#else
    rx_format->masterClockHz = state->mclk_rx_rate;
#endif
    rx_format->protocol = config.protocol;

    if (pfmt->num_channels >= 2)
        rx_format->stereo = kSAI_Stereo;
    else
        rx_format->stereo = kSAI_MonoRight;

    rx_format->audio_channel = pfmt->num_channels;
    rx_format->slot = slots;

    rx_format->isFrameSyncCompact = false;

#if defined(FSL_FEATURE_SAI_FIFO_COUNT) && (FSL_FEATURE_SAI_FIFO_COUNT > 1)
    rx_format->watermark = IMX_SAI_FIFO_WATERMARK_RX - 1;
#endif

    state->rx_period.size =
        pfmt->period_size * rx_format->audio_channel * rx_format->bitWidth / 8;

    state->rx_period.frames = pfmt->period_size;

    /* FIXME: Translate kstatus to LK status */
    ret = SAI_TransferRxSetFormat(
              base,
              rx_handle,
              rx_format,
              state->mclk_rx_rate,
              rx_format->masterClockHz);
    printlk(LK_VERBOSE, "FIFO watermark: %d\n", rx_handle->watermark);

    /* adjust dataline mask */
    uint32_t fifo_num = (rx_format->audio_channel) / (rx_format->slot);
    uint32_t fifo_mask = (1 << fifo_num) - 1;
    uint8_t start_lane = 0;

    for (i = 0; i < SAI_HW_MAX_LANE; i++) {
        if (rx_format->channelMask & (1 << i)) {
            start_lane = i;
            break;
        }
    }
    printlk(LK_VERBOSE, "start_lane = %d\n", start_lane);

    if (rx_format->channelMask)
        fifo_mask = rx_format->channelMask;

    SAI_RxSetChannelFIFOMask(base, fifo_mask);
    printlk(LK_VERBOSE, "Setting RX FIFO mask to 0x%x\n", fifo_mask);

    /* set FCOMB according to device tree */
    base->RCR4 &= ~I2S_RCR4_FCOMB_MASK;
    base->RCR4 |= (state->rx_fcomb << I2S_RCR4_FCOMB_SHIFT);

#ifdef IMX_SAI_RX_SETUP_LOCK
    /* Release spinlock temporarily to explicitly insert a preemption point */
    spin_unlock_irqrestore(&state->rx_lock, lock_state);
    spin_lock_irqsave(&state->rx_lock, lock_state);
#endif

    if (rx_handle->dma_enable)
        imx_sai_dma_setup(dev, true);

    imx_set_hw_state(hw_state, kSAI_hw_Prepared);

    smp_wmb();

#ifdef IMX_SAI_RX_SETUP_LOCK
    spin_unlock_irqrestore(&state->rx_lock, lock_state);
#endif

    printlk(LK_DEBUG, "Rx period size: %d\n", state->rx_period.size);
    printlk(LK_DEBUG, "RX circular buffer size: %ld\n", state->rx_circ_buf_size);
    printlk(LK_DEBUG, "SAI_Rx init Done: %d/%d (i2s/audio ch)"
           ", %d (sample rate).\n",
           rx_format->channel,
           rx_format->audio_channel,
           rx_format->sampleRate_Hz);

    return ret;
}

static status_t imx_sai_rx_en_counters(struct device *dev, bool en)
{
    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    I2S_Type *base = state->io_base;
    ASSERT(base);

    struct imx_hw_state_s *hw_state = &state->rx_hw_state;
    /* Prevent preemption while enabling counters */
    spin_lock_saved_state_t lock_state;
    spin_lock_t *lock = &state->rx_bitcnt_lock;

    spin_lock_irqsave(lock, lock_state);

    SAI_TransferEnCounters(base, en, true);

    if (en)
        imx_set_hw_state(hw_state, kSAI_hw_Counting);
    else
        imx_clear_hw_state(hw_state, kSAI_hw_Counting);

    spin_unlock_irqrestore(lock, lock_state);

    return 0;
}

static status_t imx_sai_tx_en_counters(struct device *dev, bool en)
{
    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    I2S_Type *base = state->io_base;
    ASSERT(base);

    sai_handle_t *tx_handle = &state->sai_tx_handle;
    ASSERT(tx_handle);

    struct imx_hw_state_s *hw_state = &state->tx_hw_state;

    /* Prevent preemption while enabling counters */
    spin_lock_saved_state_t lock_state;
    spin_lock_t *lock = &state->tx_bitcnt_lock;

    spin_lock_irqsave(lock, lock_state);

    SAI_TransferEnCounters(base, en, false);

    if (en)
        imx_set_hw_state(hw_state, kSAI_hw_Counting);
    else
        imx_clear_hw_state(hw_state, kSAI_hw_Counting);

    spin_unlock_irqrestore(lock, lock_state);

    return 0;
}

static status_t imx_sai_en_counters(struct device *dev, int request)
{
    bool enable = !!(request & SAI_IOC_CNT_EN_ENABLE);
    bool is_read = !!(request & SAI_IOC_CNT_EN_IS_READ);
    bool is_spdif = !!(request & SAI_IOC_CNT_EN_MODE_SPDIF);

    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    I2S_Type *base = state->io_base;
    ASSERT(base);

    if (is_spdif) {
        sai_config_t config;
        SAI_RxGetDefaultConfig(&config);
        config.masterSlave = kSAI_Master;
        config.bclkSource = kSAI_BclkSourceOtherSai0;
        config.protocol = kSAI_BusI2S;
        config.syncMode = kSAI_ModeAsync;
        config.mclkSource = state->mclk_select_rx;
        SAI_RxInit(base, &config);
        base->RCR2 &= ~I2S_TCR2_BYP_MASK;
        base->RCR2 &= ~I2S_TCR2_DIV_MASK;
        base->RCR5 = I2S_RCR5_WNW(31) | I2S_RCR5_W0W(31) | I2S_RCR5_FBT(31);

        /* Enable Rx transfer */
        SAI_RxEnable(base, true);

        return 0;
    }

    if (is_read) {
        return imx_sai_rx_en_counters(dev, enable);
    } else {
        return imx_sai_tx_en_counters(dev, enable);
    }
}

#ifndef SAI_CNT_MAX_GETCNT_TIME_DEVIATION_US
#define SAI_CNT_MAX_GETCNT_TIME_DEVIATION_US 5
#endif

#ifndef SAI_CNT_MAX_GETCNT_RETRY
#define SAI_CNT_MAX_GETCNT_RETRY 3
#endif

static inline void __imx_sai_get_counters(
                            I2S_Type *base, spin_lock_t *lock, bool is_read,
                            uint32_t *bitc, uint32_t *ts, uint64_t *cpu_ts)
{
    unsigned retry_cnt =  SAI_CNT_MAX_GETCNT_RETRY;
    uint64_t now, elapsed;
    /* Prevent preemption while reading counters */
    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(lock, lock_state);
retry:
    now = current_time_hires();
    SAI_TransferGetBitCounters(base, bitc, ts, is_read);
    *cpu_ts = current_time_hires();
    elapsed = *cpu_ts - now;
    if (retry_cnt && (elapsed > SAI_CNT_MAX_GETCNT_TIME_DEVIATION_US)) {
        printlk(LK_NOTICE, "%s:%d: More than %llu us elapsed ! Retrying (%d)...\n",
                __PRETTY_FUNCTION__, __LINE__, elapsed, retry_cnt);
        retry_cnt--;
        goto retry;
    }

    spin_unlock_irqrestore(lock, lock_state);

    printlk(LK_VERBOSE, "bitc: %x ts: %x\n", *bitc, *ts);
}

static status_t imx_sai_rx_get_counters(struct device *dev,
                            uint32_t *bitc, uint32_t *ts, uint64_t *cpu_ts)
{
    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    I2S_Type *base = state->io_base;
    ASSERT(base);

    struct imx_hw_state_s *hw_state = &state->rx_hw_state;

    DEBUG_ASSERT(imx_is_hw_state(hw_state, kSAI_hw_Counting));

    __imx_sai_get_counters(base, &state->rx_bitcnt_lock, true, bitc, ts, cpu_ts);

    return 0;
}

static status_t imx_sai_tx_get_counters(struct device *dev,
                            uint32_t *bitc, uint32_t *ts, uint64_t *cpu_ts)
{
    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    I2S_Type *base = state->io_base;
    ASSERT(base);

    struct imx_hw_state_s *hw_state = &state->tx_hw_state;

    DEBUG_ASSERT(imx_is_hw_state(hw_state, kSAI_hw_Counting));

    __imx_sai_get_counters(base, &state->tx_bitcnt_lock, false, bitc, ts, cpu_ts);

    return 0;
}

static status_t imx_sai_get_counters(struct device *dev,
                                        struct sai_ioc_cmd_cnt *cmd)
{
    if (!cmd)
        return ERR_INVALID_ARGS;

    if (cmd->is_read)
        return imx_sai_rx_get_counters(dev, &cmd->bitc, &cmd->ts, &cmd->cpu_ts);
    else
        return imx_sai_tx_get_counters(dev, &cmd->bitc, &cmd->ts, &cmd->cpu_ts);
}

static status_t imx_sai_set_clk_mode(struct device *dev, int mode)
{
    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    struct imx_sai_state *state = dev->state;

    printlk(LK_INFO, "%s:%d: Switching sai %d to clock mode %d\n",
            __PRETTY_FUNCTION__, __LINE__, state->bus_id, mode);
    if (state)
        state->clk_mode = mode;

    return 0;
}

static status_t imx_sai_get_timestamp(struct device *dev,
                                        struct sai_ioc_cmd_timestamp *ts)
{
    struct imx_sai_state *state = dev->state;

    if (ts->is_read) {
        ts->start = state->sai_rx_handle.ts_start;
        ts->stop = state->sai_rx_handle.ts_stop;
    } else {
        ts->start = state->sai_tx_handle.ts_start;
        ts->stop = state->sai_tx_handle.ts_stop;
    }

    return 0;
}

static status_t imx_sai_set_timestamp(struct device *dev,
                                        struct sai_ioc_cmd_timestamp *ts)
{
    struct imx_sai_state *state = dev->state;

    if ((ts->is_read == false) && (ts->start > 0))
        state->sai_tx_handle.ts_start = ts->start;
    else
        return ERR_INVALID_ARGS;

    return 0;
}

static status_t imx_sai_get_latency(struct device *dev,
                                        struct sai_ioc_cmd_latency *cmd)
{
    struct imx_sai_state *state = dev->state;

    if (cmd->is_read) {
        /* Does not make so much sense... at least for now */
        cmd->activation = 0;
        cmd->warmup = 0;
    } else {
        sai_transfer_format_t *tx_format = &state->sai_tx_fmt;
        struct imx_period_s *period = &state->tx_period;
        unsigned samples = period->frames * period->warmup_ratio;
        if (state->sai_tx_handle.dma_enable == true) {
            samples =
                period->dma_period_prefill * period->frames / IMX_SAI_DMA_PERIOD_RATIO
                    + period->fifo_prefill;
        } else {
            samples = period->frames * period->warmup_ratio;
        }
        cmd->activation = 0;
        printlk(LK_VERBOSE, "Ratio: %f, period frames: %d, fifo %d, Fs: %d\n",
            period->warmup_ratio,
            period->frames,
            period->fifo_prefill,
            tx_format->sampleRate_Hz
        );
        cmd->warmup = roundf((samples * 1000000.0) / tx_format->sampleRate_Hz);
    }
    return 0;
}

static status_t imx_sai_get_buffer_size(struct device *dev,
                                            struct sai_ioc_cmd_buffer_sz *buf_sz)
{
    struct imx_sai_state *state = dev->state;

    if (buf_sz->is_read)
        buf_sz->sz = state->rx_circ_buf_size;
    else
        buf_sz->sz = state->tx_circ_buf_size;

    return 0;
}

static status_t imx_sai_set_rce(struct device *dev, uint8_t *channelMask)
{
    struct imx_sai_state *state = dev->state;
    sai_transfer_format_t *rx_format = &state->sai_rx_fmt;

    rx_format->channelMask = *channelMask;
    return 0;
}

#ifdef IMX_SAI_AUTODETECT
#include <lib/cksum.h>
#include <platform.h>

#define IMX_SAI_DBG_BUFFER_NR 4

static unsigned dbg_counter = 0;
#define IMX_SAI_DBG_GET_INDEX (dbg_counter & (IMX_SAI_DBG_BUFFER_NR - 1))
static u8 dbg_buf[IMX_SAI_DBG_BUFFER_NR][4096];
static u32 dbg_crc[IMX_SAI_DBG_BUFFER_NR];
static lk_time_t dbg_ts[IMX_SAI_DBG_BUFFER_NR];
static size_t dbg_index = 0;
static u16 dbg_burst= 0;
static size_t dbg_len = 0;
static unsigned state = 0;
#define IMX_SAI_AUTODETECT_SEEK 0
#define IMX_SAI_AUTODETECT_PA 1
#define IMX_SAI_AUTODETECT_PB 2
#define IMX_SAI_AUTODETECT_PC 4
#define IMX_SAI_AUTODETECT_PD 8
#define IMX_SAI_AUTODETECT_PAYLOAD 16
#define IMX_SAI_AUTODETECT_DONE 32

static void imx_sai_autodetect(const void *buf, size_t len)
{
    size_t remaining = len;
    const u16 *ptr = buf;
    size_t xfer_len;
    unsigned index = IMX_SAI_DBG_GET_INDEX;

    while (remaining) {
        switch(state) {
        case IMX_SAI_AUTODETECT_SEEK:
            dbg_index = 0;
            if (*ptr == 0xF872) {
                state = IMX_SAI_AUTODETECT_PA;
                *((u16 *)&dbg_buf[index][dbg_index]) = *ptr; dbg_index += 2;
            }
            ptr++; remaining -= 2;
            break;
        case IMX_SAI_AUTODETECT_PA:
            if (*ptr == 0x4e1f) {
                state = IMX_SAI_AUTODETECT_PB;
                *((u16 *)&dbg_buf[index][dbg_index]) = *ptr; dbg_index += 2;
            } else {
                state = IMX_SAI_AUTODETECT_SEEK;
            }
            ptr++; remaining -= 2;
            break;
        case IMX_SAI_AUTODETECT_PB:
            dbg_burst = *ptr;
            *((u16 *)&dbg_buf[index][dbg_index]) = *ptr; dbg_index += 2;
            ptr++; remaining -= 2;
            state = IMX_SAI_AUTODETECT_PC;
            break;
        case IMX_SAI_AUTODETECT_PC:
            dbg_len = *ptr;
            dbg_len /= 8;
            *((u16 *)&dbg_buf[index][dbg_index]) = *ptr; dbg_index += 2;
            ptr++; remaining -= 2;
            state = IMX_SAI_AUTODETECT_PD;
            break;
        case IMX_SAI_AUTODETECT_PD:
        case IMX_SAI_AUTODETECT_PAYLOAD:
            xfer_len = MIN(remaining, dbg_len + 8 - dbg_index);
            memcpy(dbg_buf[index] + dbg_index, ptr, xfer_len);
            dbg_index += xfer_len;
            remaining -= xfer_len;
            ptr += (xfer_len/2);
            if (dbg_index >= (dbg_len + 8)) {
                remaining = 0;
                state = IMX_SAI_AUTODETECT_DONE;
            } else {
                state = IMX_SAI_AUTODETECT_PAYLOAD;
            }
            break;
        case IMX_SAI_AUTODETECT_DONE:
        default:
            dbg_len = 0;
            dbg_index = 0;
            dbg_burst = 0;
            state = IMX_SAI_AUTODETECT_SEEK;
//            memset(dbg_buf[index], 0, sizeof(dbg_buf));
            index = ++dbg_counter & 3;
        }
    }

    if (state == IMX_SAI_AUTODETECT_DONE) {
        dbg_ts[index] = current_time_hires();
        dbg_crc[index] = crc32(0, dbg_buf[index] + 8, dbg_len);
#ifdef IMX_SAI_AUTODETECT_PRINT
        u32 crc;
        crc = crc32(0, dbg_buf[index] + 8, dbg_len);
        printlk(LK_NOTICE, "%s:%d: New frame: crc32: %x\n",
                __PRETTY_FUNCTION__, __LINE__, crc);
//        hexdump(dbg_buf[index], dbg_len + 8);
#endif
    }
}

#if WITH_LIB_CONSOLE

#include <lib/console.h>

static size_t imx_sai_dbg_get_pkt_len(const void *buf)
{
    unsigned div;
    unsigned mul = 1;
    size_t len;
    const u16 *ptr = (const u16*) buf;

    if (*ptr++ != 0xF872)
        return 0;

    if (*ptr++ != 0x4e1f)
        return 0;

    switch(*ptr & 0x1F) {
    case 17:
    case 21:
    case 22:
        div = 1;
        break;
    case 23:
        switch ((*ptr & 0x60) >> 5) {
        case 0:
            div = 1;
            mul = 8;
            break;
        default:
            div = 8;
        }
        break;
    case 24:
        switch ((*ptr & 0x60) >> 5) {
        case 2:
            div = 1;
            mul = 8;
            break;
        default:
            div = 1;
        }
        break;
    case 25:
        switch ((*ptr & 0x60) >> 5) {
        case 0:
            div = 1;
            mul = 1;
            break;
        default:
            mul = 8;
            div = 1;
        }
        break;
    default:
        div = 8;
        mul = 1;
    }

    len = *++ptr;
    len *= mul;
    len /= div;

    return len;
}

static int cmd_sai_dbg(int argc, const cmd_args *argv);

STATIC_COMMAND_START
STATIC_COMMAND("sai", "SAI debug commands", &cmd_sai_dbg )
STATIC_COMMAND_END(sai);

static int cmd_sai_dbg(int argc, const cmd_args *argv)
{
    unsigned index;
    for (index = 0; index < IMX_SAI_DBG_BUFFER_NR; index++) {
        size_t len;
        u32 crc;
        len = imx_sai_dbg_get_pkt_len(dbg_buf[index]);
        if (len == 0)
            continue;
        crc = crc32(0, dbg_buf[index] + 8, len);

        printlk(LK_NOTICE, "%s:%d: Frame (%d - %u - %ld), crc32(new/old): %x/%x\n",
                        __PRETTY_FUNCTION__, __LINE__, index, dbg_ts[index], len, crc, dbg_crc[index]);
        hexdump(dbg_buf[index] + 8, len);
    }

    return 0;
}

#endif

#else
#define imx_sai_autodetect(x, y) {}
#endif

static status_t imx_sai_read(struct device *dev, void *buf, size_t len)
{
    size_t remaining, read, used;
    void *xfer;
    int ret = 0;

    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    sai_handle_t *handle = &state->sai_rx_handle;
    ASSERT(handle);

    struct imx_hw_state_s *hw_state = &state->rx_hw_state;

    mutex_acquire(&state->rx_mutex);

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->rx_lock, lock_state);
    used = cbuf_space_used(&state->rx_circ_buf);
    if (len > used)
        handle->xfer_remaining = len - used;
    else
        handle->xfer_remaining = 0;

retry:
    imx_sai_rx_start_hw(state);
    smp_wmb();
    spin_unlock_irqrestore(&state->rx_lock, lock_state);
    remaining = len;
    xfer = buf;

    printlk(LK_VERBOSE, "len: %ld\n", len);
    if (!imx_is_hw_state(hw_state, kSAI_hw_Started)) {
#if ((defined IMX_SAI_READ_AUTOSTART) && (IMX_SAI_READ_AUTOSTART == 1))
        imx_sai_rx_start(dev);
#else
        /* read operation when driver is stopped - abort */
        printlk(LK_ERR, "%s:%d: SAI%d: port stopped - abort\n", __PRETTY_FUNCTION__, __LINE__, state->bus_id);
        ret = ERR_BAD_STATE;
        goto done;
#endif
    }

    while (remaining) {
        read = cbuf_read(&state->rx_circ_buf, xfer, remaining, false);
        remaining -= read;
        xfer += read;

        if (remaining)
            event_wait(&state->rx_wait);

        if (handle->xfer_remaining < 0) {
            printlk(LK_NOTICE, "%s:%d: SAI%d: Abort (%ld)\n",
                    __PRETTY_FUNCTION__, __LINE__, state->bus_id,
                    handle->xfer_remaining);
            break;
        }
    }

    ret = handle->xfer_remaining;
    spin_lock_irqsave(&state->rx_lock, lock_state);

    handle->xfer_remaining = 0;
    smp_wmb();

    event_unsignal(&state->rx_wait);

    if (ret) {
        printlk(LK_ERR, "%s:%d: SAI%d: cbuf reset (including indexes) on error %d\n",
                __PRETTY_FUNCTION__, __LINE__, state->bus_id, ret);
        cbuf_reset_indexes(&state->rx_circ_buf);
        imx_clear_hw_state(hw_state, kSAI_hw_Flushing);

        if (handle->dma_enable) {
            printlk(LK_DEBUG, "sai%d: Reconfigure RX DMA channel...\n", state->bus_id);

            /* Disable zero-copy callback to avoid further cache ops */
            state->sai_rx_handle.dma_chan->desc->cb = NULL;
            state->sai_rx_handle.dma_chan->desc->zerocopy_cb = NULL;
        }

        spin_unlock_irqrestore(&state->rx_lock, lock_state);

        /* Stop SAI request */
        SAI_RxEnable(state->io_base, false);

        SAI_RxSoftwareReset(state->io_base, kSAI_ResetTypeFIFO);

        if (handle->dma_enable) {
            /* Terminate DMA channel and reallocate a new one */
            imx_sai_dma_setup(dev, true);
            dma_resume_channel(state->sai_rx_handle.dma_chan);
        }

        /* Start SAI request */
        SAI_RxEnable(state->io_base, true);

        spin_lock_irqsave(&state->rx_lock, lock_state);

        if (ret == ERR_IO)
            imx_sai_rx_start_hw(state);
    }

#if ((defined IMX_SAI_READ_SILENT_RETRY) && (IMX_SAI_READ_SILENT_RETRY == 1))
    if (ret < 0) {
        printlk(LK_ERR, "%s:%d: Error (%d) while reading, retrying...\n",
                __PRETTY_FUNCTION__, __LINE__, ret);
        handle->xfer_remaining = len;
        goto retry;
    }
#endif
    spin_unlock_irqrestore(&state->rx_lock, lock_state);

done:
    printlk(LK_VERBOSE, "len: %ld..done(%d)\n", len, ret);
    mutex_release(&state->rx_mutex);

    imx_sai_autodetect(buf, len);

    return ret;
}

static struct device_class sai_device_class = {
    .name = "sai",
};


static status_t imx_sai_ioctl(struct device *dev, int request, void *argp)
{
    status_t ret;
    struct imx_sai_state *state = dev->state;
    struct sai_ioc_cmd_pll_k *pll_k_param;

    if (!dev)
        return ERR_NOT_FOUND;

    if ((IOCGROUP(request) != IOCTL_DEV_SAI) || (!argp))
        return ERR_INVALID_ARGS;

    int cmd = IOCBASECMD(request);

    switch(request) {
    case SAI_IOC_CNT_EN:
        ret =  imx_sai_en_counters(dev, *((int *) argp));
        break;

    case SAI_IOC_CNT_GET:
        ret = imx_sai_get_counters(dev, (struct sai_ioc_cmd_cnt *) argp);
        break;

    case SAI_IOC_CLK_MODE:
        ret = imx_sai_set_clk_mode(dev, *(int *) argp);
        break;

    case SAI_IOC_TS_GET:
        ret = imx_sai_get_timestamp(dev, (struct sai_ioc_cmd_timestamp *) argp);
        break;

    case SAI_IOC_TS_SET:
        ret = imx_sai_set_timestamp(dev, (struct sai_ioc_cmd_timestamp *) argp);
        break;

    case SAI_IOC_LATENCY_GET:
        ret = imx_sai_get_latency(dev, (struct sai_ioc_cmd_latency *) argp);
        break;

    case SAI_IOC_BUF_SZ_GET:
        ret = imx_sai_get_buffer_size(dev, (struct sai_ioc_cmd_buffer_sz *) argp);
        break;

    case SAI_IOC_PLL_K_SET:
        pll_k_param = (struct sai_ioc_cmd_pll_k*) argp;
        imx_sai_set_mclk(state, pll_k_param->sampling_rate, true,
                         &pll_k_param->pll_k_value);
        ret = 0;
        break;

    case SAI_IOC_RCE_SET:
        ret = imx_sai_set_rce(dev, (uint8_t *) argp);
        break;

    case SAI_IOC_SZ_USED_GET: {
        struct sai_ioc_cmd_space_used *sai_space_used = argp;
        cbuf_t *cbuf;
        sai_transfer_format_t *format;

        if (sai_space_used->is_read == true) {
            cbuf = &state->rx_circ_buf;
            format = &state->sai_rx_fmt;
        } else {
            cbuf = &state->tx_circ_buf;
            format = &state->sai_tx_fmt;
        }

        sai_space_used->used = cbuf_space_used(cbuf) /
            (format->audio_channel * format->bitWidth / 8);
        ret = 0;
        break;
    }

    default:
        printlk(LK_ERR, "%s:%d: Invalid sai ioctl(%d)\n",
                __PRETTY_FUNCTION__, __LINE__, cmd);
        ret = ERR_NOT_FOUND;
    }

    return ret;
}

static status_t imx_sai_tx_set_callback(struct device *dev,
                                                    sai_cb_t cb, void *cookie)
{
    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->tx_cb_lock, lock_state);

    state->tx_cb = cb;
    state->tx_cb_cookie = cookie;

    spin_unlock_irqrestore(&state->tx_cb_lock, lock_state);

    return 0;
}

static status_t imx_sai_rx_set_callback(struct device *dev,
                                                    sai_cb_t cb, void *cookie)
{
    struct imx_sai_state *state = dev->state;
    ASSERT(state);

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->rx_cb_lock, lock_state);

    state->rx_cb = cb;
    state->rx_cb_cookie = cookie;

    spin_unlock_irqrestore(&state->rx_cb_lock, lock_state);

    return 0;
}

static struct sai_ops the_ops = {
    .std = {
        .device_class = &sai_device_class,
        .init = imx_sai_init,
        .ioctl = imx_sai_ioctl,
    },
    .tx_open = imx_sai_tx_open,
    .tx_close = imx_sai_tx_close,
    .tx_start = imx_sai_tx_start,
    .tx_stop = imx_sai_tx_stop,
    .tx_flush = imx_sai_tx_flush,
    .tx_drop = imx_sai_tx_drop,
    .tx_setup = imx_sai_tx_setup,
    .tx_en_cnt = imx_sai_tx_en_counters,
    .tx_get_cnt = imx_sai_tx_get_counters,
    .tx_set_callback = imx_sai_tx_set_callback,
    .write = imx_sai_write,
    .rx_open = imx_sai_rx_open,
    .rx_close = imx_sai_rx_close,
    .rx_start = imx_sai_rx_start,
    .rx_stop = imx_sai_rx_stop,
    .rx_flush = imx_sai_rx_flush,
    .rx_setup = imx_sai_rx_setup,
    .rx_en_cnt = imx_sai_rx_en_counters,
    .rx_get_cnt = imx_sai_rx_get_counters,
    .rx_set_callback = imx_sai_rx_set_callback,
    .read = imx_sai_read,
};

DRIVER_EXPORT_WITH_LVL(sai, &the_ops.std, DRIVER_INIT_PLATFORM);

struct device *class_sai_get_device_by_id(int id)
{
    struct device *dev = NULL;
    struct imx_sai_state *state = NULL;

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&imx_sai_list_lock, lock_state);

    list_for_every_entry(&imx_sai_list, state, struct imx_sai_state, node) {
        if (state->bus_id == id) {
            dev = state->device;
            break;
        }
    }

    spin_unlock_irqrestore(&imx_sai_list_lock, lock_state);

    return dev;
}
