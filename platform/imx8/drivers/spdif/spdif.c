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
#include <sys/types.h>
#include <stdlib.h>
#include <dev/class/spdif.h>
#include <platform/spdif.h>
#include <platform/spdif_cs.h>
#include "spdif_hw.h"
#include "platform.h"

#include <kernel/spinlock.h>
#include <kernel/semaphore.h>
#include <kernel/event.h>
#include <kernel/vm.h>

#include <lib/appargs.h>
#include <dev/interrupt.h>
#include <lib/cbuf.h>
#include <lib/dpc.h>
#include <assert.h>
#include <err.h>
#include <debug.h>

#include <dev/dma.h>

#include <sys/ioctl.h>
#include <dev/spdif_ioctl.h>

#include <platform/imx_common.h>

#ifndef IMX_SPDIF_MEMALLOC_ON_INIT
#define IMX_SPDIF_MEMALLOC_ON_INIT 1
#endif

#ifndef IMX_SPDIF_READ_AUTOSTART
#define IMX_SPDIF_READ_AUTOSTART 1
#endif

/* TODO: Move this to a common include */
#define PINMUX_ENABLE 1
#define POWER_ENABLE 1

#undef IMX_SPDIF_PARANOID

#define IMX_SPDIF_ERROR_RATE_LIMIT_MS 50

/* Channel Status definitions */
#define IMX_SPDIF_IEC60958_FRAME_SIZE 192
#define IMX_SPDIF_IEC60958_SUBFRM_CS_BIT_MASK BIT(30)

#define IMX_SPDIF_RAW_PREAMBLE_MASK 0xF
#define IMX_SPDIF_RAW_PREAMBLE_X    0x4
#define IMX_SPDIF_RAW_PREAMBLE_Y    0x8
#define IMX_SPDIF_RAW_PREAMBLE_Z    0xC


/* Currently hardcoded in the HW macro */
#define IMX_SPDIF_FIFO_STAGES 16

/* Number of frames to discard after a DPLL locked event */
#define IMX_SPDIF_RX_FRAMES_WARMUP 1024

#define IMX_SPDIF_DMA_PERIOD    (1024)
#define IMX_SPDIF_DMA_NR_PERIOD (4)
#define IMX_SPDIF_MAX_DMA_NR_PERIOD (16)
#define DMA_BUFFER_ALIGNMENT    (128)

#define IMX_SPDIF_IRQ_RX_ERRORS \
    kSPDIF_ValidityFlagNoGood | kSPDIF_RxIllegalSymbol | \
    kSPDIF_RxParityBitError | kSPDIF_RxFIFOError | \
    kSPDIF_RxFIFOResync

#define BIT(x) (1ULL << x)


enum spdif_hw_state
{
    kSPDIF_Down         = (1UL << 0),   /*!< SPDIF is powered down. */
    kSPDIF_Up           = (1UL << 1),   /*!< SPDIF is powered up. */
    kSPDIF_Prepared     = (1UL << 2),   /*!< SPDIF is prepared  */
    kSPDIF_Ready        = (1UL << 3),   /*!< SPDIF is prepared and ready. */
    kSPDIF_On           = (1UL << 4),   /*!< SPDIF is on. IRQ flowing */
    kSPDIF_Busy         = (1UL << 5),   /*!< SPDIF is busy, transfer on going */
    kSPDIF_Done         = (1UL << 6),   /*!< Transfer is done. */
    kSPDIF_Error        = (1UL << 7),   /*!< Transfer error occurred. */
    kSPDIF_Locked       = (1UL << 8),   /*!< DPLL Locked. */
    kSPDIF_Warmup       = (1UL << 9),   /*!< Warm up completed */
    kSPDIF_Flushing     = (1UL << 10),   /*!< Flushing on going */
    kSPDIF_Running      = kSPDIF_On | kSPDIF_Locked | kSPDIF_Warmup,
};

struct imx_spdif_state {
    int bus_id;
    SPDIF_Type *io_base;
    struct device *device;
    struct list_node node;
    spin_lock_t cb_lock;
    spdif_cb_t cb;
    void * cb_cookie;
#define SPDIF_MAX_RX_BUF_SIZE (1024 * 1024)
    cbuf_t rx_circ_buf;
    size_t rx_circ_buf_size;
    spin_lock_t rx_lock;
    mutex_t rx_mutex;
    event_t rx_wait;
    int rx_frames_warmup;
    unsigned clk_rate;
    unsigned clk_fs_rate;
    struct imx_hw_state_s hw_state;
    /* Interface with SPDIF HW */
    spdif_handle_t rx_handle;
    spdif_config_t hw_config;
    spdif_hw_params_t hw_params;
    bool dma_enable;
    semaphore_t dma_period_sem;
    thread_t *dma_thread;
    struct dma_descriptor *dma_desc;
    uint64_t dma_period_elapsed;
    void *dma_buffer;
    paddr_t dma_buffer_phys;
    uint32_t dma_mode;
    unsigned dma_nr_periods;
    unsigned dma_period_size;
    /* Channel Status */
    #define IMX_SPDIF_CS_DATA_SIZE (IMX_SPDIF_IEC60958_FRAME_SIZE / 8)
    #define IMX_SPDIF_CS_DATA_BUFFERS 2
    bool cs_locked;
    size_t cs_bit_count;
    size_t cs_current;
    uint8_t cs_buff[IMX_SPDIF_CS_DATA_BUFFERS][IMX_SPDIF_CS_DATA_SIZE];
    bool use_spdif_cs;
    struct device *spdif_cs_device;
    bool monitor;
    uint64_t time_error_event;
};

static inline void imx_spdif_set_hw_state(struct imx_spdif_state *handle,
                                                                uint32_t state)
{
    imx_set_hw_state(&handle->hw_state, state);
}

static inline void imx_spdif_clear_hw_state(struct imx_spdif_state *handle, uint32_t state)
{
    imx_clear_hw_state(&handle->hw_state, state);
}

static inline bool imx_spdif_is_hw_state(struct imx_spdif_state *handle,
                                            enum spdif_hw_state expected_state)
{
    return imx_is_hw_state(&handle->hw_state, expected_state);
}

static inline status_t imx_spdif_check_state(
                                    struct imx_spdif_state *handle,
                                    uint32_t expected_state,
                                    const char *msg)
{

    return imx_check_hw_state(&handle->hw_state, expected_state, msg);
}

static inline status_t imx_spdif_check_strict_state(
                                    struct imx_spdif_state *handle,
                                    uint32_t expected_state,
                                    const char *msg)
{

    return imx_check_strict_hw_state(&handle->hw_state, expected_state, msg);
}

static inline uint32_t imx_spdif_get_hw_state(struct imx_spdif_state *state)
{
    return imx_get_hw_state(&state->hw_state);
}

void SPDIF_RxStart(SPDIF_Type *base, uint32_t error_mask, bool dma_enable)
{
    uint32_t mask = error_mask;

    SPDIF_RxFIFOReset(base);

    /* Remove previous pending status flags */
    if (error_mask)
        SPDIF_ClearStatusFlags(base, error_mask);

    /* Enable Rx DMA hardware requests */
    if (dma_enable)
        SPDIF_EnableDMA(base, kSPDIF_RxDMAEnable, true);
    else
        mask |= kSPDIF_RxFIFOFull;

    /* Enable Interrupts */
    SPDIF_EnableInterrupts(base, mask);

    /* Enable Rx transfer */
    smp_wmb();
    SPDIF_RxEnable(base, true);
}

void SPDIF_RxStop(SPDIF_Type *base, uint32_t error_mask, bool dma_enable)
{
    uint32_t mask = error_mask;

    /* Disable Rx transfer */
    SPDIF_RxEnable(base, false);
    smp_wmb();

    /* Disable Rx DMA hardware requests */
    if (dma_enable)
        SPDIF_EnableDMA(base, kSPDIF_RxDMAEnable, false);
    else
        mask |= kSPDIF_RxFIFOFull;
    SPDIF_DisableInterrupts(base, mask);
    if (error_mask)
        SPDIF_ClearStatusFlags(base, error_mask);
}

static struct list_node imx_spdif_list = LIST_INITIAL_VALUE(imx_spdif_list);
static spin_lock_t imx_spdif_list_lock = SPIN_LOCK_INITIAL_VALUE;

/* CS (Channel Status ) functions to be called with rx_lock held */
static void *imx_spdif_cs_get_current_buff(struct imx_spdif_state *state)
{
    return (void *)state->cs_buff[state->cs_current];
}

static void *imx_spdif_cs_get_next_buff(struct imx_spdif_state *state)
{
    size_t next = (state->cs_current + 1) % IMX_SPDIF_CS_DATA_BUFFERS;
    return (void *)state->cs_buff[next];
}

static void imx_spdif_cs_flip_buff(struct imx_spdif_state *state)
{
    size_t next = (state->cs_current + 1) % IMX_SPDIF_CS_DATA_BUFFERS;
    state->cs_current = next;
}

static void imx_spdif_cs_clear_buffs(struct imx_spdif_state *state)
{
    uint8_t *buff;

    buff = imx_spdif_cs_get_current_buff(state);
    memset(buff, 0, IMX_SPDIF_CS_DATA_SIZE);
    buff = imx_spdif_cs_get_next_buff(state);
    memset(buff, 0, IMX_SPDIF_CS_DATA_SIZE);
}

static void imx_spdif_cs_set_sync(struct imx_spdif_state *state, bool locked)
{
    state->cs_locked = locked;
    state->cs_bit_count = 0;
    if (!locked)
        /* clear CS data to force update on relock */
        imx_spdif_cs_clear_buffs(state);
}

static void imx_spdif_cs_callback(void *cookie, enum spdif_cs_event evt,
                               void *data, size_t data_size)
{
    struct imx_spdif_state *state;
    state = (struct imx_spdif_state *)cookie;
    spin_lock_saved_state_t lock_state;
    uint8_t *next;

    /* copy last consistent set of CS data under lock */
    spin_lock_irqsave(&state->rx_lock, lock_state);
    next = imx_spdif_cs_get_next_buff(state);
    memcpy(next, data, IMX_SPDIF_CS_DATA_SIZE);
    spin_unlock_irqrestore(&state->rx_lock, lock_state);
}

static void imx_spdif_cs_init(struct imx_spdif_state *state)
{
    state->cs_current = 0;
    imx_spdif_cs_set_sync(state, false);
    state->spdif_cs_device = of_device_lookup_device(state->device, "spdif-cs");
    state->use_spdif_cs = !!(state->spdif_cs_device);
}

void imx_spdif_cs_parse(struct imx_spdif_state *state,
                               void *buf, size_t len, bool *update)
{
    size_t sframes;
    uint32_t *psframe = (uint32_t *)buf;
    uint32_t sframe;
    uint8_t *current, *next;
    size_t offset;
    unsigned bitpos;
    bool z, cs;

    /* Even number of subframes: (L,R) frames */
    DEBUG_ASSERT((len % (sizeof(uint32_t) * 2)) == 0);

    sframes = len / sizeof(uint32_t);
    next = imx_spdif_cs_get_next_buff(state);
    current = imx_spdif_cs_get_current_buff(state);
    *update = false;

    /* spdif_cs driver updates asynchronously next buffer */
    if (state->use_spdif_cs) {
        if (memcmp(current, next, IMX_SPDIF_CS_DATA_SIZE)) {
            memcpy(current, next, IMX_SPDIF_CS_DATA_SIZE);
            imx_spdif_cs_flip_buff(state);
            *update = true;
        }

        return;
    }

    /* Go through every L subframe - R subframe ignored */
    while (sframes) {
         /* Parse Z and CS bits within subframe */
         sframe = *psframe;
         z = ((sframe & IMX_SPDIF_RAW_PREAMBLE_MASK)
                         == IMX_SPDIF_RAW_PREAMBLE_Z);
         cs = sframe & IMX_SPDIF_IEC60958_SUBFRM_CS_BIT_MASK;

        /*
         * Lock to IEC60958 multiframe 192 lock first
         * Confirm lock at every multiframe 192 start
         */
        if (!state->cs_locked) {
            if (z) {
                printlk(LK_VERBOSE, "mf192 locked\n");
                imx_spdif_cs_set_sync(state, true);
            }
        } else if (state->cs_bit_count == 0) {
            if (!z) {
                printlk(LK_VERBOSE, "mf192 lock lost\n");
                imx_spdif_cs_set_sync(state, false);
            }
        } else {
            if (z) {
                printlk(LK_VERBOSE, "mf192 sync error\n");
                imx_spdif_cs_set_sync(state, false);
            }
        }

        /* No lock - continue to next frame */
        if (!state->cs_locked)
            goto next;

        /* Accumulate CS bits in next buffer */
        bitpos = state->cs_bit_count % 8;
        offset = state->cs_bit_count / 8;

        if (cs)
            next[offset] |=  BIT(bitpos);
        else
            next[offset] &= ~BIT(bitpos);

        state->cs_bit_count = (state->cs_bit_count + 1) % IMX_SPDIF_IEC60958_FRAME_SIZE;

        /* End of multiframe 192 - check if new CS data */
        if (state->cs_bit_count == 0) {
            if (memcmp(current, next, IMX_SPDIF_CS_DATA_SIZE)) {
                /* flip current / next - notify */
                printlk(LK_VERBOSE, "cs data update\n");
                imx_spdif_cs_flip_buff(state);
                next = imx_spdif_cs_get_next_buff(state);
                current = imx_spdif_cs_get_current_buff(state);
                *update = true;
            }
        }

next:
        sframes -= 2;
        psframe += 2;
    }
}


static void imx_spdif_clk_rate_change_cb(void *arg)
{
    struct imx_spdif_state *state = arg;
    unsigned rate = state->clk_fs_rate;

    state->cb(SPDIF_EVENT_AUDIO_SAMPLE_RATE, &rate, state->cb_cookie);

    if (state->use_spdif_cs)
        spdif_cs_start(state->spdif_cs_device, state->clk_fs_rate);

    return;
}

static void imx_spdif_clk_lost_cb(void *arg)
{
    struct imx_spdif_state *state = arg;

    state->cb(SPDIF_EVENT_AUDIO_CLK_LOST, NULL, state->cb_cookie);

    if (state->use_spdif_cs)
        spdif_cs_stop(state->spdif_cs_device);

    return;
}

static void imx_spdif_cs_change_cb(void *arg)
{
    struct imx_spdif_state *state = arg;
    spin_lock_saved_state_t lock_state;
    uint8_t cs_data[IMX_SPDIF_CS_DATA_SIZE], *current;

    /* copy last consistent valid set of CS data under lock */
    spin_lock_irqsave(&state->rx_lock, lock_state);
    current = imx_spdif_cs_get_current_buff(state);
    memcpy(cs_data, current, IMX_SPDIF_CS_DATA_SIZE);
    spin_unlock_irqrestore(&state->rx_lock, lock_state);

    state->cb(SPDIF_EVENT_AUDIO_CHANNEL_STATUS, cs_data, state->cb_cookie);

    return;
}

static void imx_spdif_error_cb(void *arg)
{
    struct imx_spdif_state *state = arg;

    state->cb(SPDIF_EVENT_ERROR, NULL, state->cb_cookie);

    return;
}

static struct imx_spdif_isr_rx_flag {
    uint32_t bit;
    const char *msg;
    bool is_error;
} imx_spdif_isr_rx_flags[] = {
    {
        .bit = kSPDIF_RxIllegalSymbol,
        .msg = "Illegal receiver symbol error",
        .is_error = true
    },
    {
        .bit = kSPDIF_ValidityFlagNoGood,
        .msg = "Invalid flag set error",
        .is_error = true
    },
    {
        .bit = kSPDIF_RxParityBitError,
        .msg = "Parity error",
        .is_error = true
    },
    {
        .bit = kSPDIF_RxFIFOError,
        .msg = "Rx FIFO underrun/overrun error",
        .is_error = true
    },
    {
        .bit = kSPDIF_RxFIFOResync,
        .msg = "Rx FIFO resync",
        .is_error = true
    },
    {
        .bit = kSPDIF_RxDPLLLocked,
        .msg = "DPLL locked",
        .is_error = false
    },
    {
        .bit = kSPDIF_LockLoss,
        .msg = "DPLL lock loss",
        .is_error = true
    },
};

static bool imx_spdif_push_to_rx_circ_buffer(struct imx_spdif_state *state,
                                            void *buf, size_t xfer_len)
{
    spdif_handle_t *handle = &state->rx_handle;
    bool reschedule = false;
    size_t written = 0;

    /* Push the data to circular buffer */
    printlk(LK_VERBOSE, "hw state: (%x), Pushing %ld to rx circular buffer\n",
            imx_spdif_get_hw_state(state), xfer_len);

    size_t avail = cbuf_space_avail(&state->rx_circ_buf);

    if (xfer_len > avail)
        printlk(LK_NOTICE, "%s:%d: No more room in spdif circ buffer (avail %ld). Dropping %ld bytes\n",
                __PRETTY_FUNCTION__, __LINE__, avail, xfer_len);
    else {
        written = cbuf_write(&state->rx_circ_buf, buf, xfer_len, false);
        DEBUG_ASSERT(written == xfer_len);
    }

    /* Wake up thread if transfer on going */
    if (imx_spdif_is_hw_state(state, kSPDIF_Busy)
            && (handle->xfer_remaining > 0)) {
        handle->xfer_remaining -= written;
        if ((handle->xfer_remaining <= 0) ||
            (cbuf_space_used(&state->rx_circ_buf) > handle->cbuf_watermark)) {
            handle->xfer_remaining = MAX(0, handle->xfer_remaining);
            event_signal(&state->rx_wait, false);
            reschedule = true;
        }
        imx_spdif_set_hw_state(state, kSPDIF_Done);
    }
    return reschedule;
}

static bool imx_spdif_report_error(struct imx_spdif_state *state)
{
    uint64_t now, diff, last;
    bool ret = false;

    now =  current_time_hires();
    last = state->time_error_event;
    if (now >= last)
        diff = (now - last);
    else
        diff = (UINT64_MAX - last + 1 + now);

    if (diff > (IMX_SPDIF_ERROR_RATE_LIMIT_MS * 1000)) {
        state->time_error_event = now;
        dpc_queue(imx_spdif_error_cb, state, DPC_FLAG_NORESCHED);
        ret = true;
    }

    return ret;
}

static bool imx_spdif_isr_rx(struct imx_spdif_state *state)
{
    bool reschedule = false;
    bool cs_update = false;
    uint32_t sis, err_pending, sie, sise, i;

    SPDIF_Type *base = state->io_base;
    ASSERT(base);

    spdif_handle_t *handle = &state->rx_handle;

    spin_lock(&state->rx_lock);
    sis = SPDIF_GetStatusFlag(base);
    sie = base->SIE;
    bool dll_locked, was_locked;
    sise = sis & sie;
    err_pending = 0;

    printlk(LK_DEBUG, "hw state: (%x), sis: %#x, sie: %#x, sise: %#x\n",
                                imx_spdif_get_hw_state(state), sis, sie, sise);
    /* DPLL */
    dll_locked = !!(base->SRPC & SPDIF_SRPC_LOCK_MASK);
    if (sise & kSPDIF_RxDPLLLocked) {
        SPDIF_ClearStatusFlags(base, kSPDIF_RxDPLLLocked | handle->irq_mask);
        printlk(LK_NOTICE, "PLL locked (%x/%x/%x)\n", sis, sise, dll_locked);
    }

    if (sise & kSPDIF_LockLoss) {
        SPDIF_ClearStatusFlags(base, kSPDIF_LockLoss | handle->irq_mask);
        printlk(LK_NOTICE, "PLL unlocked (%x/%x/%x)\n", sis, sise, dll_locked);
    }

    dll_locked = !!(base->SRPC & SPDIF_SRPC_LOCK_MASK);
    was_locked = imx_spdif_is_hw_state(state, kSPDIF_Locked);

    if (!was_locked && dll_locked) {
        state->rx_frames_warmup = IMX_SPDIF_RX_FRAMES_WARMUP;
        /* Need to warm up now ...*/
        if (state->dma_enable == false)
            SPDIF_RxStart(base, 0, state->dma_enable);
        imx_spdif_set_hw_state(state, kSPDIF_Locked);
        printlk(LK_NOTICE, "hw state: (%x), DPLL Locked, Warming up\n",
                                            imx_spdif_get_hw_state(state));
    }

    if (was_locked && !dll_locked) {
        printlk(LK_NOTICE, "hw state: (%x), DPLL Lock loss\n",
                                            imx_spdif_get_hw_state(state));
        if (state->dma_enable)
            SPDIF_DisableInterrupts(base, handle->irq_mask);
        else
            SPDIF_RxStop(base, kSPDIF_LockLoss | handle->irq_mask, state->dma_enable);
        imx_spdif_clear_hw_state(state, kSPDIF_Locked | kSPDIF_Warmup);
        if (state->cb) {
            dpc_queue(imx_spdif_clk_lost_cb, state, DPC_FLAG_NORESCHED);
            imx_spdif_cs_set_sync(state, false);
            dpc_queue(imx_spdif_cs_change_cb, state, DPC_FLAG_NORESCHED);
        }
        reschedule = true;
    }

    /* Error handling: We care only about the enabled interrupt */
    for (i = 0; i < ARRAY_SIZE(imx_spdif_isr_rx_flags); i++) {
            struct imx_spdif_isr_rx_flag *isr_rx = &imx_spdif_isr_rx_flags[i];
            if (sise & isr_rx->bit) {
                printlk(LK_NOTICE, "%s:%d: %s\n", __PRETTY_FUNCTION__, __LINE__,
                        isr_rx->msg);
                if (isr_rx->is_error)
                    err_pending |= isr_rx->bit;
            }
    }

    if (err_pending) {
        SPDIF_ClearStatusFlags(base, err_pending);
        if (imx_spdif_is_hw_state(state, kSPDIF_Busy)) {
            /* FIXME/TODO: Reset the Hw RX FIFO ? */
            printlk(LK_NOTICE, "%s:%d: Aborting transfer, hw error(s): %x, remains %ld bytes\n",
                    __PRETTY_FUNCTION__, __LINE__, err_pending,
                    handle->xfer_remaining);
            handle->xfer_remaining = ERR_IO;
            imx_spdif_set_hw_state(state, kSPDIF_Error);
            reschedule = true;
            event_signal(&state->rx_wait, false);
            smp_wmb();
            goto exit_rx_isr;
        }
    }

    /*
     * Data Handling: Only 32b raw mode supported so far.
     */
    if (sise & kSPDIF_RxFIFOFull) {
        unsigned i;
        size_t xfer_len = handle->watermark * 2 * 4;
        uint32_t *buffer = handle->buf;
        uint32_t chl, chr;

        printlk(LK_DEBUG, "hw state: (%x), dll (%d), watermark(%d): Rx FIFO Full \n",
                                 imx_spdif_get_hw_state(state), dll_locked, handle->watermark);
        /* Read the hw fifo and fill in intermediate buffer */
        for ( i = 0; i < handle->watermark; i++) {
            chl = SPDIF_ReadLeftData(base);
            chr = SPDIF_ReadRightData(base);
            /* Discard if the DLL is not locked */
            if (dll_locked) {
                /* Skip the first N frames after a DPLL lock event */
                if (state->rx_frames_warmup) {
                    printlk(LK_DEBUG, "hw state: (%x), Skipping frame (%d)\n",
                        imx_spdif_get_hw_state(state), state->rx_frames_warmup);
                    state->rx_frames_warmup--;
                    xfer_len -= 8;
                    if (state->rx_frames_warmup == 0) {
                        imx_spdif_set_hw_state(state, kSPDIF_Warmup);
                        printlk(LK_NOTICE, "hw state: (%x), Warm up completed)\n",
                            imx_spdif_get_hw_state(state));
                        state->clk_fs_rate = SPDIF_GetRxSampleRate(base, state->clk_rate);
                        printlk(LK_NOTICE, "fs rate: %d\n", state->clk_fs_rate);
                        if (state->cb)
                            dpc_queue(imx_spdif_clk_rate_change_cb, state, DPC_FLAG_NORESCHED);
                        reschedule = true;
                        /* Avoid spurious interrupt of past interrupts */
                        if (!(imx_spdif_is_hw_state(state, kSPDIF_On))) {
                            printlk(LK_NOTICE, "hw state: (%x), Disabling Rx\n",
                                imx_spdif_get_hw_state(state));
                            SPDIF_RxStop(base, handle->irq_mask, state->dma_enable);
                            dma_pause_channel(state->rx_handle.dma_chan);
                        } else {
                            dma_resume_channel(state->rx_handle.dma_chan);
                            SPDIF_RxStart(base, handle->irq_mask, state->dma_enable);
                        }
                    }
                } else {
                    *buffer++ = chl;
                    *buffer++ = chr;
                }
            }
        }

        if (dll_locked && xfer_len && (imx_spdif_is_hw_state(state, kSPDIF_Running))) {
            if (imx_spdif_is_hw_state(state, kSPDIF_Flushing)) {
                if (handle->xfer_remaining) {
                    handle->xfer_remaining = ERR_CANCELLED;
                    reschedule = true;
                    event_signal(&state->rx_wait, false);
                }
            } else {
                if (!state->monitor)
                    reschedule |= imx_spdif_push_to_rx_circ_buffer(state, handle->buf,
                                                               xfer_len);
            }
            imx_spdif_cs_parse(state, handle->buf, xfer_len, &cs_update);
        }
    }

exit_rx_isr:
    smp_wmb();
    spin_unlock(&state->rx_lock);

    if (state->cb && cs_update) {
        dpc_queue(imx_spdif_cs_change_cb, state, DPC_FLAG_NORESCHED);
        reschedule = true;
    }

    if (state->cb && err_pending) {
        if (imx_spdif_report_error(state))
            reschedule = true;
    }

    return reschedule;
}

static int imx_spdif_dma_worker_thread(void *arg)
{
    struct imx_spdif_state *state = arg;
    bool rate_update;
    bool cs_update;

    SPDIF_Type *base = state->io_base;
    ASSERT(base);

    struct dma_buf_vec_s {
        unsigned valid;
        unsigned length;
        void *vaddr[state->dma_nr_periods];
    } dma_buf_vec;

    while(true) {
        /* A reschedule will be trigger if semaphore is not ready */
        sem_wait(&state->dma_period_sem);
        smp_mb();
        struct dma_descriptor *desc = state->dma_desc;
        printlk(LK_DEBUG, "Period elapsed (DMA/Worker): [%llu:%llu]\n",
            desc->period_elapsed, state->dma_period_elapsed);
        uint64_t period_elapsed = desc->period_elapsed - state->dma_period_elapsed;

#ifdef ENABLE_PERMISSIVE_MODE
        if (period_elapsed > state->dma_nr_periods) {
            printlk(LK_WARNING, "WARNING PERMISSIVE: period elapsed not expected (%lld vs. %d)\n",
                    __PRETTY_FUNCTION__, __LINE__, period_elapsed, state->dma_nr_periods);

            state->dma_period_elapsed +=
                ((period_elapsed/state->dma_nr_periods - 1) * state->dma_nr_periods +
                 1 + period_elapsed % state->dma_nr_periods);

            period_elapsed = state->dma_nr_periods - 1;
	    }
#else
        if (period_elapsed > state->dma_nr_periods) {
            panic("Mismatch on hw/sw period elapsed: %lld/%lld/%x\n" ,
                    desc->period_elapsed,
                    state->dma_period_elapsed,
                    imx_spdif_get_hw_state(state));
        }
#endif

        size_t payload_sz = period_elapsed * state->dma_period_size;
        spdif_handle_t *handle = &state->rx_handle;
        spin_lock_saved_state_t lock_state;
        rate_update = false;
        cs_update = false;
        dma_buf_vec.valid = 0;
        dma_buf_vec.length = state->dma_period_size;
        spin_lock_irqsave(&state->rx_lock, lock_state);
        if (imx_spdif_is_hw_state(state, kSPDIF_Locked)) {
            if (imx_spdif_is_hw_state(state, kSPDIF_Warmup)) {
                unsigned i, index;
                size_t len = state->dma_period_size;
                for ( i = 0; i < period_elapsed; i++) {
                    index = state->dma_period_elapsed % state->dma_nr_periods;
                    void * buf = state->dma_buffer;
                    buf += len * index;
                    /* Are we flushing ? */
                    if (imx_spdif_is_hw_state(state, kSPDIF_Flushing)) {
                        printlk(LK_NOTICE,
                            "Flushing request detected (DMA mode)(%x)\n",
                                imx_spdif_get_hw_state(state));
                        if (!imx_spdif_is_hw_state(state, kSPDIF_Busy)) {
                            printlk(LK_NOTICE,
                                "Flushing request without pending xfer (%x))\n",
                                    imx_spdif_get_hw_state(state));
                            imx_spdif_clear_hw_state(state, kSPDIF_Flushing);
                        } else {
                            if (handle->xfer_remaining > 0) {
                                handle->xfer_remaining = ERR_CANCELLED;
                                event_signal(&state->rx_wait, false);
                                printlk(LK_NOTICE,
                                    "Reader thread awaken on flush request (%x)\n",
                                        imx_spdif_get_hw_state(state));
                            } else {
                                printlk(LK_NOTICE,
                                    "Flushing active while no xfer remaining (%x)\n",
                                        imx_spdif_get_hw_state(state));
                                imx_spdif_clear_hw_state(state, kSPDIF_Flushing);
                            }
                        }
                        cbuf_trash(&state->rx_circ_buf, state->dma_period_size);
                    } else {
                        if (state->dma_mode == kSPDIF_DMA_Private_Buffer)
                            arch_clean_invalidate_cache_range((vaddr_t) buf, len);
                        /*
                         * Getting head pointer without locking works only as the
                         * worker is the only producer
                         *
                         */
                        cbuf_t *cbuf = &state->rx_circ_buf;
                        dma_buf_vec.vaddr[dma_buf_vec.valid] = cbuf->buf + cbuf->head;
                        printlk(LK_INFO, "Pushing (%d) to %p\n",
                            dma_buf_vec.valid, dma_buf_vec.vaddr[dma_buf_vec.valid]);
                        dma_buf_vec.valid++;
                        if (!state->monitor)
                            imx_spdif_push_to_rx_circ_buffer(state, buf, len);
                    }
                    state->dma_period_elapsed++;
                }
            } else {
                state->dma_period_elapsed = desc->period_elapsed;
                cbuf_trash(&state->rx_circ_buf, payload_sz);
                if (state->rx_frames_warmup) {
                    state->rx_frames_warmup -= (payload_sz / 8);
                    printlk(LK_NOTICE, "hw state: (%x), Skipping (%ld) frames\n",
                            imx_spdif_get_hw_state(state), (payload_sz / 8));
                }
                if (state->rx_frames_warmup <= 0) {
                    state->rx_frames_warmup = 0;
                    imx_spdif_set_hw_state(state, kSPDIF_Warmup);
                    printlk(LK_NOTICE, "hw state: (%x), Warm up completed)\n",
                                imx_spdif_get_hw_state(state));
                    state->clk_fs_rate = SPDIF_GetRxSampleRate(base, state->clk_rate);
                    printlk(LK_NOTICE, "fs rate: %d\n", state->clk_fs_rate);
                    rate_update = true;
                    SPDIF_ClearStatusFlags(base, handle->irq_mask);
                    SPDIF_EnableInterrupts(base, handle->irq_mask);
                }
            }
        } else {
            state->dma_period_elapsed = desc->period_elapsed;
            cbuf_trash(&state->rx_circ_buf, payload_sz);
        }

        smp_mb();
        spin_unlock_irqrestore(&state->rx_lock, lock_state);

        /* Channel Status parsing */
        if (dma_buf_vec.valid > 0) {
            unsigned index;
            size_t len = dma_buf_vec.length;
            bool update;
            for (index = 0; index < dma_buf_vec.valid; index++) {
                void *buf = dma_buf_vec.vaddr[index];
                printlk(LK_INFO, "Pulling (%d) from %p\n",
                                        index, dma_buf_vec.vaddr[index]);
                spin_lock_irqsave(&state->rx_lock, lock_state);
                imx_spdif_cs_parse(state, buf, len, &update);
                spin_unlock_irqrestore(&state->rx_lock, lock_state);
                cs_update = (cs_update || update);
            }
        }

        if (state->cb) {
            if (rate_update)
                dpc_queue(imx_spdif_clk_rate_change_cb, state, DPC_FLAG_NORESCHED);
            if (cs_update)
                dpc_queue(imx_spdif_cs_change_cb, state, DPC_FLAG_NORESCHED);
            if (rate_update || cs_update)
                thread_preempt();
        }
    }
    return 0;
}

static bool imx_spdif_dma_cb(struct dma_descriptor *desc)
{
    struct imx_spdif_state *state = desc->cookie;

    printlk(LK_DEBUG, "Descriptor completed with status %d, period elapsed %llu, bytes transferred %llu \n", desc->status, desc->period_elapsed, desc->bytes_transferred);

    sem_post(&state->dma_period_sem, false);

    return true;
}

static bool imx_spdif_rx_dma_zerocopy_cb(struct dma_descriptor *desc,
                                               uint32_t *addr, uint32_t *len)
{
    struct imx_spdif_state *state = desc->cookie;

    uint32_t offset_next = ((desc->period_elapsed - 1 + state->dma_nr_periods)
            * state->dma_period_size) % state->rx_circ_buf_size;

    if (kSPDIF_DMA_Zerocopy_Cached == desc->dma_mode) {
        void *buffer = *addr - state->dma_buffer_phys + state->dma_buffer;
        arch_clean_invalidate_cache_range((vaddr_t) buffer, (size_t)*len);
    }

    *addr = (uint32_t) state->dma_buffer_phys + offset_next;

    return true;
}

static enum handler_return imx_spdif_isr(void *arg)
{
    struct device *dev = (struct device *)arg;
    ASSERT(dev);

    struct imx_spdif_state *state = dev->state;
    ASSERT(state);

    bool reschedule = false;

    if (imx_spdif_isr_rx(state))
        reschedule = true;

    if (reschedule)
        return INT_RESCHEDULE;
    else
        return INT_NO_RESCHEDULE;
}

static status_t imx_spdif_mem_alloc(struct imx_spdif_state *state)
{
    spdif_handle_t *handle = &state->rx_handle;

    /* This likely means a memory leak if buf is not NULL */
    ASSERT(handle->buf == NULL);
    ASSERT(state->rx_circ_buf.buf == NULL);

    size_t buf_size = IMX_SPDIF_FIFO_STAGES * 2 * 4;
    handle->buf = malloc(buf_size);
    ASSERT(handle->buf);

    size_t circ_buf_size = state->rx_circ_buf_size;
    void *circ_buf;

    const char * circ_buf_name = "spdif rx circ buf";
    switch (state->dma_mode) {
    status_t ret_alloc;
    case kSPDIF_DMA_Zerocopy_Uncached:
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
        state->dma_buffer = circ_buf;
        break;

    case kSPDIF_DMA_Zerocopy_Cached:
        circ_buf = memalign(DMA_BUFFER_ALIGNMENT, circ_buf_size);
        state->dma_buffer = circ_buf;
        break;

    default:
        /* Standard DMA mode with intermediate buffer */
        circ_buf = malloc(circ_buf_size);
        if (state->dma_enable)
            state->dma_buffer = memalign(DMA_BUFFER_ALIGNMENT,
                            state->dma_nr_periods * state->dma_period_size);
        break;
    }

    ASSERT(circ_buf);
    ASSERT(state->dma_buffer);
    state->rx_circ_buf.buf = circ_buf;

    printlk(LK_NOTICE, "rx circ buf (size / address) : %ldbytes / 0x%p\n",
                  circ_buf_size,
                  circ_buf);

    return 0;
}

static status_t imx_spdif_mem_free(struct imx_spdif_state *state)
{
    spdif_handle_t *handle = &state->rx_handle;

    ASSERT(handle->buf != NULL);
    ASSERT(state->rx_circ_buf.buf != NULL);

    free(handle->buf);

    switch (state->dma_mode) {
    case kSPDIF_DMA_Zerocopy_Uncached:
        vmm_free_region(vmm_get_kernel_aspace(),
                (vaddr_t) state->rx_circ_buf.buf);
        break;
    case kSPDIF_DMA_Private_Buffer:
        if (state->dma_enable)
            free(state->dma_buffer);
        free(state->rx_circ_buf.buf);
        break;
    case kSPDIF_DMA_Zerocopy_Cached:
    default:
        free(state->rx_circ_buf.buf);
        break;
    }

    handle->buf = NULL;
    state->rx_circ_buf.buf = NULL;

    return 0;
}



static uint8_t s_spdif_rx_watermark[4] = {1, 4, 8, 16};
static status_t imx_spdif_init(struct device *dev)
{
    const struct device_config_data *config = dev->config;
    struct imx_spdif_state *state;

    printlk(LK_INFO, "%s: entry\n", __func__);
    state = malloc(sizeof(struct imx_spdif_state));
    ASSERT(state);
    memset(state, 0, sizeof(*state));
    dev->state = state;
    state->device = dev;
    state->bus_id = config->bus_id;

    struct device_cfg_reg *reg =
                        device_config_get_register_by_name(config, "core");
    ASSERT(reg);

    struct device_cfg_irq *irq =
                        device_config_get_irq_by_name(config, "core");
    ASSERT(irq);

    spin_lock_init(&state->rx_lock);
    spin_lock_init(&state->cb_lock);
    mutex_init(&state->rx_mutex);
    event_init(&state->rx_wait, false, EVENT_FLAG_AUTOUNSIGNAL);

    imx_init_hw_state(&state->hw_state);
    imx_spdif_set_hw_state(state, kSPDIF_Up);

    struct device_cfg_clk *clk_bus =
                        device_config_get_clk_by_name(config, "bus");
    ASSERT(clk_bus);

    devcfg_set_clock(clk_bus);
    state->clk_rate = clk_bus->rate;

    state->monitor = false;
    devcfg_set_pins_by_name(config->pins_cfg,
                            config->pins_cfg_cnt, "default");

    state->io_base = (SPDIF_Type *) reg->vbase;

    memset(&state->rx_handle, 0, sizeof(spdif_handle_t));


    SPDIF_GetDefaultConfig(&state->hw_config);
    SPDIF_Init(state->io_base, &state->hw_config);

    if (!!!of_device_get_bool(dev, "error-detect"))
        state->rx_handle.irq_mask = kSPDIF_RxFIFOError;
    else
        state->rx_handle.irq_mask = IMX_SPDIF_IRQ_RX_ERRORS;
    state->rx_handle.watermark =
        s_spdif_rx_watermark[
            (state->io_base->SCR & SPDIF_SCR_RXFIFOFULL_SEL_MASK) \
                >> SPDIF_SCR_RXFIFOFULL_SEL_SHIFT
        ];

    uint32_t sz;
    if (of_device_get_int32(dev, "rx-buf-length", &sz))
        sz = SPDIF_MAX_RX_BUF_SIZE;
    state->rx_circ_buf_size = sz;

    state->dma_enable = !!!of_device_get_bool(dev, "disable-dma");
    printlk(LK_NOTICE, "%s:%d: SPDIF driver running with DMA mode %sbled.\n",
            __PRETTY_FUNCTION__, __LINE__, state->dma_enable ? "ena": "disa");

    if (state->dma_enable) {
        uint32_t dma_mode;
        sem_init(&state->dma_period_sem, 0);
        if (of_device_get_int32(dev, "dma-period-length", &state->dma_period_size))
            state->dma_period_size = IMX_SPDIF_DMA_PERIOD;

        if (of_device_get_int32(dev, "dma-nr-period", &state->dma_nr_periods))
            state->dma_nr_periods = IMX_SPDIF_DMA_NR_PERIOD;

        if (state->dma_nr_periods > IMX_SPDIF_MAX_DMA_NR_PERIOD) {
            printlk(LK_NOTICE, "%s:%d: Requested number of periods (%d) exceeds max allowed (%d)\n",
                    __PRETTY_FUNCTION__, __LINE__, state->dma_nr_periods,
                    IMX_SPDIF_MAX_DMA_NR_PERIOD);
            state->dma_nr_periods = IMX_SPDIF_MAX_DMA_NR_PERIOD;
        }

        printlk(LK_NOTICE, "%s:%d: Using %d period buffers of length %d bytes\n",
                __PRETTY_FUNCTION__, __LINE__, state->dma_nr_periods,
                state->dma_period_size);

        if (of_device_get_int32(dev, "rx,dma-mode", &dma_mode))
            dma_mode = kSPDIF_DMA_Private_Buffer;

        state->dma_mode = dma_mode;

        printlk(LK_NOTICE, "%s:%d: SPDIF%d driver running with DMA RX mode %sbled (mode %d)\n",
                __PRETTY_FUNCTION__, __LINE__, state->bus_id,
                state->dma_enable ? "ena" : "disa", dma_mode);

        state->dma_thread = thread_create("spdif dma worker",
                                imx_spdif_dma_worker_thread, state,
                                HIGH_PRIORITY, DEFAULT_STACK_SIZE);
        ASSERT(state->dma_thread);
        thread_detach_and_resume(state->dma_thread);

    }

#if defined(IMX_SPDIF_MEMALLOC_ON_INIT) && (IMX_SPDIF_MEMALLOC_ON_INIT == 1)
    printlk(LK_NOTICE, "%s:%d: Allocating memory once during initialization\n",
            __PRETTY_FUNCTION__, __LINE__);
    imx_spdif_mem_alloc(state);
#endif

    imx_spdif_cs_init(state);

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&imx_spdif_list_lock, lock_state);

    list_add_tail(&imx_spdif_list, &state->node);

    spin_unlock_irqrestore(&imx_spdif_list_lock, lock_state);

    register_int_handler(irq->irq, imx_spdif_isr, dev);

    SPDIF_Dump(state->io_base);

    return NO_ERROR;
}

static status_t imx_spdif_rx_open(struct device *dev)
{
    printlk(LK_INFO, "%s: entry\n", __func__);
    status_t ret;

    struct imx_spdif_state *state = dev->state;
    ASSERT(state);

    ret = imx_spdif_check_state(state, kSPDIF_Up, "opening");
    if (ret)
        return ret;

    if (imx_spdif_is_hw_state(state, kSPDIF_Prepared))
        return 0;

    state->monitor = false;

#if (!defined(IMX_SPDIF_MEMALLOC_ON_INIT)) || (IMX_SPDIF_MEMALLOC_ON_INIT == 0)
    printlk(LK_NOTICE, "Allocating memory while opening\n");
    ret = imx_spdif_mem_alloc(state);
    if (ret) {
        printlk(LK_ERR, "%s:%d: Error while allocating memory\n",
                __PRETTY_FUNCTION__, __LINE__);
        return ret;
    }
#endif

    cbuf_initialize_etc(&state->rx_circ_buf,
                        state->rx_circ_buf_size, state->rx_circ_buf.buf);

    if (state->dma_mode == kSPDIF_DMA_Private_Buffer)
        cbuf_set_flag(&state->rx_circ_buf, CBUF_FLAG_SW_IS_WRITER);
    else
        cbuf_clear_flag(&state->rx_circ_buf, CBUF_FLAG_SW_IS_WRITER);

    imx_spdif_set_hw_state(state, kSPDIF_Prepared);

    struct device_cfg_irq *irq =
                        device_config_get_irq_by_name(dev->config, "core");
    ASSERT(irq);

    unmask_interrupt(irq->irq);

    if (state->use_spdif_cs)
        spdif_cs_register(state->spdif_cs_device, state, imx_spdif_cs_callback);

    return 0;
}

static status_t imx_spdif_rx_close(struct device *dev)
{
    printlk(LK_INFO, "%s: entry\n", __func__);
    status_t ret;

    struct imx_spdif_state *state = dev->state;
    ASSERT(state);

    ret = imx_spdif_check_state(state, kSPDIF_Prepared, "closing");
    if (ret)
        return ret;
#if 0
    if (state->dma_enable)
        dma_terminate_sync(handle->dma_chan);
#endif

#if (!defined(IMX_SPDIF_MEMALLOC_ON_INIT)) || (IMX_SPDIF_MEMALLOC_ON_INIT == 0)
    printlk(LK_NOTICE, "Freeing memory while closing\n");
    ret = imx_spdif_mem_free(state);
    if (ret) {
        printlk(LK_ERR, "%s:%d: Error while freeing memory\n",
                __PRETTY_FUNCTION__, __LINE__);
    }
#endif

    imx_spdif_clear_hw_state(state, kSPDIF_Prepared);

    struct device_cfg_irq *irq =
                        device_config_get_irq_by_name(dev->config, "core");
    ASSERT(irq);

    mask_interrupt(irq->irq);

    if (state->use_spdif_cs)
        spdif_cs_register(state->spdif_cs_device, NULL, NULL);

    return 0;
}

static status_t imx_spdif_rx_start(struct device *dev)
{
    printlk(LK_INFO, "%s: entry\n", __func__);
    status_t ret;

    struct imx_spdif_state *state = dev->state;
    ASSERT(state);

    if (imx_spdif_is_hw_state(state, kSPDIF_On))
        return 0;

    ret = imx_spdif_check_state(state, kSPDIF_Ready, "starting");
    if (ret)
        return ret;

    SPDIF_Type *base = state->io_base;
    ASSERT(base);

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->rx_lock, lock_state);

    imx_spdif_cs_set_sync(state, false);

    if (state->dma_enable) {
       switch (state->dma_mode)
       {
       case kSPDIF_DMA_Zerocopy_Uncached:
       case kSPDIF_DMA_Zerocopy_Cached:
           cbuf_reset_indexes(&state->rx_circ_buf);
           break;
       case kSPDIF_DMA_Private_Buffer:
       default:
           break;
       }
    }

    if (state->dma_enable)
        dma_resume_channel(state->rx_handle.dma_chan);
    SPDIF_RxStart(base, 0, state->dma_enable);

    /* Set state to On */
    imx_spdif_set_hw_state(state, kSPDIF_On);

    spin_unlock_irqrestore(&state->rx_lock, lock_state);

    return 0;
}

static status_t imx_spdif_rx_flush(struct device *dev)
{
    printlk(LK_INFO, "%s: entry\n", __func__);
    status_t ret = 0;

    struct imx_spdif_state *state = dev->state;
    ASSERT(state);

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->rx_lock, lock_state);

    if (imx_spdif_is_hw_state(state, kSPDIF_Busy))
        imx_spdif_set_hw_state(state, kSPDIF_Flushing);
    else
        cbuf_reset(&state->rx_circ_buf);

    spin_unlock_irqrestore(&state->rx_lock, lock_state);

    return ret;
}

static status_t imx_spdif_rx_stop(struct device *dev)
{
    printlk(LK_INFO, "%s: entry\n", __func__);
    status_t ret;

    struct imx_spdif_state *state = dev->state;
    ASSERT(state);

    spdif_handle_t *handle = &state->rx_handle;

    ret = imx_spdif_check_state(state, kSPDIF_On, "stopping");
    if (ret)
        return ret;

    SPDIF_Type *base = state->io_base;
    ASSERT(base);

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->rx_lock, lock_state);
    handle->xfer_remaining = ERR_IO;

    SPDIF_RxStop(base, kSPDIF_LockLoss | kSPDIF_RxDPLLLocked | handle->irq_mask, state->dma_enable);
    if (state->dma_enable)
        dma_pause_channel(state->rx_handle.dma_chan);
    imx_spdif_clear_hw_state(state, kSPDIF_Locked | kSPDIF_Warmup);

    imx_spdif_clear_hw_state(state, kSPDIF_On);

    /* TODO: abort any on going transfer */
    spin_unlock_irqrestore(&state->rx_lock, lock_state);

    event_signal(&state->rx_wait, false);

    if (state->use_spdif_cs)
        spdif_cs_stop(state->spdif_cs_device);

    return 0;
}

static status_t imx_spdif_rx_setup(struct device *dev, spdif_hw_params_t *fmt)
{
    printlk(LK_INFO, "%s: entry\n", __func__);
    status_t ret = 0;

    struct imx_spdif_state *state = dev->state;
    ASSERT(state);

    spdif_handle_t *handle = &state->rx_handle;

    ret = imx_spdif_check_state(state, kSPDIF_Prepared, "configuring");
    if (ret)
        return ret;

    SPDIF_Type *base = state->io_base;
    ASSERT(base);

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->rx_lock, lock_state);

    switch (fmt->fmt) {
    case SPDIF_FMT_IEC60958:
        state->hw_config.isRxRawMode = true;
        break;
    case SPDIF_FMT_IEC60958_OUTBAND:
    default:
        printlk(LK_ERR, "%s:%d: Unsupported format (%d), aborting\n",
                __PRETTY_FUNCTION__, __LINE__, fmt->fmt);
        handle->irq_mask |= \
                kSPDIF_ValidityFlagNoGood | kSPDIF_RxIllegalSymbol | \
                kSPDIF_RxParityBitError | kSPDIF_RxFIFOResync;
        ret = ERR_NOT_SUPPORTED;
        goto exit_rx_setup;
    }

    state->hw_params = *fmt;

    printlk(LK_NOTICE, "fmt %d, bit width: %d, period %d\n",
                        state->hw_params.fmt,
                        state->hw_params.bitWidth,
                        state->hw_params.period_size);

    SPDIF_Init(state->io_base, &state->hw_config);

    if (state->dma_enable) {

        if (state->rx_handle.dma_chan) {
            struct dma_descriptor *olddesc = state->rx_handle.dma_chan->desc;
            dma_terminate_sync(state->rx_handle.dma_chan);
            free(olddesc);
            state->rx_handle.dma_chan = NULL;
        }
        /* request a DMA RX channel */
        struct dma_chan *dma_chan;
        dma_chan = dma_request_chan(dev, "rx");
        ASSERT(dma_chan);
        state->rx_handle.dma_chan = dma_chan;

        /* configure DMA RX channel */
        struct dma_slave_config cfg;
        memset(&cfg, 0, sizeof(struct dma_slave_config));
        cfg.direction = DMA_DEV_TO_MEM;
        cfg.src_addr = SPDIF_RxGetLeftDataRegisterAddress(state->io_base);
        cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
        cfg.dst_addr_width = cfg.src_addr_width;
        cfg.src_maxburst = state->rx_handle.watermark;
        cfg.dst_maxburst = cfg.src_maxburst;

        ret = dma_slave_config(dma_chan, &cfg);
        if (ret) {
            printlk(LK_ERR, "%s:%d: Failed to configure DMA channel.\n",
                    __PRETTY_FUNCTION__, __LINE__);
            return ret;
        }
        /* Prepare and submit Back-End DMA channel */
        struct dma_descriptor *desc;

        state->dma_buffer_phys = phys_to_dma(vaddr_to_paddr(state->dma_buffer));
        desc = dma_prep_dma_cyclic(
            state->rx_handle.dma_chan,
            state->dma_buffer_phys,
            state->dma_period_size * state->dma_nr_periods,
            state->dma_period_size,
            DMA_DEV_TO_MEM
        );
        state->dma_period_elapsed = 0;
        state->dma_desc = desc;
        desc->cb = imx_spdif_dma_cb;
        desc->cookie = state;

        desc->dma_mode = state->dma_mode;
        desc->zerocopy_cb = imx_spdif_rx_dma_zerocopy_cb;

        dma_submit(desc);
    }

    /* Enable DPLL interrupt */
    SPDIF_EnableInterrupts(base, kSPDIF_RxDPLLLocked | kSPDIF_LockLoss);
    imx_spdif_set_hw_state(state, kSPDIF_Ready);
    imx_spdif_clear_hw_state(state, kSPDIF_Locked | kSPDIF_Warmup);

exit_rx_setup:
    spin_unlock_irqrestore(&state->rx_lock, lock_state);

    return ret;
}

static status_t imx_spdif_read(struct device *dev, void *buf, size_t len)
{
    printlk(LK_INFO, "%s: entry\n", __func__);

    size_t remaining, read, used;
    void *xfer;
    int ret = 0;

    struct imx_spdif_state *state = dev->state;
    ASSERT(state);

    spdif_handle_t *handle = &state->rx_handle;

    SPDIF_Type *base = state->io_base;
    ASSERT(base);

#ifdef IMX_SPDIF_PARANOID
    ASSERT(imx_spdif_is_hw_state(state, kSPDIF_Ready));
#endif

    if (!(imx_spdif_is_hw_state(state, kSPDIF_Ready)))
        return ERR_NOT_READY;

    if (imx_spdif_is_hw_state(state, kSPDIF_Busy))
        return ERR_BUSY;

    mutex_acquire(&state->rx_mutex);

    if (!(imx_spdif_is_hw_state(state, kSPDIF_On))) {
#if ((defined IMX_SPDIF_READ_AUTOSTART) && (IMX_SPDIF_READ_AUTOSTART == 1))
        imx_spdif_rx_start(dev);
#else
        /* read operation when driver is stopped - abort */
        printlk(LK_ERR, "%s:%d: port stopped - abort\n", __PRETTY_FUNCTION__,
                __LINE__);
        ret = ERR_BAD_STATE;
        goto done;
#endif
    }

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->rx_lock, lock_state);

    imx_spdif_set_hw_state(state, kSPDIF_Busy);
    used = cbuf_space_used(&state->rx_circ_buf);
    if (len > used)
        handle->xfer_remaining = len - used;
    else
        handle->xfer_remaining = 0;

retry:
    smp_wmb();
    spin_unlock_irqrestore(&state->rx_lock, lock_state);
    remaining = len;
    xfer = buf;

    printlk(LK_INFO, "len: %ld\n", len);

    while (remaining) {
        read = cbuf_read(&state->rx_circ_buf, xfer, remaining, false);
        remaining -= read;
        xfer += read;

        if (remaining)
            ret = event_wait_timeout(&state->rx_wait, 1000); /* 1 second timeout */

        if (ret == ERR_TIMED_OUT) {
            memset(xfer, 0, remaining);
            break;
        }

        if (handle->xfer_remaining < 0) {
            printlk(LK_NOTICE, "%s:%d: Abort (%ld)\n", __PRETTY_FUNCTION__,
                    __LINE__, handle->xfer_remaining);
            break;
        }
    }

    spin_lock_irqsave(&state->rx_lock, lock_state);

    ret = handle->xfer_remaining;
    if (ret) {
        printlk(LK_INFO, "spdif cbuf reset\n");
        cbuf_reset(&state->rx_circ_buf);
        /* TODO: Restart HW if ERR_IO */
    }

    handle->xfer_remaining = 0;
    imx_spdif_clear_hw_state(state,
                    kSPDIF_Busy | kSPDIF_Done | kSPDIF_Error | kSPDIF_Flushing);
    smp_wmb();
#if ((defined IMX_SPDIF_READ_SILENT_RETRY) && (IMX_SPDIF_READ_SILENT_RETRY == 1))
    if (ret < 0) {
        printlk(LK_ERR, "%s:%d: Error (%d) while reading, retrying...\n",
                __PRETTY_FUNCTION__, __LINE__, ret);
        handle->xfer_remaining = len;
        imx_spdif_set_hw_state(state, kSPDIF_Busy);
        goto retry;
    }
#endif
    spin_unlock_irqrestore(&state->rx_lock, lock_state);

done:
    printlk(LK_INFO, "len: %ld..done(%d)\n", len, ret);
    mutex_release(&state->rx_mutex);

    return ret;
}

static status_t imx_spdif_ioctl(struct device *dev, int request, void *argp)
{
    status_t ret = ERR_NOT_FOUND;
    struct imx_spdif_state *state;

    if (!dev)
        return ERR_NOT_FOUND;
    state = dev->state;

    if ((IOCGROUP(request) != IOCTL_DEV_SPDIF) || (!argp))
        return ERR_INVALID_ARGS;

    int cmd = IOCBASECMD(request);

    switch(request) {
    case SPDIF_IOC_MONITOR_SET:
        state->monitor = !!(*((int *) argp));
        ret = NO_ERROR;
        break;

    default:
        printlk(LK_ERR, "%s:%d: Invalid spdif ioctl(%d)\n",
                __PRETTY_FUNCTION__, __LINE__, cmd);
        ret = ERR_NOT_FOUND;
    }

    return ret;
}

static status_t imx_spdif_set_callback(struct device *dev, spdif_cb_t cb, void *cookie)
{
    struct imx_spdif_state *state = dev->state;
    status_t ret = 0;

    printlk(LK_NOTICE, "Set SPDIF callback/cookie: [%p/%p]\n", cb, cookie);
    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->cb_lock, lock_state);

    state->cb = cb;
    state->cb_cookie = cookie;

    spin_unlock_irqrestore(&state->cb_lock, lock_state);

    return ret;
};

static struct device_class spdif_device_class = {
    .name = "spdif",
};

static struct spdif_ops the_ops = {
    .std = {
        .device_class = &spdif_device_class,
        .init = imx_spdif_init,
        .ioctl = imx_spdif_ioctl,
    },
    .tx_open = NULL,
    .tx_close = NULL,
    .tx_start = NULL,
    .tx_stop = NULL,
    .tx_setup = NULL,
    .tx_flush = NULL,
    .write = NULL,
    .set_callback = imx_spdif_set_callback,
    .rx_open = imx_spdif_rx_open,
    .rx_close = imx_spdif_rx_close,
    .rx_start = imx_spdif_rx_start,
    .rx_stop = imx_spdif_rx_stop,
    .rx_setup = imx_spdif_rx_setup,
    .rx_flush = imx_spdif_rx_flush,
    .read = imx_spdif_read,
};

DRIVER_EXPORT_WITH_LVL(spdif, &the_ops.std, DRIVER_INIT_PLATFORM);

struct device *class_spdif_get_device_by_id(int id)
{
    struct device *dev = NULL;
    struct imx_spdif_state *state = NULL;

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&imx_spdif_list_lock, lock_state);

    list_for_every_entry(&imx_spdif_list, state, struct imx_spdif_state, node) {
        if (state->bus_id == id) {
            dev = state->device;
            break;
        }
    }

    spin_unlock_irqrestore(&imx_spdif_list_lock, lock_state);

    return dev;
}
