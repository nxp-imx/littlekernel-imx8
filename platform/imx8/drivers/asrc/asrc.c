/*
 * The Clear BSD License
 * Copyright 2020-2021 NXP
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
#include <dev/class/asrc.h>
#include <platform.h>
#include <sys/ioctl.h>
#include <dev/asrc_ioctl.h>
#include "asrc_hw.h"

#include <dev/dma.h>
#include <dev/interrupt.h>

#include <kernel/spinlock.h>
#include <kernel/thread.h>
#include <kernel/semaphore.h>
#include <kernel/event.h>
#include <kernel/vm.h>

#include <lib/appargs.h>
#include <lib/cbuf.h>
#include <assert.h>
#include <err.h>
#include <debug.h>

#include <platform/imx_common.h>


/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define ASRC_MAX_CONTEXTS           (ASRC_SUPPORT_MAXIMUM_CONTEXT_PROCESSOR_NUMBER)
#define ASRC_MAX_SUPPORTED_CHANNELS (32)

#define ASRC_DMA_RX 0
#define ASRC_DMA_TX 1

#define IMX_ASRC_DMA_NR_PERIOD      (8)
#define IMX_ASRC_DMA_PERIOD_RATIO   (4) /* one dma period = fmt_period / ratio */

#define ASRC_FRMT_MAX_PERIOD        (512)

#define ASRC_DMA_RX_MAX_PERIOD      (ASRC_FRMT_MAX_PERIOD * 4 * 8) / IMX_ASRC_DMA_PERIOD_RATIO
#define ASRC_DMA_TX_MAX_PERIOD      (ASRC_FRMT_MAX_PERIOD * 4 * 8) / IMX_ASRC_DMA_PERIOD_RATIO

#define ASRC_DMA_OUT_CBUF_SIZE      (64 * 1024)

#define ASRC_DMA_MAX_REQUEST_SIZE   (2048)
#define ASRC_DMA_MODE               (2) /* zero-copy */
#define ASRC_MIN_XFER_SIZE          (4096)

enum asrc_hw_state {
    kASRC_Down      = (1UL << 0),   /*!< ASRC is powered down */
    kASRC_Up        = (1UL << 1),   /*!< ASRC is powered up */
    kASRC_Prepared  = (1UL << 2),   /*!< ASRC is prepared */
    kASRC_Ready     = (1UL << 3),   /*!< ASRC is prepared and ready */
    kASRC_On        = (1UL << 4),   /*!< ASRC is on. IRQ flowing */
    kASRC_Busy      = (1UL << 5),   /*!< ASRC is busy, transfer on going */
    kASRC_Flushed   = (1UL << 6),   /*!< ASRC is flushed */
    kASRC_Done      = (1UL << 7),   /*!< Transfer is done */
    kASRC_Error     = (1UL << 8),   /*!< Transfer error occurred */
    kASRC_Running   = kASRC_On,
};

struct imx_asrc_state {
    int bus_id;
    ASRC_Type *io_base;
    struct device *device;
    struct list_node node;
    struct imx_hw_state_s hw_state;
    unsigned clk_rate;
    unsigned irq_stats;

    /* Interface with ASRC HW */
    asrc_context_t context;
    asrc_context_config_t context_config;

    float dynamic_ratio;
    unsigned data_width;
    unsigned resampler_fract_bits;
    bool first_start;


    /* Context data */
    event_t runstop_completion_event;

    /* DMA (per context) */
    bool dma_enable;

    semaphore_t dma_rx_period_sem;
    event_t dma_rx_completion_event;
    unsigned dma_rx_nr_periods;
    unsigned dma_rx_period_size_max;
    thread_t *dma_rx_thread;
    struct dma_chan *rx_dma_chan;
    struct dma_descriptor *rx_dma_desc;
    unsigned rx_dma_rem_size;
    unsigned rx_dma_period_size;
    unsigned rx_dma_n_periods;

    semaphore_t dma_tx_period_sem;
    event_t dma_tx_completion_event;
    unsigned dma_tx_nr_periods;
    unsigned dma_tx_period_size_max;
    thread_t *dma_tx_thread;
    struct dma_chan *tx_dma_chan;
    struct dma_descriptor *tx_dma_desc;
    unsigned tx_dma_size;
    unsigned tx_dma_period_size;
    unsigned tx_dma_n_periods;

    cbuf_t dma_tx_cbuf;
    void *dma_tx_cbuf_buffer;
    unsigned dma_tx_buffer_size;
    paddr_t dma_tx_buffer_phys;
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
const char *asrc_dma_channels_name[ASRC_MAX_CONTEXTS][2] = {
    { "rx0", "tx0" },
    { "rx1", "tx1" },
    { "rx2", "tx2" },
    { "rx3", "tx3" }
};

/*******************************************************************************
 * Code
 ******************************************************************************/
static inline void imx_asrc_set_hw_state(struct imx_asrc_state *handle, uint32_t state)
{
    imx_set_hw_state(&handle->hw_state, state);
}
static inline void imx_asrc_clear_hw_state(struct imx_asrc_state *handle, uint32_t state)
{
    imx_clear_hw_state(&handle->hw_state, state);
}
static inline bool imx_asrc_is_hw_state(struct imx_asrc_state *handle,
                                        enum asrc_hw_state expected_state)
{
    return imx_is_hw_state(&handle->hw_state, expected_state);
}
static inline status_t imx_asrc_check_state(
                                    struct imx_asrc_state *handle,
                                    uint32_t expected_state,
                                    const char *msg)
{
    return imx_check_hw_state(&handle->hw_state, expected_state, msg);
}
static inline status_t imx_asrc_check_strict_state(
                                    struct imx_asrc_state *handle,
                                    uint32_t expected_state,
                                    const char *msg)
{
    return imx_check_strict_hw_state(&handle->hw_state, expected_state, msg);
}
static inline uint32_t imx_asrc_get_hw_state(struct imx_asrc_state *state)
{
    return imx_get_hw_state(&state->hw_state);
}

static enum handler_return imx_asrc_isr(void *arg)
{
    struct device *dev = (struct device *)arg;
    ASSERT(dev);

    struct imx_asrc_state *state = dev->state;
    ASSERT(state);

    bool reschedule = false;

    uint32_t asrc_irq_flags = ASRC_GetInterruptStatus(state->io_base);

    state->irq_stats++;

    ASSERT(0 == (asrc_irq_flags & kASRC_Context0InputFifoOverflow));

    //ASSERT(0 == (asrc_irq_flags & kASRC_Context0OutFifoReadEmpty));

    if (asrc_irq_flags & kASRC_Context0RunStopDone) {
        printlk(LK_VERBOSE, "%s: Context%u flushed ! (RunStopDone)\n", __func__,
                (unsigned ) state->context);
        ASRC_ClearInterruptStatus(state->io_base, kASRC_Context0RunStopDone);
        event_signal(&state->runstop_completion_event, false);
    }

    if (reschedule)
        return INT_RESCHEDULE;
    else
        return INT_NO_RESCHEDULE;
}

static bool imx_asrc_rx_dma_cb(struct dma_descriptor *desc)
{
    struct imx_asrc_state *state = desc->cookie;
    bool dma_release_descriptor_to_dma = false;

    if (desc->status == DMA_XFER_COMPLETE) {
        /*** Transfer success ***/

        if (state->rx_dma_rem_size < state->rx_dma_period_size) {
            /* this MUST never been reached */
            printlk(LK_ERR, "Error: End of DMA transfer signaled while %u bytes remaining only...\n", state->rx_dma_rem_size);
            ASSERT(0);
        }

        /* Compute remaining data size to be transferred */
        state->rx_dma_rem_size -= state->rx_dma_period_size;

        /* Check whether current buffer descriptor must be released or not,
         * based on remaining data size and other buffer descriptors which
         * have already been released.
         */
        if ((state->dma_rx_nr_periods * state->rx_dma_period_size) <= state->rx_dma_rem_size) {
            dma_release_descriptor_to_dma = true;
        } else {
            printlk(LK_DEBUG,
                              "Descriptor RX won't be released, remaining transfer %u \n",
                              state->rx_dma_rem_size);
        }

        if (state->rx_dma_rem_size < state->rx_dma_period_size) {
            /* Pause immediately DMA channel :
             * this is done while holding the DMA spinlock, in order
             * to avoid further races while freeing DMA descriptor
             */
            dma_pause_channel(state->rx_dma_chan);
            /* cannot free dma descriptor right now as we are in ISR context,
             * but it will be done by normal thread right after completion
             * signal's reception.
             * However, force it to NULL here to prevent SDMA from handling
             * more buffer descriptor related to it.
             */
            state->rx_dma_chan->desc = NULL;

            printlk(LK_DEBUG, "Signaling DMA end of transfer\n");
            event_signal(&state->dma_rx_completion_event, false);

            //ASSERT(state->rx_dma_rem_size == 0);
        }
    } else {
        printlk(LK_WARNING, "Wrong DMA descriptor status (set to %u)\n", (unsigned)desc->status);
    }

    return dma_release_descriptor_to_dma;
}

static bool imx_asrc_rx_dma_zerocopy_cb(struct dma_descriptor *desc,
                                        uint32_t *addr, uint32_t *len)
{
    if (desc) {
        struct imx_asrc_state *state = desc->cookie;

        /* Move input buffer pointer */
        *addr += state->dma_rx_nr_periods * state->rx_dma_period_size;
    }
    return true;
}

static bool imx_asrc_tx_dma_cb(struct dma_descriptor *desc)
{
    struct imx_asrc_state *state = desc->cookie;
    bool dma_release_descriptor_to_dma = false;

    if (desc->status == DMA_XFER_COMPLETE) {
        /*** Transfer success ***/

        /* Accumulate output data size */
        state->tx_dma_size += state->tx_dma_period_size;

        /* Give back descriptor to DMA */
        dma_release_descriptor_to_dma = true;

        /* Notify worker thread */
        sem_post(&state->dma_tx_period_sem, false);
    }

    return dma_release_descriptor_to_dma;
}

static bool imx_asrc_tx_dma_zerocopy_cb(struct dma_descriptor *desc,
                                        uint32_t *addr, uint32_t *len)
{
    struct imx_asrc_state *state = desc->cookie;
    uint32_t cbuf_used_size = cbuf_size(&state->dma_tx_cbuf);

    uint32_t offset = ((desc->period_elapsed - 1
            + state->dma_tx_nr_periods)
            * state->tx_dma_period_size)
            % cbuf_used_size;

    /* Move output buffer pointer */
    *addr = (uint32_t) state->dma_tx_buffer_phys + offset;

    return true;
}

static int imx_asrc_tx_dma_worker_thread(void *arg)
{
    struct imx_asrc_state *state = arg;

    printlk(LK_VERBOSE, "%s: entry\n", __func__);

    while (true) {
        size_t written_bytes;

        /* Waiting for notification from DMA callback */
        sem_wait(&state->dma_tx_period_sem);

        printlk(LK_DEBUG, "%s: %u bytes transferred on tx\n",
                __func__,
                state->tx_dma_size);

        /* Update circular buffer indexes */
        written_bytes = cbuf_write(&state->dma_tx_cbuf, NULL,
                                   (size_t) state->tx_dma_period_size, false);
        ASSERT(written_bytes == (size_t) state->tx_dma_period_size);
    }

    return 0;
}

static status_t imx_asrc_dma_setup(struct device *dev, bool is_rx, asrc_context_t context)
{
    unsigned ret = 0;
    const char *channel_name = asrc_dma_channels_name[(unsigned)context][is_rx ? ASRC_DMA_RX : ASRC_DMA_TX];

    enum dma_data_direction dma_transfer_direction;
    struct dma_slave_config cfg_dma;
    struct dma_chan *dma_chan = NULL;
    struct dma_chan **pp_dma_chan;

    struct imx_asrc_state *state = dev->state;
    ASSERT(state);

    ASSERT(context == kASRC_Context0);

    /* FIFO refill size, i.e. delta between fifo size and fifo watermark */
    unsigned fifo_watermark, fifo_depth;
    unsigned bytes_per_request;

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);
    printlk(LK_VERBOSE, "%s: DMA %s configuration request\n",
            __PRETTY_FUNCTION__, is_rx ? "rx" : "tx");

    /* DMA requests transfer size computation.
     *  transfer_size = f(fifo_watermark,n_channels) */
    if (is_rx) {
        /* RX */
        fifo_depth = FSL_ASRC_INPUT_FIFO_DEPTH;
        fifo_watermark = state->context_config.contextInput.watermark;
        bytes_per_request = fifo_depth - fifo_watermark;
        bytes_per_request *= state->context_config.contextInput.accessCtrl.accessGroupLen;
        bytes_per_request *= state->context_config.contextInput.accessCtrl.accessIterations;
        bytes_per_request *= state->data_width;
        pp_dma_chan = &state->rx_dma_chan;

        state->rx_dma_period_size = bytes_per_request;
    } else {
        /* TX */
        fifo_depth = FSL_ASRC_OUTPUT_FIFO_DEPTH;
        fifo_watermark = state->context_config.contextOutput.watermark;
        bytes_per_request = fifo_watermark;
        bytes_per_request *= state->context_config.contextOutput.accessCtrl.accessGroupLen;
        bytes_per_request *= state->context_config.contextOutput.accessCtrl.accessIterations;
        bytes_per_request *= state->data_width;
        pp_dma_chan = &state->tx_dma_chan;

        /* define DMA period size to its maximum value, making sure
         * that period_size is an integer multiple of bytes_per_request */
        state->tx_dma_period_size = state->dma_tx_period_size_max / bytes_per_request * bytes_per_request;
    }

    printlk(LK_NOTICE, "%s: %s: FIFO watermark/depth set to %u/%u\n",
            __PRETTY_FUNCTION__, channel_name, fifo_watermark, fifo_depth);
    printlk(LK_NOTICE, "%s: %s: DMA will transfer a total of %u bytes per request\n",
                __PRETTY_FUNCTION__, channel_name, bytes_per_request);

    /* configure DMA channel */
    if (is_rx) {
        /* RX */
        dma_transfer_direction = DMA_MEM_TO_DEV;

        cfg_dma.direction = dma_transfer_direction;
        cfg_dma.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
        cfg_dma.dst_addr_width = cfg_dma.src_addr_width;
        cfg_dma.src_maxburst = bytes_per_request;
        cfg_dma.dst_maxburst = cfg_dma.src_maxburst;
        cfg_dma.src_addr = cfg_dma.dst_addr = (paddr_t) &state->io_base->WRFIFO[0];

        /* hack for sai multi-fifo sdma script usage */
        cfg_dma.src_sample_num = cfg_dma.dst_sample_num = 2; /* arbitrary */
        cfg_dma.src_fifo_num = cfg_dma.dst_fifo_num = 1; /* use a single fifo in asrc */

    } else {
        /* TX */
        dma_transfer_direction = DMA_DEV_TO_MEM;

        cfg_dma.direction = dma_transfer_direction;
        cfg_dma.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
        cfg_dma.dst_addr_width = cfg_dma.src_addr_width;
        cfg_dma.src_maxburst = bytes_per_request;
        cfg_dma.dst_maxburst = cfg_dma.src_maxburst;
        cfg_dma.src_addr = cfg_dma.dst_addr = (paddr_t) &state->io_base->RDFIFO[0];

        /* hack for sai multi-fifo sdma script usage */
        cfg_dma.src_sample_num = cfg_dma.dst_sample_num = 2; /* arbitrary */
        cfg_dma.src_fifo_num = cfg_dma.dst_fifo_num = 1; /* use a single fifo in asrc */
    }

    /* Get channel */
    dma_chan = dma_request_chan(dev, channel_name);
    ASSERT(dma_chan);
    *pp_dma_chan = dma_chan;

    printlk(LK_DEBUG, "%s: found channel %s\n", __PRETTY_FUNCTION__, dma_chan->name);

    /* Configure channel */
    printlk(LK_DEBUG, "%s: cfg_dma.src_addr_width=%d\n", __PRETTY_FUNCTION__, cfg_dma.src_addr_width);
    printlk(LK_DEBUG, "%s: cfg_dma.src_maxburst=%d\n", __PRETTY_FUNCTION__, cfg_dma.src_maxburst);

    /* set DMA controller specific parameters */
    ret = dma_slave_config(*pp_dma_chan, &cfg_dma);
    if (ret) {
        printlk(LK_ERR, "%s:%d: asrc%d: Failed to configure DMA %s channel.\n",
                __PRETTY_FUNCTION__, __LINE__, state->bus_id,
               channel_name);
        return ret;
    }

    return ret;
}

static status_t imx_asrc_dma_start(struct device *dev, bool is_rx, asrc_context_t context, asrc_transfer_t *xfer)
{
    status_t ret = NO_ERROR;

    struct imx_asrc_state *state = dev->state;
    ASSERT(state);

    ASSERT(context == kASRC_Context0);

    paddr_t dma_buffer_phys; /* buffer address */
    unsigned n_periods;      /* number of buffer descriptors per DMA channel */
    unsigned period_size;    /* DMA period size (in bytes) */

    /* Set DMA channel name */
    const char* channel_name = asrc_dma_channels_name[(unsigned)context][is_rx ? ASRC_DMA_RX : ASRC_DMA_TX];

    size_t in_size = (size_t) xfer->inDataSize;

    if (is_rx) {
        /**** IN ****/

        /* Prepare and submit Back-End DMA channel */
        n_periods = state->dma_rx_nr_periods;
        period_size = state->rx_dma_period_size; /* 1 DMA period = 1 ASRC request */

        if (period_size > ASRC_DMA_MAX_REQUEST_SIZE) {
            printlk(LK_WARNING,
                    "asrc%d: %s: DMA period size %u exceeds maximum allowed value\n",
                    state->bus_id, channel_name, period_size);
            period_size = state->rx_dma_period_size = ASRC_DMA_MAX_REQUEST_SIZE;
            printlk(LK_WARNING,
                    "asrc%d: %s: Restricting DMA period size to %uB\n",
                    state->bus_id, channel_name, period_size);
            state->rx_dma_chan->slave_config.src_maxburst = period_size;
            state->rx_dma_chan->slave_config.dst_maxburst = period_size;
        }

        if (period_size * n_periods > in_size) {
            printlk(LK_WARNING,
                    "asrc%d: %s: Sum of buffer descriptors' size (%uB) exceeds input payload's size (%uB)\n",
                    state->bus_id, channel_name, period_size * n_periods, (unsigned )in_size);
            n_periods = state->dma_rx_nr_periods = in_size / period_size;
            printlk(LK_WARNING,
                    "asrc%d: Restricting DMA number of period descriptors to %u\n",
                    state->bus_id, n_periods);
        }

        dma_buffer_phys = phys_to_dma(vaddr_to_paddr(xfer->inDataAddr));

        state->rx_dma_desc = dma_prep_dma_cyclic(
                state->rx_dma_chan, /* dma channel handle */
                dma_buffer_phys, /* buf_addr */
                n_periods * period_size, /* buf_len */
                period_size, /* period_len */
                state->rx_dma_chan->slave_config.direction /* direction */
                );

        if (NULL == state->rx_dma_desc) {
            printlk(LK_DEBUG, "asrc%d: %s: Failed to prepare SDMA descriptor",
                    state->bus_id, channel_name);
            return ERR_NO_RESOURCES;
        }

        state->rx_dma_n_periods = n_periods;
        state->rx_dma_rem_size = in_size;

        state->rx_dma_desc->cookie = state;
        state->rx_dma_desc->cb = imx_asrc_rx_dma_cb;
        state->rx_dma_desc->zerocopy_cb = imx_asrc_rx_dma_zerocopy_cb;
        state->rx_dma_desc->dma_mode = ASRC_DMA_MODE;

        /* Cache: clean and invalidate input buffer */
        arch_clean_cache_range((vaddr_t) xfer->inDataAddr, xfer->inDataSize);

        dma_submit(state->rx_dma_desc);
        dma_resume_channel(state->rx_dma_chan);
    } else {
        /**** OUT ****/

        /* Prepare and submit Back-End DMA channel */
        n_periods = state->dma_tx_nr_periods;
        period_size = state->tx_dma_period_size; /* bytes per ASRC request */

        if (period_size > ASRC_DMA_MAX_REQUEST_SIZE) {
            printlk(LK_WARNING,
                    "asrc%d: %s: DMA period size %u exceeds maximum allowed value\n",
                    state->bus_id, channel_name, period_size);
            period_size = state->tx_dma_period_size = ASRC_DMA_MAX_REQUEST_SIZE
                    / state->tx_dma_chan->slave_config.src_maxburst
                    * state->tx_dma_chan->slave_config.src_maxburst;
            printlk(LK_WARNING,
                    "asrc%d: %s: Restricting DMA period size to %uB\n",
                    state->bus_id, channel_name, period_size);
        }

        dma_buffer_phys = phys_to_dma(vaddr_to_paddr(state->dma_tx_cbuf_buffer));

        state->tx_dma_desc = dma_prep_dma_cyclic(
                state->tx_dma_chan, /* dma channel handle */
                dma_buffer_phys, /* buf_addr */
                n_periods * period_size, /* buf_len */
                period_size, /* period_len */
                state->tx_dma_chan->slave_config.direction /* direction */
                );

        if (NULL == state->tx_dma_desc) {
            printlk(LK_DEBUG, "asrc%d: %s: Failed to prepare SDMA descriptor",
                    state->bus_id, channel_name);
            return ERR_NO_RESOURCES;
        }

        /* Make sure circular buffer size is an integer multiple of dma period size */
        size_t cbuf_usable_size = (size_t) (state->dma_tx_buffer_size / period_size * period_size);
        cbuf_adjust_size(&state->dma_tx_cbuf, cbuf_usable_size);
        if (cbuf_usable_size != state->dma_tx_buffer_size) {
            printlk(LK_INFO,
                    "asrc%d: %s: limited output circular buffer size to %lu\n",
                    state->bus_id, channel_name, cbuf_usable_size);
        }

        /* Initialize output circular buffer... */
        cbuf_reset_indexes(&state->dma_tx_cbuf);
        /* ... and keep track of its physical base address */
        state->dma_tx_buffer_phys = dma_buffer_phys;

        state->tx_dma_period_size = period_size;
        state->tx_dma_n_periods = n_periods;
        state->tx_dma_size = 0;

        state->tx_dma_desc->cookie = state;
        state->tx_dma_desc->cb = imx_asrc_tx_dma_cb;
        state->tx_dma_desc->zerocopy_cb = imx_asrc_tx_dma_zerocopy_cb;
        state->tx_dma_desc->dma_mode = ASRC_DMA_MODE;

        dma_submit(state->tx_dma_desc);
        dma_resume_channel(state->tx_dma_chan);
    }

    return ret;
}

static status_t imx_asrc_dma_stop(struct device *dev, asrc_context_t context)
{
    status_t ret = NO_ERROR;

    struct imx_asrc_state *state = dev->state;
    ASSERT(state);

    ASSERT(context == kASRC_Context0);

    struct dma_descriptor *rx_desc = state->rx_dma_chan->desc;
    struct dma_descriptor *tx_desc = state->tx_dma_chan->desc;

    dma_terminate_sync(state->rx_dma_chan);
    dma_terminate_sync(state->tx_dma_chan);

    if (rx_desc) free(rx_desc);
    if (tx_desc) free(tx_desc);

    return ret;
}

/* Init */
static status_t imx_asrc_init(struct device *dev)
{
    const struct device_config_data *config = dev->config;
    struct imx_asrc_state *state;

    printlk(LK_VERBOSE, "%s: entry\n", __func__);

    state = malloc(sizeof(struct imx_asrc_state));
    ASSERT(state);
    memset(state, 0, sizeof(*state));
    dev->state = state;
    state->device = dev;
    state->bus_id = config->bus_id;

    struct device_cfg_reg *reg =
        device_config_get_register_by_name(config, "core");
    ASSERT(reg);

    struct device_cfg_clk *mclk_asrc =
        device_config_get_clk_by_name(config, "core");
    ASSERT(mclk_asrc);

    devcfg_set_clock(mclk_asrc);

    struct device_cfg_irq *irq =
                        device_config_get_irq_by_name(config, "core");
    ASSERT(irq);

    /* DMA properties */
    state->dma_enable = true;
    if (!!of_device_get_bool(dev, "disable-dma")) {
        state->dma_enable = false;
    }
    printlk(LK_INFO, "%s: asrc%d: dma is %sabled\n", __func__, state->bus_id,
            state->dma_enable ? "en" : "dis");

    if (state->dma_enable) {
        /**** IN ****/
        unsigned dma_rx_period_size_max = ASRC_DMA_RX_MAX_PERIOD;
        unsigned dma_rx_nr_periods;

        event_init(&state->dma_rx_completion_event, false, EVENT_FLAG_AUTOUNSIGNAL);
        sem_init(&state->dma_rx_period_sem, 0);

        state->dma_rx_period_size_max = dma_rx_period_size_max;

        if (of_device_get_int32(dev, "dma-rx-nr-period", &dma_rx_nr_periods))
            dma_rx_nr_periods = IMX_ASRC_DMA_NR_PERIOD;
        state->dma_rx_nr_periods = dma_rx_nr_periods;

        printlk(LK_INFO, "%s:%d: RX: Using %d period buffers of length %d bytes\n",
                __PRETTY_FUNCTION__, __LINE__,
                state->dma_rx_nr_periods,
                state->dma_rx_period_size_max);

        /**** OUT ****/
        unsigned dma_tx_period_size_max = ASRC_DMA_TX_MAX_PERIOD;
        unsigned dma_tx_nr_periods;

        event_init(&state->dma_tx_completion_event, false, EVENT_FLAG_AUTOUNSIGNAL);
        sem_init(&state->dma_tx_period_sem, 0);

        state->dma_tx_period_size_max = dma_tx_period_size_max;

        if (of_device_get_int32(dev, "dma-tx-nr-period", &dma_tx_nr_periods))
            dma_tx_nr_periods = IMX_ASRC_DMA_NR_PERIOD;
        state->dma_tx_nr_periods = dma_tx_nr_periods;

        printlk(LK_INFO, "%s:%d: TX: Using %d period buffers of length %d bytes\n",
                __PRETTY_FUNCTION__, __LINE__,
                state->dma_tx_nr_periods,
                state->dma_tx_period_size_max);

        state->dma_tx_buffer_size = ASRC_DMA_OUT_CBUF_SIZE;

        const char* buffer_name = "asrc_out_cbuf";
        void **dma_buffer = &state->dma_tx_cbuf_buffer;

        status_t ret_alloc;
        ret_alloc = vmm_alloc_contiguous(vmm_get_kernel_aspace(),
                                            buffer_name,
                                            state->dma_tx_buffer_size,
                                            dma_buffer,
                                            8 /* log2_align */,
                                            0 /* vmm flags */,
                                            ARCH_MMU_FLAG_CACHED);
        printlk(LK_VERBOSE, "%s: vmm_alloc_contig returns %d, ptr %p, 0x%lx\n",
            buffer_name,
            ret_alloc, *dma_buffer,
            vaddr_to_paddr((void *)*dma_buffer));
        if (ret_alloc < 0) {
            printlk(LK_ERR, "%s:%d: failed to allocate memory for %s.\n",
                    __PRETTY_FUNCTION__, __LINE__, buffer_name);
            ASSERT(0);
        }

        ASSERT(state->dma_tx_cbuf_buffer);

        cbuf_initialize_etc(&state->dma_tx_cbuf, state->dma_tx_buffer_size, state->dma_tx_cbuf_buffer);
        cbuf_set_flag(&state->dma_tx_cbuf, CBUF_FLAG_BUF_IS_CACHEABLE);
        cbuf_clear_flag(&state->dma_tx_cbuf, CBUF_FLAG_SW_IS_WRITER);
        cbuf_set_flag(&state->dma_tx_cbuf, CBUF_FLAG_SW_IS_READER);

        /* create SDMA workers */
        state->dma_tx_thread = thread_create(
            "asrc_tx dma worker",
            imx_asrc_tx_dma_worker_thread, state,
            HIGH_PRIORITY, DEFAULT_STACK_SIZE);
        ASSERT(state->dma_tx_thread);
        thread_set_pinned_cpu(state->dma_tx_thread, -1);
        thread_detach_and_resume(state->dma_tx_thread);
    }

    state->clk_rate = mclk_asrc->rate;
    state->io_base = (ASRC_Type *) reg->vbase;

    ASRC_Init(state->io_base);
    imx_init_hw_state(&state->hw_state);
    imx_asrc_set_hw_state(state, kASRC_Up);

    /* Initialize RUN_STOP event */
    event_init(&state->runstop_completion_event, false, EVENT_FLAG_AUTOUNSIGNAL);

    /* interrupts */
    printlk(LK_INFO, "%s:%d: Register and unmask IRQ %u\n",
            __PRETTY_FUNCTION__,
            __LINE__, irq->irq);
    register_int_handler(irq->irq, imx_asrc_isr, dev);
    unmask_interrupt(irq->irq);

    /* Enable all irq sources */
    ASRC_ClearInterruptStatus(state->io_base,kASRC_ContextAllInterruptStatus);
    ASRC_EnableInterrupt(state->io_base,kASRC_ContextAllInterruptStatus);

    ASRC_Dump(state->io_base);

    return NO_ERROR;
}

/* Open */
static status_t imx_asrc_open(struct device *dev)
{
    status_t ret = NO_ERROR;

    printlk(LK_VERBOSE, "%s: entry\n", __func__);

    struct imx_asrc_state *state = dev->state;
    ASSERT(state);

    ret = imx_asrc_check_state(state, kASRC_Up, "opening");
    if (ret)
        return ret;

    if (imx_asrc_is_hw_state(state, kASRC_Prepared))
        return 0;

    state->dynamic_ratio = 0.0;
    ASRC_SetResamplingRatioModifier(state->io_base, state->context, 0);

    imx_asrc_set_hw_state(state, kASRC_Prepared);

    return ret;
}

/* Close */
static status_t imx_asrc_close(struct device *dev)
{
    printlk(LK_VERBOSE, "%s: entry\n", __func__);
    status_t ret = NO_ERROR;

    struct imx_asrc_state *state = dev->state;
    ASSERT(state);

    ASRC_Type *base = state->io_base;
    ASSERT(base);

    ret = imx_asrc_check_state(state, kASRC_Prepared, "closing");
    if (ret)
        return ret;

    ASRC_DisableContextSlots(base, state->context);

    imx_asrc_clear_hw_state(state, kASRC_Prepared);

    return ret;
}

/* Start */
static status_t imx_asrc_start(struct device *dev,
                               asrc_transfer_params_t *transfer_params,
                               asrc_out_params_t *out_params)
{
    printlk(LK_VERBOSE, "%s: entry\n", __func__);
    status_t ret = NO_ERROR;

    struct imx_asrc_state *state = dev->state;
    ASSERT(state);

    ret = imx_asrc_check_state(state, kASRC_Ready, "starting");
    if (ret)
        return ret;

    ASRC_Type *base = state->io_base;
    ASSERT(base);

    uint32_t *outbuf = (uint32_t*) transfer_params->out_buffer;
    uint32_t produced_tx_bytes = 0;

    asrc_transfer_t xfer;
    xfer.inDataAddr = (uint32_t*) transfer_params->in_buffer;
    xfer.inDataSize  = transfer_params->in_size_bytes;
    xfer.outDataAddr = outbuf;
    xfer.outDataSize = 0; /* zero-init output generated buffer size */

    /* enable context run */
    if (state->first_start) {
        printlk(LK_DEBUG, "%s: Enable ASRC context %u\n", __func__, (unsigned)state->context);
        ASRC_EnableContextRun(base, state->context, true);
        ASRC_EnableContextRunStop(base, state->context, false);
        state->first_start = false;

        if (state->dma_enable) {
            /* DMA-managed version : start output DMA */
            ret = imx_asrc_dma_start(dev, false, (unsigned)state->context, &xfer);
            if (ret) {
                printlk(LK_ERR, "%s: Failed to start DMA...\n", __func__);
                return ret;
            }

            /* Enable ASRC DMA output requests for this context */
            ASRC_EnableContextOutDMA(state->io_base, state->context, true);
        }
    }

    /* Set state to On */
    imx_asrc_set_hw_state(state, kASRC_On);

    printlk(LK_DEBUG, "%s: ASRC start\n", __func__);
    printlk(LK_DEBUG, "%s:     in  ==> %u\n", __func__, xfer.inDataSize);

    if (state->dma_enable) {

        if (xfer.inDataSize < ASRC_MIN_XFER_SIZE) {
            /* nothing for now */
            printlk(LK_DEBUG,
                    "%s: ASRC CPU transfer started for %u input bytes. Waiting for completion...\n",
                    __func__, transfer_params->in_size_bytes);

            /* Push data using CPU (not enough payload provided to use DMA buffers) */
            ASRC_PushBlocking(base, state->context, &xfer);
        } else {
            /* DMA-managed version */
            ret = imx_asrc_dma_start(dev, true, state->context, &xfer);
            if (ret) {
                printlk(LK_ERR, "%s: Failed to start DMA...\n", __func__);
                return ret;
            }

            printlk(LK_DEBUG,
                    "%s: ASRC DMA started for %u input bytes. Waiting for completion...\n",
                    __func__, transfer_params->in_size_bytes);

            /* Enable ASRC DMA input requests for this context */
            ASRC_EnableContextInDMA(state->io_base, state->context, true);

            /* Wait for DMA completion */
            event_wait(&state->dma_rx_completion_event);

            /* Free rx dma descriptor */
            free(state->rx_dma_desc);

            /* Disable ASRC DMA input requests for this context */
            ASRC_EnableContextInDMA(state->io_base, state->context, false);
        }

        size_t size_avail = cbuf_space_used(&state->dma_tx_cbuf);
        printlk(LK_DEBUG,
                "%s: ASRC stopped DMA after producing %lu bytes.\n",
                __func__, size_avail);

        /* Consume output cbuf data */
        size_t size_consumed = cbuf_read(&state->dma_tx_cbuf, outbuf, size_avail, false);

        /* keep track of output size */
        produced_tx_bytes += size_consumed;

        /* update output buffer address */
        outbuf = (uint32_t *)((uint8_t *) transfer_params->out_buffer + produced_tx_bytes);
    } else {
        printlk(LK_DEBUG, "%s: CPU transfer : inDataAddr=%p, inDataSize=%u, outDataAddr=%p, outDataSize=%u\n",
                __func__, xfer.inDataAddr, xfer.inDataSize, xfer.outDataAddr, xfer.outDataSize);

        /* 100% CPU-managed version */
        ASRC_TransferBlocking(base, state->context, &xfer);

        printlk(LK_DEBUG, "%s: CPU transfer : returned %u bytes from ASRC\n", __func__, xfer.outDataSize);
    }

    /* keep track of output size */
    produced_tx_bytes += xfer.outDataSize;

    /* update output buffer address */
    outbuf = (uint32_t *)((uint8_t *) transfer_params->out_buffer + produced_tx_bytes);

    printlk(LK_VERBOSE, "%s: ASRC read output returned %u samples (and %u interrupts)\n", __func__, produced_tx_bytes, state->irq_stats);

    printlk(LK_DEBUG, "%s:     out ==> %u\n", __func__, produced_tx_bytes);

    /* Return output buffer size */
    out_params->out_size_bytes = produced_tx_bytes;

    return ret;
}

/* Flush - optional */
static status_t imx_asrc_flush(struct device *dev,
                               asrc_flush_params_t *flush_params,
                               asrc_out_params_t *out_params)
{
    printlk(LK_VERBOSE, "%s: entry\n", __func__);
    status_t ret = NO_ERROR;

    struct imx_asrc_state *state = dev->state;
    ASSERT(state);

    ASRC_Type *base = state->io_base;
    ASSERT(base);

    uint32_t *outbuf = (uint32_t*) flush_params->out_buffer;
    uint32_t produced_tx_bytes = 0;
    uint32_t sampleCount = FSL_ASRC_OUTPUT_FIFO_DEPTH * ASRC_MAX_SUPPORTED_CHANNELS;

    /* Enable RUN_STOP mode */
    ASRC_EnableContextRunStop(base, state->context, true);

    /* Flush until RUN_STOP completion */
    do {
        /* Flush output FIFO */
        uint32_t rd_samples = ASRC_ReadFIFORemainedSample(state->io_base, state->context, outbuf,
               state->data_width,
               sampleCount);

        rd_samples *= state->data_width;

        /* Keep track of output size */
        produced_tx_bytes += rd_samples;

        /* Update output buffer address */
        outbuf = (uint32_t *)((uint8_t *) flush_params->out_buffer + produced_tx_bytes);
    } while (NO_ERROR != event_wait_timeout(&state->runstop_completion_event, 10));

    out_params->out_size_bytes = produced_tx_bytes;

    /* Set state to Flushed */
    imx_asrc_set_hw_state(state, kASRC_Flushed);

    return ret;
}

/* Stop */
static status_t imx_asrc_stop(struct device *dev)
{
    printlk(LK_VERBOSE, "%s: entry\n", __func__);
    status_t ret = NO_ERROR;

    struct imx_asrc_state *state = dev->state;
    ASSERT(state);

    ret = imx_asrc_check_state(state, kASRC_On, "stopping");
    if (ret)
        return ret;

    ASRC_Type *base = state->io_base;
    ASSERT(base);

    if (state->dma_enable) {
        /* Disable ASRC DMA output requests for this context */
        ASRC_EnableContextOutDMA(state->io_base, state->context, false);

        /* Dirty fix : need this arbitrary delay to ensure that SDMA is
         * not in the middle of a transfer while stopping and deallocating the channel.
         * We have NO synchronization primitive between SDMA and CPU for such case.
         */
        thread_sleep((lk_time_t)1);

        /* Stop and deallocate DMA resources */
        imx_asrc_dma_stop(dev, state->context);
    }

    /* Drain any remaining samples from FIFO is flush has not been done */
    if (!imx_asrc_is_hw_state(state, kASRC_Flushed)) {
        /* Enable RUN_STOP mode */
        ASRC_EnableContextRunStop(base, state->context, true);

        /* Drop until RUN_STOP completion */
        do {
            /* Drop samples from output FIFO */
            ASRC_DiscardFIFORemainedSample(state->io_base, state->context);
        } while (NO_ERROR != event_wait_timeout(&state->runstop_completion_event, 10));
    }

    printlk(LK_DEBUG, "%s: Disable ASRC context %u\n", __func__, (unsigned)state->context);
    ASRC_EnableContextRun(base, state->context, false);
    ASRC_EnableContextRunStop(base, state->context, false);

    state->first_start = true;

    imx_asrc_clear_hw_state(state, kASRC_On | kASRC_Flushed);

    return ret;
}

/* Setup */
static status_t imx_asrc_setup(struct device *dev, asrc_audio_params_t *fmt)
{
    printlk(LK_VERBOSE, "%s: entry\n", __func__);
    status_t ret = NO_ERROR;

    struct imx_asrc_state *state = dev->state;
    ASSERT(state);

    ret = imx_asrc_check_state(state, kASRC_Prepared, "configuring");
    if (ret)
        return ret;

    state->context = kASRC_Context0;    // Context 0
    state->data_width = 4;              // 32 bits supports only
    state->first_start = true;

    asrc_context_config_t *context_config = &state->context_config;
    ASRC_GetContextDefaultConfig(context_config,
                                fmt->num_channels,
                                fmt->in_sample_rate,
                                fmt->out_sample_rate);
    ASRC_Update_dataType_Config(context_config,
                                fmt->data_type,
                                fmt->data_type);

    state->resampler_fract_bits = ASRC_GetResampleFractBits(context_config);

    /* Force output fifo watermark to half of its size (default is size/8) */
    context_config->contextOutput.watermark = FSL_ASRC_OUTPUT_FIFO_DEPTH / 2U;

    ret = ASRC_SetContextConfig(state->io_base, state->context, context_config);
    if (ret)
        return ret;

    printlk(LK_DEBUG, "%s: num_channels    = %u\n", __func__, fmt->num_channels);
    printlk(LK_DEBUG, "%s: in_sample_rate  = %u\n", __func__, fmt->in_sample_rate);
    printlk(LK_DEBUG, "%s: out_sample_rate = %u\n", __func__, fmt->out_sample_rate);
    printlk(LK_DEBUG, "%s: data_type       = %u\n", __func__, (unsigned)fmt->data_type);

    ASRC_Dump(state->io_base);

    /* DMA */
    if (state->dma_enable) {
        imx_asrc_dma_setup(dev, true /* in */, state->context);
        imx_asrc_dma_setup(dev, false /* out */, state->context);
    }

    imx_asrc_set_hw_state(state, kASRC_Ready);

    return ret;
}

static status_t imx_asrc_dynamic_ratio( struct device *dev,
                                        struct asrc_ioc_cmd_ratio *ratio,
                                        bool is_read)
{
    struct imx_asrc_state *state = dev->state;

    if (is_read)
        ratio->dynamic = state->dynamic_ratio;
    else {
        state->dynamic_ratio = ratio->dynamic;

        // Scale from double value to correct resampler format (either 5.39, 6.38 or 7.37)
        double ratio_scaled = state->dynamic_ratio * (1ULL << state->resampler_fract_bits);

        // Clip to 32-bit two's complement signed, as resampling ratio updates are added to the lowest
        // 31-bits of the 44-bit resampling ratio
        int32_t ratio_update;
        if (ratio_scaled > INT_MAX)
            ratio_update = INT_MAX;
        else if (ratio_scaled < INT_MIN)
            ratio_update = INT_MIN;
        else
            ratio_update = (int32_t) ratio_scaled;

        ASRC_SetResamplingRatioModifier(state->io_base, state->context, ratio_update);
    }

    return 0;
}

/* Ioctl */
static status_t imx_asrc_ioctl(struct device *dev, int request, void *argp)
{
    status_t ret;

    if (!dev)
        return ERR_NOT_FOUND;

    if ((IOCGROUP(request) != IOCTL_DEV_ASRC) || (!argp))
        return ERR_INVALID_ARGS;

    int cmd = IOCBASECMD(request);

    switch(request) {
    case ASRC_IOC_DYNAMIC_RATIO_GET:
        ret = imx_asrc_dynamic_ratio(dev, (struct asrc_ioc_cmd_ratio *) argp, true);
        break;

    case ASRC_IOC_DYNAMIC_RATIO_SET:
        ret = imx_asrc_dynamic_ratio(dev, (struct asrc_ioc_cmd_ratio *) argp, false);
        break;

    default:
        printlk(LK_ERR, "Invalid asrc ioctl(%d)\n", cmd);
        ret = ERR_NOT_FOUND;
    }

    return ret;
}

static struct device_class asrc_device_class = {
    .name = "asrc",
};

static struct asrc_ops the_ops = {
    .std = {
        .device_class = &asrc_device_class,
        .init = imx_asrc_init,
        .ioctl = imx_asrc_ioctl,
    },
    .open = imx_asrc_open,
    .close = imx_asrc_close,
    .setup = imx_asrc_setup,
    .start = imx_asrc_start,
    .flush = imx_asrc_flush,
    .stop = imx_asrc_stop,
};

DRIVER_EXPORT(asrc, &the_ops.std);
