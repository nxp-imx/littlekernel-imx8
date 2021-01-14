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
#include <dev/class/pdm.h>
#include <platform.h>
#include <sys/ioctl.h>
#include <dev/pdm_ioctl.h>
#include "pdm_hw.h"

#include <kernel/spinlock.h>
#include <kernel/semaphore.h>
#include <kernel/event.h>

#include <lib/appargs.h>
#include <dev/interrupt.h>
#include <lib/cbuf.h>
#include <assert.h>
#include <err.h>
#include <debug.h>


#undef IMX_PDM_PARANOID

/* Currently hardcoded in the HW macro */
#define IMX_PDM_FIFO_STAGES 4
#define PDM_MAX_CHANNELS 8
#define PDM_FIFO_WATERMARK 4
#define PDM_CIC_OVERSAMPLE_RATE 0
#define PDM_SAMPLE_CLOCK_RATE (1536000U * 2U) /* PDM_CLK will vary depending on Quality Mode and OSR*/

//ALEX TODO enable IRQ errors
enum pdm_hw_state {
    kPDM_Down         = (1UL << 0),   /*!< PDM is powered down. */
    kPDM_Up           = (1UL << 1),   /*!< PDM is powered up. */
    kPDM_Prepared     = (1UL << 2),   /*!< PDM is prepared  */
    kPDM_Ready        = (1UL << 3),   /*!< PDM is prepared and ready. */
    kPDM_On           = (1UL << 4),   /*!< PDM is on. IRQ flowing */
    kPDM_Busy         = (1UL << 5),   /*!< PDM is busy, transfer on going */
    kPDM_Done         = (1UL << 6),   /*!< Transfer is done. */
    kPDM_Error        = (1UL << 7),   /*!< Transfer error occurred. */
    kPDM_Running      = kPDM_On,
};

static inline void imx_pdm_set_hw_state(pdm_handle_t *handle, uint32_t state)
{
    handle->state |= state;
    smp_wmb();
}

static inline void imx_pdm_clear_hw_state(pdm_handle_t *handle, uint32_t state)
{
    handle->state &= ~state;
    smp_wmb();
}

struct imx_pdm_state {
    int bus_id;
    PDM_Type *io_base;
    struct device *device;
    struct list_node node;
    /* 4MB as max circular buffer size */
#define PDM_MAX_RX_BUF_SIZE (4 * 1024 * 1024)
    cbuf_t rx_circ_buf;
    size_t rx_circ_buf_size;
    spin_lock_t rx_lock;
    mutex_t rx_mutex;
    event_t rx_wait;
    unsigned clk_rate;
    /* Interface with PDM HW */
    pdm_handle_t rx_handle;
    pdm_config_t hw_config;
    pdm_channel_config_t channel_config;
};

static inline bool imx_pdm_is_hw_state(pdm_handle_t *handle,
                                       enum _pdm_status expected_state)
{
    return ((handle->state & expected_state) == expected_state);
}

static status_t _imx_pdm_check_state(
    pdm_handle_t *handle,
    enum _pdm_status expected_state,
    const char *msg,
    bool strict)
{
    bool cond;
    if (strict)
        cond = (handle->state == expected_state);
    else
        cond = imx_pdm_is_hw_state(handle, expected_state);
#ifdef IMX_PDM_PARANOID
    ASSERT(cond);
#else
    if (!cond) {
        printlk(LK_INFO, "%s:%d: Wrong state (%x) while %s the interface\n",
                __PRETTY_FUNCTION__, __LINE__, handle->state, msg);
        return ERR_BAD_STATE;
    }
#endif
    return 0;
}

static inline status_t imx_pdm_check_state(
    pdm_handle_t *handle,
    uint32_t expected_state,
    const char *msg)
{
    return _imx_pdm_check_state(handle, expected_state, msg, false);
}

static inline status_t imx_pdm_check_strict_state(
    pdm_handle_t *handle,
    uint32_t expected_state,
    const char *msg)
{
    return _imx_pdm_check_state(handle, expected_state, msg, true);
}

void PDM_RxStart(PDM_Type *base, uint32_t mask)
{
    /* Remove previous pending status flags */
    if (mask)
        PDM_ClearStatus(base, mask);
    /* Enable Interrupts */
    PDM_EnableInterrupts(base, mask);
    /* Enable Rx transfer */
    PDM_Enable(base, true);
}

void PDM_RxStop(PDM_Type *base, uint32_t mask)
{
    PDM_DisableInterrupts(base, mask);
    PDM_Enable(base, false);
    if (mask)
        PDM_ClearStatus(base, mask);
}

static struct list_node imx_pdm_list = LIST_INITIAL_VALUE(imx_pdm_list);
static spin_lock_t imx_pdm_list_lock = SPIN_LOCK_INITIAL_VALUE;

static enum handler_return imx_pdm_isr_error(void *arg)
{
    struct device *dev = (struct device *)arg;
    ASSERT(dev);

    struct imx_pdm_state *state = dev->state;
    ASSERT(state);

    PDM_Type *base = state->io_base;
    ASSERT(base);

    printlk(LK_ERR, "%s:%d: overflow/underflow error on pdm\n",
            __PRETTY_FUNCTION__, __LINE__);
    spin_lock(&state->rx_lock);
    PDM_ClearFIFOStatus(base, 0x00FF00FF);
    spin_unlock(&state->rx_lock);

    return INT_RESCHEDULE;
}
static bool imx_pdm_isr_rx(struct imx_pdm_state *state)
{
    bool reschedule = false;
    uint32_t stat;
    PDM_Type *base = state->io_base;
    ASSERT(base);

    pdm_handle_t *handle = &state->rx_handle;

    spin_lock(&state->rx_lock);
    stat = PDM_GetStatus(base);

    if (stat & kPDM_EnableChannelAll) {
        size_t written = 0;
        size_t xfer_len = handle->watermark * handle->channelNums * 2;
        int16_t *buffer = handle->buf;

        PDM_ReadNonBlocking(base,
                            handle->startChannel,
                            handle->channelNums,
                            buffer,
                            handle->watermark);
        PDM_ClearStatus(base, stat & kPDM_EnableChannelAll);

        if (xfer_len && (imx_pdm_is_hw_state(handle, kPDM_Running))) {
            size_t avail = cbuf_space_avail(&state->rx_circ_buf);
            /* Push the data to circular buffer */
            printlk(LK_VERBOSE, "hw state: (%x), Pushing %ld to rx circular buffer\n",
                          handle->state, xfer_len);
            if (xfer_len > avail)
                printlk(LK_NOTICE, "%s:%d: No more room in pdm circ buffer (avail %ld). Dropping %ld bytes\n",
                       __PRETTY_FUNCTION__, __LINE__, avail, xfer_len);
            else {
                written = cbuf_write(&state->rx_circ_buf, handle->buf, xfer_len, false);
                DEBUG_ASSERT(written == xfer_len);
            }

            /* Wake up thread if transfer on going */
            if (imx_pdm_is_hw_state(handle, kPDM_Busy)
                    && (handle->xfer_remaining > 0)) {
                handle->xfer_remaining -= written;
                if ((handle->xfer_remaining <= 0) ||
                        (cbuf_space_used(&state->rx_circ_buf) > handle->cbuf_watermark)) {
                    handle->xfer_remaining = MAX(0, handle->xfer_remaining);
                    event_signal(&state->rx_wait, false);
                    reschedule = true;
                }
                imx_pdm_set_hw_state(handle, kPDM_Done);
            }
        }
    }

exit_rx_isr:
    smp_wmb();
    spin_unlock(&state->rx_lock);

    return reschedule;
}

static enum handler_return imx_pdm_isr(void *arg)
{
    struct device *dev = (struct device *)arg;
    ASSERT(dev);

    struct imx_pdm_state *state = dev->state;
    ASSERT(state);

    bool reschedule = false;

    if (imx_pdm_isr_rx(state))
        reschedule = true;

    if (reschedule)
        return INT_RESCHEDULE;
    else
        return INT_NO_RESCHEDULE;
}

static status_t imx_pdm_init(struct device *dev)
{
    const struct device_config_data *config = dev->config;
    struct imx_pdm_state *state;
    pdm_config_t *hw_config;
    pdm_channel_config_t *channel_config;

    printlk(LK_VERBOSE, "%s: entry\n", __func__);
    state = malloc(sizeof(struct imx_pdm_state));
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

    struct device_cfg_irq *irq_error =
        device_config_get_irq_by_name(config, "error");
    ASSERT(irq_error);
    spin_lock_init(&state->rx_lock);
    mutex_init(&state->rx_mutex);
    event_init(&state->rx_wait, false, EVENT_FLAG_AUTOUNSIGNAL);

    struct device_cfg_clk *mclk_pdm =
        device_config_get_clk_by_name(config, "mclk-pdm");
    ASSERT(mclk_pdm);

    devcfg_set_clock(mclk_pdm);
    state->clk_rate = mclk_pdm->rate;
    devcfg_set_pins_by_name(config->pins_cfg,
                            config->pins_cfg_cnt, "default");

    state->io_base = (PDM_Type *) reg->vbase;

    memset(&state->rx_handle, 0, sizeof(pdm_handle_t));
    pdm_handle_t *handle = &state->rx_handle;

    hw_config = &state->hw_config;
    hw_config->enableDoze = false;
    hw_config->fifoWatermark = PDM_FIFO_WATERMARK - 1;
    hw_config->qualityMode = kPDM_QualityModeVeryLow0;
    hw_config->cicOverSampleRate = PDM_CIC_OVERSAMPLE_RATE;

    channel_config = &state->channel_config;
    channel_config->cutOffFreq = kPDM_DcRemoverBypass;
    channel_config->gain = kPDM_DfOutputGain2;

    PDM_Init(state->io_base, hw_config);
    handle->state = kPDM_Up;
    handle->watermark = (state->io_base->FIFO_CTRL & PDM_FIFO_CTRL_FIFOWMK_MASK) +  1;
    printlk(LK_VERBOSE, "FIFO Watermark: (%d)\n", handle->watermark);
    PDM_Reset(state->io_base);
    register_int_handler(irq->irq, imx_pdm_isr, dev);

    register_int_handler(irq_error->irq, imx_pdm_isr_error, dev);
    handle->irq_mask = kPDM_FIFOInterruptEnable;

    PDM_RxStop(state->io_base, kPDM_FIFOInterruptEnable | kPDM_ErrorInterruptEnable);

    spin_lock(&imx_pdm_list_lock);
    list_add_tail(&imx_pdm_list, &state->node);
    spin_unlock(&imx_pdm_list_lock);

    PDM_Dump(state->io_base);

    return 0;
}

static status_t imx_pdm_rx_open(struct device *dev)
{
    printlk(LK_VERBOSE, "%s: entry\n", __func__);
    status_t ret;

    struct imx_pdm_state *state = dev->state;
    ASSERT(state);

    pdm_handle_t *handle = &state->rx_handle;

    ret = imx_pdm_check_state(handle, kPDM_Up, "opening");
    if (ret)
        return ret;

    if (handle->state & kPDM_Prepared)
        return 0;

    /* This likely means a memory leak if buf is not NULL */
    ASSERT(handle->buf == NULL);
    ASSERT(state->rx_circ_buf.buf == NULL);

    handle->buf = malloc(IMX_PDM_FIFO_STAGES * PDM_MAX_CHANNELS * sizeof(int16_t));
    ASSERT(handle->buf);

    state->rx_circ_buf.buf = malloc(PDM_MAX_RX_BUF_SIZE);
    ASSERT(state->rx_circ_buf.buf);

    cbuf_initialize_etc(&state->rx_circ_buf,
                        PDM_MAX_RX_BUF_SIZE, state->rx_circ_buf.buf);

    imx_pdm_set_hw_state(handle, kPDM_Prepared);

    struct device_cfg_irq *irq =
                        device_config_get_irq_by_name(dev->config, "core");
    ASSERT(irq);

    unmask_interrupt(irq->irq);

    struct device_cfg_irq *irq_error =
                        device_config_get_irq_by_name(dev->config, "error");
    ASSERT(irq_error);

    unmask_interrupt(irq_error->irq);

    return 0;
}

static status_t imx_pdm_rx_close(struct device *dev)
{
    printlk(LK_VERBOSE, "%s: entry\n", __func__);
    status_t ret;

    struct imx_pdm_state *state = dev->state;
    ASSERT(state);

    pdm_handle_t *handle = &state->rx_handle;

    ret = imx_pdm_check_state(handle, kPDM_Prepared, "closing");
    if (ret)
        return ret;

    ASSERT(handle->buf != NULL);
    ASSERT(state->rx_circ_buf.buf != NULL);

    free(handle->buf);
    free(state->rx_circ_buf.buf);

    handle->buf = NULL;
    state->rx_circ_buf.buf = NULL;

    imx_pdm_clear_hw_state(handle, kPDM_Prepared);
    struct device_cfg_irq *irq =
                        device_config_get_irq_by_name(dev->config, "core");
    ASSERT(irq);

    mask_interrupt(irq->irq);

    struct device_cfg_irq *irq_error =
                        device_config_get_irq_by_name(dev->config, "error");

    ASSERT(irq_error);

    mask_interrupt(irq_error->irq);

    return 0;
}

static status_t imx_pdm_rx_start(struct device *dev)
{
    printlk(LK_VERBOSE, "%s: entry\n", __func__);
    status_t ret;

    struct imx_pdm_state *state = dev->state;
    ASSERT(state);
    pdm_handle_t *handle = &state->rx_handle;

    if (handle->state & kPDM_On)
        return 0;

    ret = imx_pdm_check_state(handle, kPDM_Ready, "starting");
    if (ret)
        return ret;

    PDM_Type *base = state->io_base;
    ASSERT(base);

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->rx_lock, lock_state);

    /* Set state to On */
    imx_pdm_set_hw_state(handle, kPDM_On);

    handle->ts_start = current_time_hires();
    PDM_RxStart(base, handle->irq_mask);

    spin_unlock_irqrestore(&state->rx_lock, lock_state);
    //PDM_Dump(state->io_base);
    return 0;
}

static status_t imx_pdm_rx_stop(struct device *dev)
{
    printlk(LK_VERBOSE, "%s: entry\n", __func__);
    status_t ret;

    struct imx_pdm_state *state = dev->state;
    ASSERT(state);

    pdm_handle_t *handle = &state->rx_handle;

    ret = imx_pdm_check_state(handle, kPDM_On, "stopping");
    if (ret)
        return ret;

    PDM_Type *base = state->io_base;
    ASSERT(base);

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->rx_lock, lock_state);
    handle->xfer_remaining = ERR_IO;

    handle->ts_stop = current_time_hires();
    PDM_RxStop(base, handle->irq_mask);
    imx_pdm_clear_hw_state(handle, kPDM_On);

    /* TODO: abort any on going transfer */
    spin_unlock_irqrestore(&state->rx_lock, lock_state);
    event_signal(&state->rx_wait, false);

    return 0;
}

static status_t imx_pdm_rx_setup(struct device *dev, pdm_hw_params_t *fmt)
{
    int i, sample_chan_mask;

    printlk(LK_VERBOSE, "%s: entry\n", __func__);
    status_t ret = 0;

    struct imx_pdm_state *state = dev->state;
    ASSERT(state);

    pdm_handle_t *handle = &state->rx_handle;
    pdm_config_t *hw_config = &state->hw_config;

    ret = imx_pdm_check_state(handle, kPDM_Prepared, "configuring");
    if (ret)
        return ret;

    PDM_Type *base = state->io_base;
    ASSERT(base);

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->rx_lock, lock_state);

    handle->startChannel = fmt->start_channel;
    handle->channelNums = fmt->num_channels;
    ASSERT((handle->startChannel + handle->channelNums) <= FSL_FEATURE_PDM_CHANNEL_NUM);
    printlk(LK_VERBOSE, "start channel: %d\n", handle->startChannel);
    printlk(LK_VERBOSE, "num channels: %d\n", handle->channelNums);
    sample_chan_mask = 0;
    for (i = handle->startChannel; i < (handle->channelNums + handle->startChannel); i++) {
        PDM_SetChannelConfig(state->io_base, i, &state->channel_config);
        sample_chan_mask |= (1 << i);

    }
    ret = PDM_SetSampleRate(state->io_base, sample_chan_mask, hw_config->qualityMode, hw_config->cicOverSampleRate, (state->clk_rate / PDM_SAMPLE_CLOCK_RATE));
    imx_pdm_set_hw_state(handle, kPDM_Ready);

exit_rx_setup:
    spin_unlock_irqrestore(&state->rx_lock, lock_state);

    return ret;
}

static status_t imx_pdm_read(struct device *dev, void *buf, size_t len)
{
    printlk(LK_VERBOSE, "%s: entry\n", __func__);

    size_t remaining, read, used;
    void *xfer;
    int ret = 0;

    struct imx_pdm_state *state = dev->state;
    ASSERT(state);

    pdm_handle_t *handle = &state->rx_handle;

    PDM_Type *base = state->io_base;
    ASSERT(base);

    if (!(handle->state & kPDM_Ready))
        return ERR_NOT_READY;

    if (handle->state & kPDM_Busy)
        return ERR_BUSY;

    mutex_acquire(&state->rx_mutex);

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->rx_lock, lock_state);

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

    printlk(LK_VERBOSE, "len: %ld\n", len);

    ASSERT(handle->state & kPDM_On);

    imx_pdm_set_hw_state(handle, kPDM_Busy);

    while (remaining) {
        read = cbuf_read(&state->rx_circ_buf, xfer, remaining, false);
        remaining -= read;
        xfer += read;

        if (remaining)
            ret = event_wait_timeout(&state->rx_wait, 100); /* 100 msecond timeout */

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
        printlk(LK_VERBOSE, "pdm cbuf reset\n");
        cbuf_reset(&state->rx_circ_buf);
    }

    handle->xfer_remaining = 0;

    imx_pdm_clear_hw_state(handle, kPDM_Busy | kPDM_Done | kPDM_Error);

    smp_wmb();
#if ((defined IMX_PDM_READ_SILENT_RETRY) && (IMX_PDM_READ_SILENT_RETRY == 1))
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

    return 0;
}

static status_t imx_pdm_get_timestamp(struct device *dev,
                                        struct pdm_ioc_cmd_timestamp *ts)
{
    struct imx_pdm_state *state = dev->state;

    ts->start = state->rx_handle.ts_start;
    ts->stop = state->rx_handle.ts_stop;

    return 0;
}

static status_t imx_pdm_set_timestamp(struct device *dev,
                                        struct pdm_ioc_cmd_timestamp *ts)
{
    struct imx_pdm_state *state = dev->state;

    if ((ts->is_read == true) && (ts->start > 0))
        state->rx_handle.ts_start = ts->start;
    else
        return ERR_INVALID_ARGS;

    return 0;
}


static status_t imx_pdm_get_latency(struct device *dev,
                                        struct pdm_ioc_cmd_latency *cmd)
{
    struct imx_pdm_state *state = dev->state;
    (void)state;

    return 0;
}

static status_t imx_pdm_ioctl(struct device *dev, int request, void *argp)
{
    status_t ret;

    if (!dev)
        return ERR_NOT_FOUND;

    if ((IOCGROUP(request) != IOCTL_DEV_PDM) || (!argp))
        return ERR_INVALID_ARGS;

    int cmd = IOCBASECMD(request);

    switch(request) {

    case PDM_IOC_TS_GET:
        ret = imx_pdm_get_timestamp(dev, (struct pdm_ioc_cmd_timestamp *) argp);
        break;

    case PDM_IOC_TS_SET:
        ret = imx_pdm_set_timestamp(dev, (struct pdm_ioc_cmd_timestamp *) argp);
        break;

    case PDM_IOC_LATENCY_GET:
        ret = imx_pdm_get_latency(dev, (struct pdm_ioc_cmd_latency *) argp);
        break;

    default:
        printlk(LK_ERR, "Invalid pdm ioctl(%d)\n", cmd);
        ret = ERR_NOT_FOUND;
    }

    return ret;
}

static struct device_class pdm_device_class = {
    .name = "pdm",
};

static struct pdm_ops the_ops = {
    .std = {
        .device_class = &pdm_device_class,
        .init = imx_pdm_init,
        .ioctl = imx_pdm_ioctl,
    },
    .tx_open = NULL,
    .tx_close = NULL,
    .tx_start = NULL,
    .tx_stop = NULL,
    .tx_setup = NULL,
    .write = NULL,
    .rx_open = imx_pdm_rx_open,
    .rx_close = imx_pdm_rx_close,
    .rx_start = imx_pdm_rx_start,
    .rx_stop = imx_pdm_rx_stop,
    .rx_setup = imx_pdm_rx_setup,
    .read = imx_pdm_read,
};

DRIVER_EXPORT(pdm, &the_ops.std);

struct device *class_pdm_get_device_by_id(int id)
{
    struct device *dev = NULL;
    struct imx_pdm_state *state = NULL;

    spin_lock(&imx_pdm_list_lock);
    list_for_every_entry(&imx_pdm_list, state, struct imx_pdm_state, node) {
        if (state->bus_id == id) {
            dev = state->device;
            break;
        }
    }
    spin_unlock(&imx_pdm_list_lock);

    return dev;
}
