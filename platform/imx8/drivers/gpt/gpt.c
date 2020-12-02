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
#include <lib/appargs.h>
#include "platform/interrupts.h"
#include <dev/class/gpt.h>

#include <kernel/spinlock.h>
#include <kernel/semaphore.h>
#include <kernel/event.h>
#include "lib/dpc.h"

#include <assert.h>
#include <err.h>

#include "gpt_hw.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define PINMUX_ENABLE 1
#define POWER_ENABLE 1

struct gpt_timer_handle {
    GPT_Type *io_base;
    struct gpt_config config;
    gpt_timer_callback_t callback; /*!< Callback function called at an event*/
    void *userData;                /*!< Callback parameter passed to callback function*/
};
typedef struct gpt_timer_handle gpt_timer_handle_t;

struct gpt_capture_handle {
    GPT_Type *io_base;
    struct gpt_capture_config config;
    gpt_capture_callback_t callback; /*!< Callback function called at an event*/
    void *userData;                  /*!< Callback parameter passed to callback function*/
};
typedef struct gpt_capture_handle gpt_capture_handle_t;

struct imx_gpt_state {
    int bus_id;
    GPT_Type *io_base;
    spin_lock_t lock;
    mutex_t mutex;
    event_t wait;
    unsigned irq;
    struct device *device;
    struct list_node node;
    /* interface with GPT HW */
    struct gpt_timer_handle gpt_timer_handle;
    struct gpt_capture_handle gpt_capture_handle[GPT_ICR_COUNT];
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
static struct list_node imx_gpt_list = LIST_INITIAL_VALUE(imx_gpt_list);
static spin_lock_t imx_gpt_list_lock = SPIN_LOCK_INITIAL_VALUE;


/*******************************************************************************
 * Code
 ******************************************************************************/

/** GPT interrupt handler
 */
static enum handler_return imx_gpt_isr(void *arg)
{
    struct device *dev = (struct device *)arg;
    ASSERT(dev);
    struct imx_gpt_state *state = dev->state;
    ASSERT(state);
    bool reschedule = false;

    GPT_Type *base = state->io_base;

    ASSERT(base);
    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);
    /* A interrupt is generated in any of of the following conditions
     *
     * - Rollover Interrupt
     * - Input Capture Interrupt 1, 2
     * - Output Compare Interrupt 1, 2, 3
     *
     * */

    if (GPT_GetStatusFlags(base, kGPT_RollOverFlag)) {
        /* clear interrupt */
        GPT_ClearStatusFlags(base, kGPT_RollOverFlag);
        /* call timer callback function */
        struct gpt_timer_handle *handle = &state->gpt_timer_handle;
        if (handle->callback) {
            dpc_queue(handle->callback, handle->userData, DPC_FLAG_NORESCHED);
            reschedule = true;
        }
    }
    if (GPT_GetStatusFlags(base, kGPT_InputCapture1Flag)) {
        /* clear interrupt */
        GPT_ClearStatusFlags(base, kGPT_InputCapture1Flag);
        /* call capture callback function */
        struct gpt_capture_handle *handle = &state->gpt_capture_handle[kGPT_InputCapture_Channel1];
        if (handle->callback) {
            dpc_queue(handle->callback, handle->userData, DPC_FLAG_NORESCHED);
            reschedule = true;
        }
    }
    if (GPT_GetStatusFlags(base, kGPT_InputCapture2Flag)) {
        /* clear interrupt */
        GPT_ClearStatusFlags(base, kGPT_InputCapture2Flag);
        /* call capture callback function */
        struct gpt_capture_handle *handle = &state->gpt_capture_handle[kGPT_InputCapture_Channel2];
        if (handle->callback) {
            dpc_queue(handle->callback, handle->userData, DPC_FLAG_NORESCHED);
            reschedule = true;
        }
    }
    /* TODO add OF1/2/3 */
    printlk(LK_VERBOSE, "%s: exit\n", __PRETTY_FUNCTION__);

    /* reschedule when dpc is queued */
    if (reschedule) {
        return INT_RESCHEDULE;
    } else {
        return INT_NO_RESCHEDULE;
    }
}


/** Initialize GPT driver and select clock source
 */
static status_t imx_gpt_init(struct device *dev)
{
    const struct device_config_data *imx_config = dev->config;
    struct imx_gpt_state *state;
    status_t ret = 0;

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);
    /* sanity checks */
    ASSERT(imx_config);

    state = malloc(sizeof(struct imx_gpt_state));
    ASSERT(state);
    memset(state, 0, sizeof(*state));
    dev->state = state;
    state->device = dev;

    state->bus_id = imx_config->bus_id;

    struct device_cfg_reg *reg =
                        device_config_get_register_by_name(imx_config, "core");
    ASSERT(reg);

    struct device_cfg_irq *irq =
                        device_config_get_irq_by_name(imx_config, "core");
    ASSERT(irq);
    spin_lock_init(&state->lock);
    mutex_init(&state->mutex);
    event_init(&state->wait, false, 0);

    struct device_cfg_clk *clk_core =
                        device_config_get_clk_by_name(imx_config, "core");
    ASSERT(clk_core);

    devcfg_set_clock(clk_core);

    devcfg_set_pins_by_name(imx_config->pins_cfg,
                            imx_config->pins_cfg_cnt, "default");

    state->io_base = (GPT_Type *) reg->vbase;

    /* create and init timer handle */
    struct gpt_config config;
    GPT_GetDefaultConfig(&config);
    struct gpt_timer_handle timer_handle;
    memset(&timer_handle, 0, sizeof(timer_handle));
    state->gpt_timer_handle = timer_handle;
    timer_handle.config = config;
    GPT_Init(state->io_base, &config);
    /* clear GPT status register */
    GPT_ClearStatusFlags(state->io_base, 0xffffffff);

    /* create and init capture1 handle */
    struct gpt_capture_config capture1_config;
    capture1_config.channel = kGPT_InputCapture_Channel1;
    capture1_config.mode = kGPT_InputOperation_Disabled;
    capture1_config.irq_enable = false;
    struct gpt_capture_handle capture1_handle;
    memset(&capture1_handle, 0, sizeof(capture1_handle));
    capture1_handle.config = capture1_config;
    state->gpt_capture_handle[kGPT_InputCapture_Channel1] = capture1_handle;

    /* create and init capture2 handle */
    struct gpt_capture_config capture2_config;
    capture2_config.channel = kGPT_InputCapture_Channel2;
    capture2_config.mode = kGPT_InputOperation_Disabled;
    capture2_config.irq_enable = false;
    struct gpt_capture_handle capture2_handle;
    memset(&capture2_handle, 0, sizeof(capture2_handle));
    capture2_handle.config = capture2_config;
    state->gpt_capture_handle[kGPT_InputCapture_Channel2] = capture2_handle;

    /* TODO init compare1,2,3 */
    GPT_DisableInterrupts(state->io_base, -1);
    GPT_ClearStatusFlags(state->io_base, -1);

    register_int_handler(irq->irq, imx_gpt_isr, dev);
    state->irq = irq->irq;


    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&imx_gpt_list_lock, lock_state);
    list_add_tail(&imx_gpt_list, &state->node);
    spin_unlock_irqrestore(&imx_gpt_list_lock, lock_state);

    printlk(LK_INFO, "%s:%d: dev: %p state: %p state->dev: %p dev->state: %p\n",
            __PRETTY_FUNCTION__, __LINE__, dev, state, state->device,
            dev->state);
    printlk(LK_VERBOSE, "%s: exit\n", __PRETTY_FUNCTION__);

    return ret;
}

/** initial configuration
 *   setup GPT counter (GPTx_CR register)
 * @param dev device
 * @param cfg timer configuration
 * @param callback function to call on IRQ
 * @param userData data passed to callback function
 */
status_t gpt_open(int id, struct gpt_config *cfg, gpt_timer_callback_t callback, void *userData)
{
    struct device *dev = class_gpt_get_device_by_id(id);
    return class_gpt_open(dev, cfg, callback, userData);
}

static status_t imx_gpt_open(struct device *dev, void *cfg, gpt_timer_callback_t callback, void *userData)
{
    struct imx_gpt_state *state;
    state = dev->state;
    ASSERT(state);
    GPT_Type *base = state->io_base;
    ASSERT(base);

    struct gpt_config *pcfg = (struct gpt_config *) cfg;
    ASSERT(pcfg);
    /* save config to timer handle */
    struct gpt_timer_handle gpt_timer_handle = state->gpt_timer_handle;
    memcpy(&gpt_timer_handle.config, pcfg, sizeof(struct gpt_config));
    /* save callback function to timer handle */
    gpt_timer_handle.callback = callback;
    gpt_timer_handle.userData = userData;

    /* timer mode */
    base->CR =
        (pcfg->enableFreeRun ? GPT_CR_FRR_MASK : 0U) |
        (pcfg->enableRunInWait ? GPT_CR_WAITEN_MASK : 0U) |
        (pcfg->enableRunInStop ? GPT_CR_STOPEN_MASK : 0U) |
        (pcfg->enableRunInDoze ? GPT_CR_DOZEEN_MASK : 0U) |
        (pcfg->enableRunInDbg ? GPT_CR_DBGEN_MASK : 0U) |
        (pcfg->enableMode ? GPT_CR_ENMOD_MASK : 0U);
    /* clock source */
    GPT_SetClockSource(base, pcfg->clockSource);
    /* clock prescaler */
    GPT_SetClockDivider(base, pcfg->divider);

    unmask_interrupt(state->irq);

    return 0;
}

/** allows driver to be reconfigured
 */
status_t gpt_close(int id)
{
    struct device *dev = class_gpt_get_device_by_id(id);
    return class_gpt_close(dev);
}

static status_t imx_gpt_close(struct device *dev)
{
    struct imx_gpt_state *state;
    state = dev->state;
    ASSERT(state);
    GPT_Type *base = state->io_base;
    ASSERT(base);

    GPT_StopTimer(base);

    GPT_SetClockSource(base, kGPT_ClockSource_Off);

    mask_interrupt(state->irq);

    GPT_DisableInterrupts(state->io_base, -1);
    GPT_ClearStatusFlags(base, -1);

    return 0;
}

/** stop the counter
 */
status_t gpt_stop(int id)
{
    struct device *dev = class_gpt_get_device_by_id(id);
    return class_gpt_stop(dev);
}

static status_t imx_gpt_stop(struct device *dev)
{
    struct imx_gpt_state *state;
    state = dev->state;
    ASSERT(state);
    GPT_Type *base = state->io_base;
    ASSERT(base);

    GPT_StopTimer(base);
    /* TODO: need to disable interrupts here ? */
    return 0;
}

/** start the counter
 */
status_t gpt_start(int id)
{
    struct device *dev = class_gpt_get_device_by_id(id);
    return class_gpt_start(dev);
}

static status_t imx_gpt_start(struct device *dev)
{
    struct imx_gpt_state *state;
    state = dev->state;
    ASSERT(state);
    GPT_Type *base = state->io_base;
    ASSERT(base);

    GPT_StartTimer(base);
    return 0;
}

/** get GPT counter value
 */
status_t gpt_get_counter(int id, uint32_t *counter)
{
    struct device *dev = class_gpt_get_device_by_id(id);
    return class_gpt_get_counter(dev, counter);
}

static status_t imx_gpt_get_counter(struct device *dev, uint32_t *counter)
{
    struct imx_gpt_state *state;
    state = dev->state;
    ASSERT(state);
    GPT_Type *base = state->io_base;
    ASSERT(base);

    *counter = GPT_GetCurrentTimerCount(base);
    return 0;
}

/** setup capture (timer input register)
 * @param dev device
 * @param ch Capture channel (1 or 2)
 * @param on Enable/Disable the capture channel
 */
status_t gpt_enable_capture(int id, unsigned int ch, bool on)
{
    struct device *dev = class_gpt_get_device_by_id(id);
    return class_gpt_enable_capture(dev, ch, on);
}

static status_t imx_gpt_enable_capture(struct device *dev, unsigned int ch, bool on)
{
    struct imx_gpt_state *state;
    state = dev->state;
    ASSERT(state);
    GPT_Type *base = state->io_base;
    ASSERT(base);

    if (on) {
        struct gpt_capture_config *pcfg = &state->gpt_capture_handle[ch].config;
        if ((unsigned) ch != pcfg->channel)
            return ERR_INVALID_ARGS;

        GPT_SetInputOperationMode(base, pcfg->channel, pcfg->mode);
    } else {
        GPT_SetInputOperationMode(base, ch, kGPT_InputOperation_Disabled);
    }

    return 0;
}

/** setup capture timer
 * @param channel Capture channel number (1 or 2)
 * @param cfg Capture configuration
 * @param cfg User callback for interrupt notification
 * @param userData User callback private data
 */
status_t gpt_setup_capture(int id, struct gpt_capture_config *cfg, gpt_capture_callback_t callback, void *userData)
{
    struct device *dev = class_gpt_get_device_by_id(id);
    return class_gpt_setup_capture(dev, cfg, callback, userData);
}


static status_t imx_gpt_setup_capture(struct device *dev, void *cfg, gpt_capture_callback_t callback, void *userData)
{
    struct imx_gpt_state *state;
    state = dev->state;
    ASSERT(state);
    GPT_Type *base = state->io_base;
    ASSERT(base);

    struct gpt_capture_config *pcfg = (struct gpt_capture_config *) cfg;
    ASSERT(pcfg);
    /* save config to capture handle */
    ASSERT(pcfg->channel <= kGPT_InputCapture_Channel2);
    struct gpt_capture_handle *gpt_capture_handle = &state->gpt_capture_handle[pcfg->channel];
    memcpy(&gpt_capture_handle->config, pcfg, sizeof(struct gpt_capture_config));

    GPT_SetInputOperationMode(base, pcfg->channel, pcfg->mode);
    /* save callback function to capture handle */
    gpt_capture_handle->callback = callback;
    gpt_capture_handle->userData = userData;

    /* enable/disable interrupts */
    uint32_t mask = 0;
    if (pcfg->channel == kGPT_InputCapture_Channel1) {
        mask = kGPT_InputCapture1InterruptEnable;
    } else if (pcfg->channel ==  kGPT_InputCapture_Channel2) {
        mask = kGPT_InputCapture2InterruptEnable;
    }
    if (pcfg->irq_enable) {
        GPT_EnableInterrupts(base, mask);
        printlk(LK_INFO, "%s:%d: Enabling GPT capture IRQ\n",
                __PRETTY_FUNCTION__, __LINE__);
    } else {
        GPT_DisableInterrupts(base, mask);
        printlk(LK_INFO, "%s:%d: Disabling GPT capture IRQ\n",
                __PRETTY_FUNCTION__, __LINE__);
    }
    return 0;
}

/** get capture timer
 * @param channel Capture channel number (1 or 2)
 * @param counter counter value when last capture event occured
 */
status_t gpt_get_capture(int id, unsigned int channel, uint32_t *counter)
{
    struct device *dev = class_gpt_get_device_by_id(id);
    return class_gpt_get_capture(dev, channel, counter);
}

static status_t imx_gpt_get_capture(struct device *dev, unsigned int channel, uint32_t *counter)
{
    struct imx_gpt_state *state;
    state = dev->state;
    ASSERT(state);
    GPT_Type *base = state->io_base;
    ASSERT(base);

    *counter = GPT_GetInputCaptureValue(base, (enum gpt_input_capture_channel) channel);
    return 0;
}

static struct device_class gpt_device_class = {
    .name = "gpt",
};

static struct gpt_ops the_ops = {
    .std = {
        .device_class = &gpt_device_class,
        .init = imx_gpt_init,
    },
    .open = imx_gpt_open,
    .close = imx_gpt_close,
    .stop = imx_gpt_stop,
    .start = imx_gpt_start,
    .get_counter = imx_gpt_get_counter,
    .setup_capture = imx_gpt_setup_capture,
    .enable_capture = imx_gpt_enable_capture,
    .get_capture = imx_gpt_get_capture,
};

DRIVER_EXPORT(gpt, &the_ops.std);

struct device *class_gpt_get_device_by_id(int id)
{
    struct device *dev = NULL;
    struct imx_gpt_state *state = NULL;

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&imx_gpt_list_lock, lock_state);

    list_for_every_entry(&imx_gpt_list, state, struct imx_gpt_state, node) {
        if (state->bus_id == id) {
            dev = state->device;
            break;
        }
    }

    spin_unlock_irqrestore(&imx_gpt_list_lock, lock_state);

    return dev;
}

