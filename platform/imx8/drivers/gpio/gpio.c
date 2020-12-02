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

#include <dev/class/gpio.h>
#include "platform/gpio.h"
#include "gpio_hw.h"

#include "kernel/thread.h"
#include "kernel/event.h"
#include "kernel/mutex.h"
#include "kernel/spinlock.h"
#include "err.h"
#include <string.h>
#include <lib/appargs.h>


struct gpio_irq_state {
#define MAX_GPIO_IRQ_NAME 8
    char name[MAX_GPIO_IRQ_NAME]; /* GPIO %d */
    GPIO_Type* base;
    unsigned irq;
    unsigned id;
    unsigned isr_bitmap;
    struct gpio_state *state;
    int_handler isr[MAX_GPIO_IRQ_PER_BANK];
    void * args[MAX_GPIO_IRQ_PER_BANK];
    thread_t *thread;
    event_t event;
    spin_lock_t lock;
    mutex_t mutex;
};

#define IMX_GPIO_MAX_PER_BANK 32
#define IMX_GPIO_NAMES_LENGTH 32
struct imx_gpio_host {
    struct device *device;
    GPIO_Type* io_base;
    struct gpio_irq_state irq_state;
    spin_lock_t lock;
    /* What are the banks with a gpio IRQ installed */
    unsigned isr_bitmap;
    char request_name[IMX_GPIO_MAX_PER_BANK][IMX_GPIO_NAMES_LENGTH];
};

#if 0
static inline struct gpio_irq_state * get_irq_state(struct gpio_state * state, unsigned id)
{
    DEBUG_ASSERT(id < state->config->max_banks);

    return &state->irq_state[id];
}

static enum handler_return imx_gpio_isr(void *args)
{
    struct gpio_irq_state * irq_state = args;


    spin_lock_saved_state_t state;

    spin_lock(&irq_state->lock);
    /* Mask the interrupts:
     * I mask only the enabled one - but might be wiser to clear everything */
    GPIO_DisableInterrupts(irq_state->base, irq_state->isr_bitmap);

    spin_unlock(&irq_state->lock);

    /* Wake the bottom half */
    event_signal(&irq_state->event, false);

    return  INT_RESCHEDULE;
}

static int imx_gpio_thread(void *args)
{
    struct gpio_irq_state * irq_state = args;
    bool thread_running = true;
    unsigned pending;
    unsigned pin;

    while(thread_running) {
        event_wait(&irq_state->event);
        /* Read pending interrupts */
        pending = GPIO_GetPinsInterruptFlags(irq_state->base);
        /* Iterate over the pending interrupts and call isr callbacks */
        for (pin  = 0; pin < MAX_GPIO_IRQ_PER_BANK; pin++) {
            if (pending & (1U << pin)) {
                if (irq_state->isr[pin])
                    irq_state->isr[pin](irq_state->args[pin]);
                GPIO_ClearPinsInterruptFlags(irq_state->base, (1U << pin));
            }
        }
        event_unsignal(&irq_state->event);
        GPIO_EnableInterrupts(irq_state->base, irq_state->isr_bitmap);
    };

    return 0;
}

static int gpio_init_irq(struct gpio_state * state, unsigned id)
{

    DEBUG_ASSERT(id < MAX_GPIO_BANKS);
    DEBUG_ASSERT(state->config->bank_mask & (1UL << id));
    DEBUG_ASSERT(state->config->irq[id]  > 32);

    struct gpio_irq_state *irq_state = get_irq_state(state, id);

    /* Clear everything ... */
    memset(irq_state, 0, sizeof(struct gpio_irq_state));

    snprintf(irq_state->name, MAX_GPIO_IRQ_NAME, "GPIO %d", id);

    irq_state->state = state;
    irq_state->id = id;
    irq_state->irq = state->config->irq[id];
    irq_state->base = (GPIO_Type *) state->config->virt_io[id];

    event_init(&irq_state->event, false, 0);
    spin_lock_init(&irq_state->lock);
    mutex_init(&irq_state->mutex);

    irq_state->thread = thread_create(
                                    irq_state->name,
                                    imx_gpio_thread,
                                    irq_state,
                                    HIGH_PRIORITY,
                                    DEFAULT_STACK_SIZE);

    assert(irq_state->thread);

    register_int_handler(state->config->irq[id], imx_gpio_isr, irq_state);

    return 0;
}

static int _gpio_config(unsigned nr, unsigned flags)
{
    gpio_pin_config_t config = {kGPIO_DigitalInput, 0, kGPIO_NoIntmode};
    unsigned bank = extract_bank(nr);
    unsigned bit = extract_bit(nr);

    struct gpio_irq_state *irq_state = get_irq_state(imx_gpio_state, bank);

    /* Configure the gpio in input or output */
    if ((flags & (GPIO_INPUT | GPIO_OUTPUT))) {
        /* Either input or output */
        DEBUG_ASSERT(flags != (GPIO_INPUT | GPIO_OUTPUT));

        if (flags & GPIO_OUTPUT) {
            config.direction = kGPIO_DigitalOutput;
            config.outputLogic = extract_default(nr);
        }

        GPIO_PinInit(irq_state->base, bit, &config);

        return 0;
    }

    /* So this is an interrupt ... */

    /* Either edge or level ... */
    DEBUG_ASSERT(flags & (GPIO_EDGE | GPIO_LEVEL));

    /* ... but not both ... */
    DEBUG_ASSERT(flags != (GPIO_EDGE | GPIO_LEVEL));

    /* Is it a edge sensitive interrupt */
    if (flags & GPIO_EDGE) {
        DEBUG_ASSERT(flags & (GPIO_RISING | GPIO_FALLING));
        if (flags & GPIO_RISING)
            config.interruptMode = kGPIO_IntRisingEdge;
        else if (flags & GPIO_FALLING)
                config.interruptMode = kGPIO_IntFallingEdge;
            else
                config.interruptMode = kGPIO_IntRisingOrFallingEdge;
    }

    /* Is it a level sensitive interrupt */
    if (flags & GPIO_LEVEL) {
        DEBUG_ASSERT(flags & (GPIO_HIGH | GPIO_LOW));
        DEBUG_ASSERT(flags != (GPIO_HIGH | GPIO_LOW));
        if (flags & GPIO_HIGH)
            config.interruptMode = kGPIO_IntHighLevel;
        else
            config.interruptMode = kGPIO_IntLowLevel;
    }

    GPIO_PinInit(irq_state->base, bit, &config);

    return 0;
}

int gpio_config(unsigned nr, unsigned flags)
{
    int ret;
    unsigned bank = extract_bank(nr);
    struct gpio_irq_state *irq_state = get_irq_state(imx_gpio_state, bank);

    spin_lock(&irq_state->lock);

    ret = _gpio_config(nr, flags);

    spin_unlock(&irq_state->lock);

    return 0;
}

void imx_gpio_config(unsigned gpio)
{
    gpio_config(gpio & 0xFFFF, GPIO_GET_FLAGS(gpio));
}

void gpio_set(unsigned nr, unsigned on)
{
    unsigned bank = extract_bank(nr);
    unsigned bit = extract_bit(nr);
    struct gpio_irq_state *irq_state = get_irq_state(imx_gpio_state, bank);

    spin_lock(&irq_state->lock);

    GPIO_PinWrite(irq_state->base, bit, on);

    spin_unlock(&irq_state->lock);
}

int gpio_get(unsigned nr)
{

    unsigned bank = extract_bank(nr);
    unsigned pin = extract_bit(nr);
    struct gpio_irq_state *irq_state = get_irq_state(imx_gpio_state, bank);

    /* FIXME: Check a spin lock is not required for read */

    return (int) GPIO_PinRead(irq_state->base, pin);
}

void register_gpio_int_handler(unsigned gpio, int_handler handler, void *args)
{
    unsigned bank = extract_bank(gpio);
    unsigned pin = extract_bit(gpio);

    struct gpio_irq_state * irq_state = get_irq_state(imx_gpio_state, bank);

    spin_lock_saved_state_t state;

    spin_lock_irqsave(&irq_state->lock, state);

    /* Init the GPIO mode */
    _gpio_config(gpio, GPIO_GET_FLAGS(gpio));

    irq_state->isr[bank] = handler;
    irq_state->args[bank] = args;
    irq_state->isr_bitmap |= (1U << pin);

    GPIO_ClearPinsInterruptFlags(irq_state->base, 1U << pin);
    GPIO_EnableInterrupts(irq_state->base, 1U << pin);

    spin_unlock_irqrestore(&irq_state->lock, state);

    spin_lock_irqsave(&imx_gpio_state->lock, state);

    if (!(imx_gpio_state->isr_bitmap & (1U << bank)))
        unmask_interrupt(irq_state->irq);

    imx_gpio_state->isr_bitmap |= (1U << bank);

    spin_unlock_irqrestore(&imx_gpio_state->lock, state);
}

void unregister_gpio_int_handler(unsigned gpio)
{

    unsigned bank = extract_bank(gpio);
    unsigned pin = extract_bit(gpio);

    struct gpio_irq_state * irq_state = get_irq_state(imx_gpio_state, bank);

    spin_lock_saved_state_t state;

    spin_lock_irqsave(&imx_gpio_state->lock, state);

    GPIO_ClearPinsInterruptFlags(irq_state->base, 1U << pin);
    GPIO_DisableInterrupts(irq_state->base, 1U << pin);

    irq_state->isr[bank] = NULL;
    irq_state->args[bank] = NULL;
    irq_state->isr_bitmap &= ~(1U << pin);

    spin_unlock(&irq_state->lock);


    spin_lock_irqsave(&imx_gpio_state->lock, state);

    if (irq_state->isr_bitmap == 0) {
        imx_gpio_state->isr_bitmap &= ~(1U << bank);
        mask_interrupt(irq_state->irq);
    }

    spin_unlock_irqrestore(&imx_gpio_state->lock, state);

}

int gpio_to_irq(unsigned nr)
{
    unsigned bank = extract_bank(nr);
    unsigned pin = extract_bit(nr);

    struct gpio_irq_state * irq_state = get_irq_state(imx_gpio_state, bank);
    return 0;
}

int gpio_init(struct imx_gpio_config *config)
{
    int i;

    spin_lock_init(&imx_gpio_state->lock);
    imx_gpio_state->config = config;
    imx_gpio_state->isr_bitmap = 0;

    for ( i = 0; i < config->max_banks; i++)
        if (config->bank_mask & (1U << i))
            gpio_init_irq(imx_gpio_state, i);

    return 0;
}
#endif
static status_t imx_gpio_init(struct device *dev)
{
    printlk(LK_NOTICE, "%s: entry\n", __PRETTY_FUNCTION__);

    const struct device_config_data *config = dev->config;
    ASSERT(config);

    uint32_t ngpios = IMX_GPIO_MAX_PER_BANK;
    of_device_get_int32(dev, "ngpios", &ngpios);
    if (ngpios > IMX_GPIO_MAX_PER_BANK) {
        printlk(LK_ERR, "%s:%d: Invalid ngpios argument (%d) - limit to %d\n",
                        __PRETTY_FUNCTION__, __LINE__, ngpios,  IMX_GPIO_MAX_PER_BANK);
        ngpios = IMX_GPIO_MAX_PER_BANK;
    }

    struct gpio_controller *ctrl = gpio_controller_add(
                                dev, ngpios, sizeof(struct imx_gpio_host));
    ASSERT(ctrl);
    printlk(LK_INFO, "%s:%d: Gpio %s: %d pins, base %d\n", __PRETTY_FUNCTION__,
            __LINE__, dev->name, ctrl->count, ctrl->base);

    dev->state = ctrl;
    struct imx_gpio_host * gpio_host = gpio_ctrl_to_host(ctrl);

    memset(gpio_host, 0, sizeof(*gpio_host));
    gpio_host->device = dev;

    struct device_cfg_reg *reg =
                        device_config_get_register_by_name(config, "core");
    ASSERT(reg);

    gpio_host->io_base = (GPIO_Type *) reg->vbase;
    ASSERT(gpio_host->io_base);

    spin_lock_init(&gpio_host->lock);

    return 0;
}

static int imx_gpio_request(struct device *dev,
                                    unsigned pin, const char *label)
{
    struct gpio_controller *ctrl = dev->state;
    ASSERT(ctrl);
    struct imx_gpio_host *gpio_host = gpio_ctrl_to_host(ctrl);
    ASSERT(gpio_host);
    const struct device_config_data *config = dev->config;
    ASSERT(config);

    if (strlen(gpio_host->request_name[pin]) != 0) {
        printlk(LK_ERR, "%s:%d: Pin %d already requested as %s\n",
                __PRETTY_FUNCTION__, __LINE__, pin,
                gpio_host->request_name[pin]);
        return ERR_BUSY;
    }
    /* Pin */
    char pin_name[IMX_GPIO_NAMES_LENGTH];

    /* Try a specialized pinctrl config */
    strncpy(pin_name, label, IMX_GPIO_NAMES_LENGTH);
    printlk(LK_INFO, "%s:%d: Attempting to configure pinctrl label %s\n",
            __PRETTY_FUNCTION__, __LINE__, pin_name);
    int ret = devcfg_set_pins_by_name(config->pins_cfg,
                            config->pins_cfg_cnt, pin_name);
    if (ret != 0) {
        /* Try a specialized pinctrl config */
        sprintf(pin_name, "IO%d", pin);
        printlk(LK_INFO, "%s:%d: Fallback pinctrl label %s configuration\n",
                __PRETTY_FUNCTION__, __LINE__, pin_name);
        ret = devcfg_set_pins_by_name(config->pins_cfg,
                            config->pins_cfg_cnt, pin_name);
        if (ret != 0) {
            printlk(LK_ERR, "%s:%d: %s pinctrl configuration missing, fail!\n",
                    __PRETTY_FUNCTION__, __LINE__, pin_name);
            return ERR_NOT_FOUND;
        }
    }

    strncpy(gpio_host->request_name[pin], label, IMX_GPIO_NAMES_LENGTH);

    return 0;
}

static int imx_gpio_free(struct device *dev, unsigned pin)
{
    struct gpio_controller *ctrl = dev->state;
    ASSERT(ctrl);
    struct imx_gpio_host *gpio_host = gpio_ctrl_to_host(ctrl);
    ASSERT(gpio_host);

    memset(gpio_host->request_name[pin], 0, IMX_GPIO_NAMES_LENGTH);
    /* Todo Release pinctrl configuration */

    return 0;
}

static int imx_gpio_direction_input(struct device *dev, unsigned pin)
{
    struct gpio_controller *ctrl = dev->state;
    ASSERT(ctrl);
    struct imx_gpio_host *gpio_host = gpio_ctrl_to_host(ctrl);
    ASSERT(gpio_host);

    gpio_pin_config_t config = {kGPIO_DigitalInput, 0, kGPIO_NoIntmode};

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&gpio_host->lock, lock_state);

    GPIO_PinInit(gpio_host->io_base, pin, &config);

    spin_unlock_irqrestore(&gpio_host->lock, lock_state);

    return 0;
}

static int imx_gpio_direction_output(struct device *dev, unsigned pin, int value)
{
    struct gpio_controller *ctrl = dev->state;
    ASSERT(ctrl);
    struct imx_gpio_host *gpio_host = gpio_ctrl_to_host(ctrl);
    ASSERT(gpio_host);

    gpio_pin_config_t config = {kGPIO_DigitalOutput, value, kGPIO_NoIntmode};

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&gpio_host->lock, lock_state);

    GPIO_PinInit(gpio_host->io_base, pin, &config);

    spin_unlock_irqrestore(&gpio_host->lock, lock_state);

    return 0;
}

static int imx_gpio_get_value(struct device *dev, unsigned pin)
{
    struct gpio_controller *ctrl = dev->state;
    ASSERT(ctrl);
    struct imx_gpio_host *gpio_host = gpio_ctrl_to_host(ctrl);
    ASSERT(gpio_host);

    return (int) GPIO_PinRead(gpio_host->io_base, pin);
}

static int imx_gpio_set_value(struct device *dev, unsigned pin, int value)
{
    struct gpio_controller *ctrl = dev->state;
    ASSERT(ctrl);
    struct imx_gpio_host *gpio_host = gpio_ctrl_to_host(ctrl);
    ASSERT(gpio_host);

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&gpio_host->lock, lock_state);

    GPIO_PinWrite(gpio_host->io_base, pin, value);

    spin_unlock_irqrestore(&gpio_host->lock, lock_state);

    return 0;
}

static int imx_gpio_get_open_drain(struct device *dev, unsigned pin)
{
    return 0;
}

static int imx_gpio_set_open_drain(struct device *dev, unsigned pin, int value)
{
    return 0;
}

static struct device_class gpio_device_class = {
    .name = "gpio",
};

struct gpio_ops imx_gpio_ops = {
    .std = {
        .device_class = &gpio_device_class,
        .init = imx_gpio_init,
    },
    .request = imx_gpio_request,
    .free = imx_gpio_free,
    .direction_input = imx_gpio_direction_input,
    .direction_output = imx_gpio_direction_output,
    .get_value = imx_gpio_get_value,
    .set_value = imx_gpio_set_value,
    .get_open_drain = imx_gpio_get_open_drain,
    .set_open_drain = imx_gpio_set_open_drain,
};

DRIVER_EXPORT_WITH_CFG_LVL(gpio, &imx_gpio_ops.std, DRIVER_INIT_CORE, sizeof(struct gpio_irq_state));
