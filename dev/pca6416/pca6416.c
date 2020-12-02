/*
 * The Clear BSD License
 * Copyright 2018, 2020 NXP
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

#include <platform/interrupts.h>
#include "dev/pca6416.h"
#include "dev/class/i2c.h"
#include "dev/i2c.h"
#include "assert.h"
#include "string.h"
#include "kernel/thread.h"
#include "kernel/event.h"
#include "kernel/mutex.h"
#include "kernel/spinlock.h"
#include <stdlib.h>
#include <debug.h>
#include <lib/appargs.h>
#include <dev/class/gpio.h>


//FIXME: Remove fsl gpio
#include "fsl_gpio.h"

/* I2C registers */
#define PCA6416_REG_INPUT_PORT_0   0
#define PCA6416_REG_INPUT_PORT_1   1
#define PCA6416_REG_OUTPUT_PORT_0  2
#define PCA6416_REG_OUTPUT_PORT_1  3
#define PCA6416_REG_INVERT_PORT_0  4
#define PCA6416_REG_INVERT_PORT_1  5
#define PCA6416_REG_CONFIG_PORT_0  6
#define PCA6416_REG_CONFIG_PORT_1  7

/* PCA6416 has two byte ports, this is byte_sel value representing both */
#define ALL_PORTS 0xffff
#define PCA6416_NR_IO 16
#define PCA6416_NAMES_LENGTH 32

#define BIT(x) (1UL << x)

/* structure representing one PCA6416 device and its registers */
struct pca6416 {
    struct device *device;
    struct device *i2c_dev;
    event_t event;
    spin_lock_t lock;
    mutex_t mutex;
    thread_t *thread;

    /* Register interrupt handlers */
    int_handler isr[PCA6416_NR_IO];
    void *isr_arg[PCA6416_NR_IO];
    unsigned isr_bitmap;

    /* platform configuration */
    unsigned irq;
    unsigned address_7bit;
    unsigned bus_id;
    void (*gpio_irq_action)(struct pca6416 *, int);

    /* Shadow port registers (=last known values of PCA6416 I2C registers) */
    uint32_t port_config;  /* 1=input, 0=output */
    uint32_t port_invert;  /* 1=invert, 0=output */
    uint32_t port_output;  /* last output value set */
    uint32_t port_input;   /* last input value read */

    char request_name[PCA6416_NR_IO][PCA6416_NAMES_LENGTH];
};


/* read consecutive I2C registers depending on byte_sel register (each non-zero byte in byte_sel causes one I2C access) */
static void pca6416_read_reg(struct pca6416 *device, int reg, uint32_t *value, uint32_t byte_sel)
{
    uint8_t *pval = (uint8_t *)value;
    uint8_t reg_ix = (uint8_t)reg;
    uint8_t size = 2;

    /* skip register 0? */
    if (!(byte_sel & 0xff)) {
        pval++;
        reg_ix++;
        size = 1;
    }
    /* skip register 1? */
    else if (!(byte_sel & 0xff00)) {
        size = 1;
    }

    i2c_read_reg_bytes(
        device->bus_id,
        device->address_7bit,
        reg_ix,
        pval,
        size
    );
}

/* write consecutive I2C registers depending on byte_sel register (each non-zero byte in byte_sel causes one I2C access) */
static void pca6416_write_reg(struct pca6416 *device, int reg, uint32_t value, uint32_t byte_sel)
{
    uint8_t *pval = (uint8_t *)&value;
    uint8_t reg_ix = (uint8_t)reg;
    uint8_t size = 2;

    /* skip register 0? */
    if (!(byte_sel & 0xff)) {
        pval++;
        reg_ix++;
        size = 1;
    }
    /* skip register 1? */
    else if (!(byte_sel & 0xff00)) {
        size = 1;
    }

    i2c_write_reg_bytes(
        device->bus_id,
        device->address_7bit,
        reg_ix,
        pval,
        size
    );
}


int pca6416_thread(void *arg)
{
    struct pca6416 *device = arg;
    bool thread_running = true;
    unsigned int_pending;
    int i;

    while (thread_running) {
        event_wait(&device->event);
        /* The GPIO has been masked in the isr */
        device->gpio_irq_action(device, IMX_PCA_GPIO_IRQ_CLEAR);

        pca6416_read_input_pins((pca6416_handle_t) device, &int_pending);
        /* TODO: supports active low interrupt */
        int_pending &= device->isr_bitmap;
        printlk(LK_DEBUG, "=====> int_pending: 0x%x\n", int_pending);
        for (i = 0; i < PCA6416_NR_IO; i++) {
            if (int_pending & (1UL << i)) {
                /* Handle orphaned interrupt gracefully */
                assert(device->isr[i]);
                device->isr[i](device->isr_arg[i]);
            }
        }

        /* Clear, unsignal and unmask the interrupt */
        event_unsignal(&device->event);
        /* FIXME: Following should be done in the bottom half of GPIO ISR */
        device->gpio_irq_action(device, IMX_PCA_GPIO_IRQ_UNMASK);
        printlk(LK_DEBUG, "<===== End int_pending: 0x%x\n", int_pending);

        thread_preempt();
    }

    return 0;
}

/* FIXME: should use GPIO subsystem to clear the interrupt
 *  or at least do not hardcode the 3 definitions below.
 * */
#define PCA6416_INT_GPIO       GPIO1
#define PCA6416_INT_GPIO_PIN   12
#define PCA6416_INT_GPIO_IRQn  GPIO1_Combined_0_15_IRQn

/* FIXME: This ISR should be in the GPIO generic ISR */
enum handler_return pca6416_isr(void *arg)
{
    struct pca6416 *device = arg;
    unsigned  gpio_int_f;
    /* Mask the interrupt */

    /* FIXME: Should be done in the GPIO global ISR Handler */
    gpio_int_f = GPIO_GetPinsInterruptFlags(PCA6416_INT_GPIO);
    printlk(LK_DEBUG, "gpio value: %x\n", gpio_int_f);
//    assert(gpio_int_f == (1U << PCA6416_INT_GPIO_PIN));

    device->gpio_irq_action(device, IMX_PCA_GPIO_IRQ_MASK);
    /* and wake up the thread */
    event_signal(&device->event, false);

    return INT_RESCHEDULE;
}

static void imx_pca_gpio_irq(struct pca6416 *state, int mode)
{
    switch (mode) {
        case IMX_PCA_GPIO_IRQ_INIT: {
            gpio_pin_config_t pca6416_int_cfg = { kGPIO_DigitalInput, 0, kGPIO_IntFallingEdge };
            /* enable PCA6416A interrupts to be detected in edge mode */
            GPIO_PinInit(PCA6416_INT_GPIO, PCA6416_INT_GPIO_PIN, &pca6416_int_cfg);
        }
        break;

        case IMX_PCA_GPIO_IRQ_UNMASK:
            GPIO_EnableInterrupts(PCA6416_INT_GPIO, 1U << PCA6416_INT_GPIO_PIN);
            break;
        case IMX_PCA_GPIO_IRQ_MASK:
            GPIO_DisableInterrupts(PCA6416_INT_GPIO, 1U << PCA6416_INT_GPIO_PIN);
            break;
        case IMX_PCA_GPIO_IRQ_CLEAR:
            GPIO_ClearPinsInterruptFlags(PCA6416_INT_GPIO, 1U << PCA6416_INT_GPIO_PIN);
            break;
    }
}
pca6416_handle_t pca6416_probe(const struct pca6416_config *config)
{
    printlk(LK_NOTICE, "%s:%d: Probing pca6416..\n", __PRETTY_FUNCTION__,
            __LINE__);
    return 0;
}

struct pca6416 *g_pca6416_device;

static status_t pca6416_init(struct device *dev)
{
    struct pca6416 *device;
    struct device *i2c_device;
    const struct device_config_data *config = dev->config;
    assert(config);
    uint32_t ngpios = PCA6416_NR_IO;
    of_device_get_int32(dev, "ngpios", &ngpios);
    if (ngpios > PCA6416_NR_IO) {
        printlk(LK_ERR, "%s:%d: Invalid ngpios argument (%d) - limit to %d\n",
                __PRETTY_FUNCTION__, __LINE__, ngpios, PCA6416_NR_IO);
        ngpios = PCA6416_NR_IO;
    }
    struct gpio_controller *ctrl = gpio_controller_add(
                                dev, ngpios, sizeof(struct pca6416));
    ASSERT(ctrl);
    printlk(LK_INFO, "%s:%d: Gpio %s: %d pins, base %d\n", __PRETTY_FUNCTION__,
            __LINE__, dev->name, ctrl->count, ctrl->base);

    dev->state = ctrl;
    device = gpio_ctrl_to_host(ctrl);

    memset(device, 0, sizeof(*device));
    device->device = dev;
    g_pca6416_device = device;

    struct device_cfg_reg *reg =
                        device_config_get_register_by_name(config, "core");
    ASSERT(reg);
    device->address_7bit = (unsigned) reg->base;

    struct device_cfg_irq *irq =
                        device_config_get_irq_by_name(config, "core");
    ASSERT(irq);

    device->irq = irq->irq;
    if (config->bus_id == -1) {
        uint32_t busid;
        of_device_get_parent_bus_id(dev, &busid);
        device->bus_id = busid;
    } else {
        device->bus_id = config->bus_id;
    }

    device->gpio_irq_action = imx_pca_gpio_irq;

    printlk(LK_NOTICE, "%s:%d: PCA6416 on i2c bus %d, IRQ %d\n",
           __PRETTY_FUNCTION__, __LINE__, device->bus_id, device->irq);

    event_init(&device->event, false, 0);
    spin_lock_init(&device->lock);
    mutex_init(&device->mutex);

    mask_interrupt(device->irq);

    device->thread = thread_create("pca6416", pca6416_thread, device,
                                            HIGH_PRIORITY, DEFAULT_STACK_SIZE);
    assert(device->thread);

    i2c_device = class_i2c_get_device_by_id(device->bus_id);
    assert(i2c_device != NULL);
    device->i2c_dev = i2c_device;

    devcfg_set_pins_by_name(config->pins_cfg, config->pins_cfg_cnt, "default");

    pca6416_set_as_input(device, (1U << ngpios) - 1);
    /* todo: handle I2C timeout later, this will hang if PCA6416 is not connected */
    pca6416_read_reg(device, PCA6416_REG_OUTPUT_PORT_0, &device->port_output, ALL_PORTS);
    pca6416_read_reg(device, PCA6416_REG_INVERT_PORT_0, &device->port_invert, ALL_PORTS);
    pca6416_read_reg(device, PCA6416_REG_CONFIG_PORT_0, &device->port_config, ALL_PORTS);
    pca6416_read_reg(device, PCA6416_REG_INPUT_PORT_0, &device->port_input, ALL_PORTS);

    /* FIXME: GPIO IRQ handling should be done through dedicated GPIO API */
    device->gpio_irq_action(device, IMX_PCA_GPIO_IRQ_INIT);
    device->gpio_irq_action(device, IMX_PCA_GPIO_IRQ_MASK);
    device->gpio_irq_action(device, IMX_PCA_GPIO_IRQ_CLEAR);

    thread_detach_and_resume(device->thread);

    device->isr_bitmap = 0;
    register_int_handler(device->irq, pca6416_isr, device);

    return 0;
}

#if 0
static struct device_class pca6416_device_class = {
    .name = "none",
};
static struct driver_ops the_ops = {
        .device_class = &pca6416_device_class,
        .init = pca6416_init,
};

DRIVER_EXPORT_WITH_LVL(pca6416, &the_ops, DRIVER_INIT_TARGET);
#endif

int pca6416_register_int(pca6416_handle_t dev, int port_no, int_handler handler, void *arg)
{
    struct pca6416 *device = (struct pca6416 *)dev;
    assert(port_no < PCA6416_NR_IO);
    assert(handler);
    printlk(LK_NOTICE, "%s:%d: Registering IRQ %p on port %d\n",
            __PRETTY_FUNCTION__, __LINE__, handler, port_no);
    // TODO: spinlock
    device->isr_bitmap |= (1UL << port_no);

    device->isr[port_no] = handler;
    device->isr_arg[port_no] = arg;

    pca6416_set_as_input(dev, (1UL << port_no));
    device->gpio_irq_action(device, IMX_PCA_GPIO_IRQ_UNMASK);

    unmask_interrupt(device->irq);

    return 0;
}

int pca6416_unregister_int(pca6416_handle_t dev, int port_no)
{
    struct pca6416 *device = (struct pca6416 *)dev;
    assert(port_no < PCA6416_NR_IO);
    // TODO: spinlock
    //
    printlk(LK_NOTICE, "%s:%d: Deregistering IRQ on port %d\n",
            __PRETTY_FUNCTION__, __LINE__, port_no);
    device->isr_bitmap &= ~(1UL << port_no);

    if (device->isr_bitmap == 0) {
        device->gpio_irq_action(device, IMX_PCA_GPIO_IRQ_MASK);
        mask_interrupt(device->irq);
    }

    device->isr[port_no] = NULL;
    device->isr_arg[port_no] = NULL;

    return 0;
}


int pca6416_set_as_input(pca6416_handle_t dev, uint32_t input_pins)
{
    struct pca6416 *device = (struct pca6416 *)dev;

    assert(device != NULL);

    mutex_acquire(&device->mutex);

    /* remember which pins are input (config bits as 1) */
    device->port_config |= input_pins;
    /* update affected I2C registers */
    pca6416_write_reg(device, PCA6416_REG_CONFIG_PORT_0, device->port_config, input_pins);

    mutex_release(&device->mutex);

    /* todo: handle I2C timeout later, the above will hang if PCA6416 is not connected */
    return 0;
}

int pca6416_set_as_output(pca6416_handle_t dev, uint32_t output_pins)
{
    struct pca6416 *device = (struct pca6416 *)dev;

    assert(device != NULL);

    mutex_acquire(&device->mutex);

    /* remember which pins are output (config bits as 0) */
    device->port_config &= ~output_pins;
    /* update affected I2C registers */
    pca6416_write_reg(device, PCA6416_REG_CONFIG_PORT_0, device->port_config, output_pins);

    mutex_release(&device->mutex);

    /* todo: handle I2C timeout later, the above will hang if PCA6416 is not connected */
    return 0;
}

int pca6416_get_direction(pca6416_handle_t dev, uint32_t *input_pins)
{
    struct pca6416 *device = (struct pca6416 *)dev;

    assert(device != NULL);
    assert(input_pins != NULL);

    /* tell last known state of which pins are input */
    *input_pins = device->port_config;
    return 0;
}

int pca6416_set_polarity_normal(pca6416_handle_t dev, uint32_t normal_pins)
{
    struct pca6416 *device = (struct pca6416 *)dev;

    assert(device != NULL);

    mutex_acquire(&device->mutex);

    /* remember which pins are non-inverted (polarity bits as 0) */
    device->port_invert &= ~normal_pins;
    /* update affected I2C registers */
    pca6416_write_reg(device, PCA6416_REG_INVERT_PORT_0, device->port_config, normal_pins);

    mutex_release(&device->mutex);

    /* todo: handle I2C timeout later, the above will hang if PCA6416 is not connected */
    return 0;
}

int pca6416_set_polarity_inverted(pca6416_handle_t dev, uint32_t inverted_pins)
{
    struct pca6416 *device = (struct pca6416 *)dev;

    assert(device != NULL);

    mutex_acquire(&device->mutex);

    /* remember which pins are inverted (polarity bits as 1) */
    device->port_invert |= inverted_pins;
    /* update affected I2C registers */
    pca6416_write_reg(device, PCA6416_REG_INVERT_PORT_0, device->port_config, inverted_pins);

    mutex_release(&device->mutex);

    /* todo: handle I2C timeout later, the above will hang if PCA6416 is not connected */
    return 0;
}

int pca6416_get_polarity(pca6416_handle_t dev, uint32_t *inverted_pins)
{
    struct pca6416 *device = (struct pca6416 *)dev;

    assert(device != NULL);
    assert(inverted_pins != NULL);

    /* tell last known state of which pins are inverted */
    *inverted_pins = device->port_invert;
    return 0;
}

int pca6416_write_output_pins(pca6416_handle_t dev, uint32_t output, uint32_t pin_mask)
{
    struct pca6416 *device = (struct pca6416 *)dev;

    assert(device != NULL);

    mutex_acquire(&device->mutex);

    /* set all pins in the mask */
    device->port_output &= ~pin_mask;
    device->port_output |= output & pin_mask;
    /* update affected I2C registers */
    pca6416_write_reg(device, PCA6416_REG_OUTPUT_PORT_0, device->port_output, pin_mask);

    mutex_release(&device->mutex);

    /* todo: handle I2C timeout later, the above will hang if PCA6416 is not connected */
    return 0;
}

int pca6416_read_output_pins(pca6416_handle_t dev, uint32_t *output)
{
    struct pca6416 *device = (struct pca6416 *)dev;

    assert(device != NULL);
    assert(output != NULL);

    /* tell last known state of output pins */
    *output = device->port_output;
    return 0;
}

int pca6416_read_input_pins(pca6416_handle_t dev, uint32_t *input)
{
    struct pca6416 *device = (struct pca6416 *)dev;

    assert(device != NULL);
    assert(input != NULL);

    mutex_acquire(&device->mutex);

    /* read input registers */
    *input = 0;
    pca6416_read_reg(device, PCA6416_REG_INPUT_PORT_0, input, ALL_PORTS);
    device->port_input = *input;

    mutex_release(&device->mutex);

    /* todo: handle I2C timeout later, the above will hang if PCA6416 is not connected */
    return 0;
}

static int pca6416_gpio_request(struct device *dev,
                                    unsigned offset, const char *label)
{
    struct gpio_controller *ctrl = dev->state;

    ASSERT(ctrl);
    printlk(LK_INFO, "%s:%d: Request pin %d with label %s .\n",
            __PRETTY_FUNCTION__, __LINE__, offset, label);

    if (offset > ctrl->count) {
        printlk(LK_ERR, "%s:%d: Offset %d out of range (>%d)\n",
                __PRETTY_FUNCTION__, __LINE__, offset, ctrl->count);
        return ERR_INVALID_ARGS;
    }

    struct pca6416 *pca6416_dev = gpio_ctrl_to_host(ctrl);
    ASSERT(pca6416_dev);

    if (strlen(pca6416_dev->request_name[offset]) != 0) {
        printlk(LK_ERR, "%s:%d: Pin %d already requested as %s\n",
                __PRETTY_FUNCTION__, __LINE__, offset,
                pca6416_dev->request_name[offset]);
        return ERR_BUSY;
    }

    strncpy(pca6416_dev->request_name[offset], label, PCA6416_NAMES_LENGTH);

    return 0;
}

static int pca6416_gpio_free(struct device *dev, unsigned offset)
{
    struct gpio_controller *ctrl = dev->state;

    ASSERT(ctrl);

    struct pca6416 *pca6416_dev = gpio_ctrl_to_host(ctrl);
    ASSERT(pca6416_dev);
    memset(pca6416_dev->request_name[offset], 0, PCA6416_NAMES_LENGTH);

    return 0;
}
static int pca6416_direction_input(struct device *dev, unsigned offset)
{
    struct gpio_controller *ctrl = dev->state;
    ASSERT(ctrl);

    return pca6416_set_as_input(gpio_ctrl_to_host(ctrl), BIT(offset));
}

static int pca6416_direction_output(struct device *dev, unsigned offset, int value)
{
    struct gpio_controller *ctrl = dev->state;
    ASSERT(ctrl);
    pca6416_handle_t pca6416_dev = gpio_ctrl_to_host(ctrl);

    pca6416_write_output_pins(pca6416_dev,
                                ((value & 1) << offset), BIT(offset));
    pca6416_set_as_output(pca6416_dev, BIT(offset));
    return 0;
}

static int pca6416_get_value(struct device *dev, unsigned offset)
{
    uint32_t input;
    struct gpio_controller *ctrl = dev->state;
    ASSERT(ctrl);
    pca6416_handle_t pca6416_dev = gpio_ctrl_to_host(ctrl);

    pca6416_read_input_pins(pca6416_dev, &input);

    return ((input >> offset) & 1);
}

static int pca6416_set_value(struct device *dev, unsigned offset, int value)
{
    struct gpio_controller *ctrl = dev->state;
    ASSERT(ctrl);
    pca6416_handle_t pca6416_dev = gpio_ctrl_to_host(ctrl);

    pca6416_write_output_pins(pca6416_dev,
                                ((value & 1) << offset), BIT(offset));
    return 0;
}

static struct device_class gpio_device_class = {
    .name = "gpio",
};

struct gpio_ops pca6416_gpio_ops = {
    .std = {
        .device_class = &gpio_device_class,
        .init = pca6416_init,
    },
    .request = pca6416_gpio_request,
    .free = pca6416_gpio_free,
    .direction_input = pca6416_direction_input,
    .direction_output = pca6416_direction_output,
    .get_value = pca6416_get_value,
    .set_value = pca6416_set_value,
    .get_open_drain = NULL,
    .set_open_drain = NULL,
};

DRIVER_EXPORT_WITH_LVL(pca6416, &pca6416_gpio_ops.std, DRIVER_INIT_PLATFORM_EARLY);
