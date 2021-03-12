/*
 * The Clear BSD License
 * Copyright 2019-2021 NXP
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
#include <err.h>
#include <malloc.h>
#include <string.h>

#define LOCAL_TRACE 0
#include "dev/i2c.h"
#include <dev/class/i2c.h>
#include <dev/class/adc.h>
#include <lib/appargs.h>

#define DRIVER_NAME "adc_pcm186x"

#define PCM186X_REG_00           (  0)
#define PCM186X_PCM_CFG          ( 11)
#define PCM186X_TDM_TX_OFFSET    ( 13)
#define PCM186X_GPIO1_0_CTRL     ( 16)
#define PCM186X_GPIO3_2_CTRL     ( 17)
#define PCM186X_GPIO1_0_DIR_CTRL ( 18)
#define PCM186X_GPIO3_2_DIR_CTRL ( 19)
#define PCM186X_GPIO_IN_OUT      ( 20)
#define PCM186X_CLK_CTRL         ( 32)
#define PCM186X_POWER_CTRL       (112)

/* PCM186X_PAGE */
#define PCM186X_RESET            (0xfe)

/* Page 0, Register 11 */
#define PCM186X_RX_WLEN          (3 << 6)
#define PCM186X_TX_WLEN          (3 << 2)
#define PCM186X_FMT              (3 << 0)
#define PCM186X_WLEN_16          (3)
#define PCM186X_WLEN_24          (1)
#define PCM186X_WLEN_32          (0)
#define PCM186X_FMT_I2S          (0)

/* Page 0, Register 16 */
#define PCM186X_GPIO0_FUNC       (7 << 0)
#define PCM186X_GPIO1_FUNC       (7 << 4)

/* Page 0, Register 17 */
#define PCM186X_GPIO2_FUNC       (7 << 0)
#define PCM186X_GPIO3_FUNC       (7 << 4)

/* Page 0, Register 18 */
#define PCM186X_GPIO0_DIR        (7 << 0)
#define PCM186X_GPIO1_DIR        (7 << 4)

/* Page 0, Register 19 */
#define PCM186X_GPIO2_DIR        (7 << 0)
#define PCM186X_GPIO3_DIR        (7 << 4)

/* Page 0, Register 32 */
#define PCM186X_SCK_XI_SEL1      (1 << 7)
#define PCM186X_SCK_XI_SEL0      (1 << 6)
#define PCM186X_SCK_SRC_PLL      (1 << 5)
#define PCM186X_MST_MODE         (1 << 4)
#define PCM186X_ADC_SRC_PLL      (1 << 3)
#define PCM186X_DSP2_SRC_PLL     (1 << 2)
#define PCM186X_DSP1_SRC_PLL     (1 << 1)
#define PCM186X_CLKDET_EN        (1 << 0)

/* Page 0, Register 112 */
#define PCM186X_PWRDN            (1 << 2)

struct adc_pcm186x_state {
    struct device *device;
    struct device *i2c_dev;
    uint32_t bus_id;
    uint32_t i2c_addr;
    uint32_t gpio_led;
};

static status_t adc_pcm186x_read_reg(struct adc_pcm186x_state *state, uint8_t reg, uint8_t *val)
{
    status_t ret;

    ret = i2c_read_reg(state->bus_id, state->i2c_addr, reg, val);

    return ret;
}

static status_t adc_pcm186x_write_reg(struct adc_pcm186x_state *state, uint8_t reg, uint8_t val)
{
    status_t ret;

    printlk(LK_VERBOSE, "adc_pcm186x_write_reg(reg: (%d), val: 0x%02x)\n", reg, val);
    ret = i2c_write_reg(state->bus_id, state->i2c_addr, reg, val);

    return ret;
}

static status_t adc_pcm186x_update_bits(struct adc_pcm186x_state *state, uint8_t reg, uint8_t mask, uint8_t val)
{
    status_t ret;
    uint8_t old, new;

    printlk(LK_VERBOSE, "adc_pcm186x_update_bits(reg: (%d), mask: 0x%02x, val: 0x%02x)\n", reg, mask, val);
    ret = adc_pcm186x_read_reg(state, reg, &old);
    if (ret)
        goto end;

    new = (old & (~mask)) | (val & mask);

    ret = adc_pcm186x_write_reg(state, reg, new);

end:
    return ret;
}

static status_t adc_pcm186x_set_gpio_val(struct adc_pcm186x_state *pcm186x_state, int gpio_nr, int val)
{
    status_t ret = NO_ERROR;

    switch (gpio_nr) {
        case 0:
            /* Set muxmode to gpio */
            ret = adc_pcm186x_update_bits(pcm186x_state,
                    PCM186X_GPIO1_0_CTRL,
                    PCM186X_GPIO0_FUNC,
                    0);
            if (ret)
                goto end;

            /* Set gpio as output */
            ret = adc_pcm186x_update_bits(pcm186x_state,
                    PCM186X_GPIO1_0_DIR_CTRL,
                    PCM186X_GPIO0_DIR,
                    0);
            if (ret)
                goto end;

            break;
        case 1:
            /* Set muxmode to gpio */
            ret = adc_pcm186x_update_bits(pcm186x_state,
                    PCM186X_GPIO1_0_CTRL,
                    PCM186X_GPIO1_FUNC,
                    0);
            if (ret)
                goto end;

            /* Set gpio as output */
            ret = adc_pcm186x_update_bits(pcm186x_state,
                    PCM186X_GPIO1_0_DIR_CTRL,
                    PCM186X_GPIO1_DIR,
                    4 << 4);
            if (ret)
                goto end;

            break;
        case 2:
            /* Set muxmode to gpio */
            ret = adc_pcm186x_update_bits(pcm186x_state,
                    PCM186X_GPIO3_2_CTRL,
                    PCM186X_GPIO2_FUNC,
                    0);
            if (ret)
                goto end;

            /* Set gpio as output */
            ret = adc_pcm186x_update_bits(pcm186x_state,
                    PCM186X_GPIO3_2_DIR_CTRL,
                    PCM186X_GPIO2_DIR,
                    4);
            if (ret)
                goto end;

            break;
        case 3:
            /* Set muxmode to gpio */
            ret = adc_pcm186x_update_bits(pcm186x_state,
                    PCM186X_GPIO3_2_CTRL,
                    PCM186X_GPIO3_FUNC,
                    0);
            if (ret)
                goto end;

            /* Set gpio as output */
            ret = adc_pcm186x_update_bits(pcm186x_state,
                    PCM186X_GPIO3_2_DIR_CTRL,
                    PCM186X_GPIO3_DIR,
                    4 << 4);
            if (ret)
                goto end;

            break;
        default:
            ret = ERR_INVALID_ARGS;
            goto end;
    }

    /* Set / clear gpio */
    ret = adc_pcm186x_update_bits(pcm186x_state,
            PCM186X_GPIO_IN_OUT,
            1 << (gpio_nr + 4),
            1 << (gpio_nr + 4));

end:
    return ret;
}

static status_t adc_pcm186x_set_gpio(struct adc_pcm186x_state *pcm186x_state, int gpio_nr)
{
    return adc_pcm186x_set_gpio_val(pcm186x_state, gpio_nr, 1);
}

static status_t adc_pcm186x_clear_gpio(struct adc_pcm186x_state *pcm186x_state, int gpio_nr)
{
    return adc_pcm186x_set_gpio_val(pcm186x_state, gpio_nr, 0);
}

static status_t adc_pcm186x_init(struct device *dev)
{
    int r;
    status_t ret = NO_ERROR;
    struct adc_pcm186x_state *pcm186x_state = malloc(sizeof(struct adc_pcm186x_state));
    struct device *i2c_device;

    ASSERT(pcm186x_state);
    memset(pcm186x_state, 0, sizeof(struct adc_pcm186x_state));

    printlk(LK_VERBOSE, "%s\n", __FUNCTION__);

    r = of_device_get_int32(dev, "bus-id-i2c", &pcm186x_state->bus_id);
    if (r) {
        printlk(LK_ERR, "bus-id-i2c fdt node property can't be read, aborting!\n");
        free(pcm186x_state);
        ret = ERR_NOT_FOUND;
        goto end;
    }

    r = of_device_get_int32(dev, "reg", &pcm186x_state->i2c_addr);
    if (r) {
        printlk(LK_ERR, "reg fdt node property can't be read, aborting!\n");
        free(pcm186x_state);
        ret = ERR_NOT_FOUND;
        goto end;
    }

    r = of_device_get_int32(dev, "gpio-led", &pcm186x_state->gpio_led);
    if (r) {
        printlk(LK_NOTICE, "gpio-led fdt node property can't be read\n");
        pcm186x_state->gpio_led = 0xffffffff;
    }

    printlk(LK_INFO, "Probing adc_pcm186x using i2c bus id: [%d]\n", pcm186x_state->bus_id);

    i2c_device = class_i2c_get_device_by_id(pcm186x_state->bus_id);
    ASSERT(i2c_device != NULL);

    pcm186x_state->i2c_dev = i2c_device;
    pcm186x_state->device = dev;

    dev->state = pcm186x_state;

end:
    return ret;
}

status_t adc_pcm186x_open(struct device *dev)
{
    status_t ret;
    struct adc_pcm186x_state *pcm186x_state = dev->state;

    printlk(LK_VERBOSE, "%s\n", __FUNCTION__);

    /* Reset ADC */
    ret = adc_pcm186x_write_reg(pcm186x_state, PCM186X_REG_00, PCM186X_RESET);

    /* Turn on LED */
    if (pcm186x_state->gpio_led != 0xffffffff)
        ret = adc_pcm186x_set_gpio(pcm186x_state, pcm186x_state->gpio_led);

    return ret;
}

status_t adc_pcm186x_close(struct device *dev)
{
    status_t ret;
    struct adc_pcm186x_state *pcm186x_state = dev->state;

    printlk(LK_VERBOSE, "%s\n", __FUNCTION__);

    /* Power down analog */
    ret = adc_pcm186x_update_bits(pcm186x_state,
            PCM186X_POWER_CTRL,
            PCM186X_PWRDN,
            1);

    /* Turn off LED */
    if (pcm186x_state->gpio_led != 0xffffffff)
        ret = adc_pcm186x_clear_gpio(pcm186x_state, pcm186x_state->gpio_led);

    return ret;
}

status_t adc_pcm186x_get_capabilities(const struct device *dev,
                                     uint32_t *capabilities)
{
    *capabilities = ADC_CAP_PKT_PCM;
    return 0;
}

status_t adc_pcm186x_set_format(struct device *dev,
                               adc_audio_hw_params_t *hw_params)
{
    status_t ret;
    struct adc_pcm186x_state *pcm186x_state = dev->state;
    uint8_t fmt, wlen;

    printlk(LK_VERBOSE, "%s\n", __FUNCTION__);

    switch (hw_params->fmt) {
        case ADC_AUDIO_FMT_I2S:
            fmt = PCM186X_FMT_I2S;
            break;
        default:
            ret = ERR_INVALID_ARGS;
            goto end;
    }

    switch (hw_params->pcm_fmt) {
        case ADC_AUDIO_PCM_FMT_16:
            wlen = PCM186X_WLEN_16;
            break;
        case ADC_AUDIO_PCM_FMT_24:
            wlen = PCM186X_WLEN_24;
            break;
        case ADC_AUDIO_PCM_FMT_32:
            wlen = PCM186X_WLEN_32;
            break;
        default:
            ret = ERR_INVALID_ARGS;
            goto end;
    }

    /* Set slave mode */
    ret = adc_pcm186x_update_bits(pcm186x_state,
            PCM186X_CLK_CTRL,
            PCM186X_MST_MODE,
            0);

    /* TODO: probably useless as we are not in DSP format */
    ret = adc_pcm186x_write_reg(pcm186x_state, PCM186X_TDM_TX_OFFSET, 0);

    /* Set format */
    ret = adc_pcm186x_update_bits(pcm186x_state,
            PCM186X_PCM_CFG,
            PCM186X_FMT,
            fmt);

    /* Set word length */
    ret = adc_pcm186x_update_bits(pcm186x_state,
            PCM186X_PCM_CFG,
            PCM186X_RX_WLEN | PCM186X_TX_WLEN,
            (wlen << 6) | (wlen << 2));

    /* Power up analog */
    ret = adc_pcm186x_update_bits(pcm186x_state,
            PCM186X_POWER_CTRL,
            PCM186X_PWRDN,
            0);

end:
    return ret;
}

status_t adc_pcm186x_reset_power(struct device *dev)
{
    printlk(LK_VERBOSE, "%s\n", __FUNCTION__);

    return 0;
}

static struct device_class adc_pcm186x_device_class = {
    .name = "adc",
};

static struct adc_ops the_ops = {
    .std = {
        .device_class = &adc_pcm186x_device_class,
        .init = adc_pcm186x_init,
    },
    .open               = adc_pcm186x_open,
    .close              = adc_pcm186x_close,
    .get_capabilities   = adc_pcm186x_get_capabilities,
    .set_format         = adc_pcm186x_set_format,
    .reset_power        = adc_pcm186x_reset_power,
};

DRIVER_EXPORT_WITH_LVL(adc_pcm186x, &the_ops.std, DRIVER_INIT_HAL);
