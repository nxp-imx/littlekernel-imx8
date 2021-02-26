/*
 * The Clear BSD License
 * Copyright 2021 NXP
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
#include "trace.h"

#define LOCAL_TRACE 0
#include "dev/i2c.h"
#include <dev/class/i2c.h>
#include <dev/class/dac.h>
#include <lib/appargs.h>

#define DRIVER_NAME "dac_pcm512x"

#define PCM512x_RESET             (  1)
#define PCM512x_POWER             (  2)
#define PCM512x_MUTE              (  3)
#define PCM512x_PLL_EN            (  4)
#define PCM512x_GPIO_EN           (  8)
#define PCM512x_BCLK_LRCLK_CFG    (  9)
#define PCM512x_MASTER_MODE       ( 12)
#define PCM512x_DAC_REF           ( 14)
#define PCM512x_SYNCHRONIZE       ( 19)
#define PCM512x_DSP_CLKDIV        ( 27)
#define PCM512x_DAC_CLKDIV        ( 28)
#define PCM512x_NCP_CLKDIV        ( 29)
#define PCM512x_OSR_CLKDIV        ( 30)
#define PCM512x_MASTER_CLKDIV_1   ( 32)
#define PCM512x_MASTER_CLKDIV_2   ( 33)
#define PCM512x_FS_SPEED_MODE     ( 34)
#define PCM512x_IDAC_1            ( 35)
#define PCM512x_IDAC_2            ( 36)
#define PCM512x_ERROR_DETECT      ( 37)
#define PCM512x_I2S_1             ( 40)
#define PCM512x_GPIO_OUTPUT_3     ( 82)
#define PCM512x_GPIO_OUTPUT_4     ( 83)
#define PCM512x_GPIO_OUTPUT_6     ( 85)
#define PCM512x_GPIO_CONTROL_1    ( 86)
#define PCM512x_ANALOG_MUTE_DET   (108)

/* Page 0, Register 1 - reset */
#define PCM512x_RSTR (1 << 0)
#define PCM512x_RSTM (1 << 4)

/* Page 0, Register 2 - power */
#define PCM512x_RQPD       (1 << 0)
#define PCM512x_RQST       (1 << 4)

/* Page 0, Register 3 - mute */
#define PCM512x_RQMR (1 << 0)
#define PCM512x_RQML (1 << 4)

/* Page 0, Register 4 - PLL */
#define PCM512x_PLLE       (1 << 0)
#define PCM512x_PLCK       (1 << 4)

/* Page 0, Register 9 - BCK, LRCLK configuration */
#define PCM512x_BCKO       (1 << 4)
#define PCM512x_LRKO       (1 << 0)
#define PCM512x_BCKP       (1 << 5)

/* Page 0, Register 12 - Master mode BCK, LRCLK reset */
#define PCM512x_RLRK       (1 << 0)
#define PCM512x_RBCK       (1 << 1)

/* Page 0, Register 14 - DAC reference */
#define PCM512x_SDAC        (7 << 4)
#define PCM512x_SDAC_SCK    (3 << 4)

/* Page 0, Register 19 - synchronize */
#define PCM512x_RQSY        (1 << 0)
#define PCM512x_RQSY_RESUME (0 << 0)
#define PCM512x_RQSY_HALT   (1 << 0)

/* Page 0, Register 34 - fs speed mode */
#define PCM512x_FSSP        (3 << 0)
#define PCM512x_FSSP_48KHZ  (0 << 0)
#define PCM512x_FSSP_96KHZ  (1 << 0)
#define PCM512x_FSSP_192KHZ (2 << 0)
#define PCM512x_FSSP_384KHZ (3 << 0)

/* Page 0, Register 37 - Error detection */
#define PCM512x_IPLK (1 << 0)
#define PCM512x_DCAS (1 << 1)
#define PCM512x_IDCM (1 << 2)
#define PCM512x_IDCH (1 << 3)
#define PCM512x_IDSK (1 << 4)
#define PCM512x_IDBK (1 << 5)
#define PCM512x_IDFS (1 << 6)

/* Page 0, Register 40 - I2S configuration */
#define PCM512x_ALEN       (3 << 0)
#define PCM512x_ALEN_16    (0 << 0)
#define PCM512x_ALEN_20    (1 << 0)
#define PCM512x_ALEN_24    (2 << 0)
#define PCM512x_ALEN_32    (3 << 0)
#define PCM512x_AFMT       (3 << 4)

struct dac_pcm512x_state {
    struct device *device;
    struct device *i2c_dev;
    uint32_t bus_id;
    uint32_t i2c_addr;
    uint32_t gpio_led;
    uint32_t gpio_osc44;
    uint32_t gpio_osc48;
};

static status_t dac_pcm512x_read_reg(struct dac_pcm512x_state *state, uint8_t reg, uint8_t *val)
{
    status_t ret;

    ret = i2c_read_reg(state->bus_id, state->i2c_addr, reg, val);

    return ret;
}

static status_t dac_pcm512x_write_reg(struct dac_pcm512x_state *state, uint8_t reg, uint8_t val)
{
    status_t ret;

    printlk(LK_VERBOSE, "dac_pcm512x_write_reg(reg: (%d), val: 0x%02x)\n", reg, val);
    ret = i2c_write_reg(state->bus_id, state->i2c_addr, reg, val);

    return ret;
}

static status_t dac_pcm512x_update_bits(struct dac_pcm512x_state *state, uint8_t reg, uint8_t mask, uint8_t val)
{
    status_t ret;
    uint8_t old, new;

    printlk(LK_VERBOSE, "dac_pcm512x_update_bits(reg: (%d), mask: 0x%02x, val: 0x%02x)\n", reg, mask, val);
    ret = dac_pcm512x_read_reg(state, reg, &old);
    if (ret)
        goto end;

    new = (old & (~mask)) | (val & mask);

    ret = dac_pcm512x_write_reg(state, reg, new);

end:
    return ret;
}

static status_t dac_pcm512x_mute(struct dac_pcm512x_state *state, bool mute)
{
    status_t ret;

    if (mute) {
        /* mute */
        ret = dac_pcm512x_update_bits(state,
                PCM512x_MUTE,
                PCM512x_RQML | PCM512x_RQMR,
                PCM512x_RQML | PCM512x_RQMR);
    }
    else {
        /* unmute */
        ret = dac_pcm512x_update_bits(state,
                PCM512x_MUTE,
                PCM512x_RQML | PCM512x_RQMR,
                0);
    }

    return ret;
}

static status_t dac_pcm512x_set_gpio_val(struct dac_pcm512x_state *pcm512x_state, int gpio_nr, int val)
{
    status_t ret = NO_ERROR;
    uint8_t gpio_bit = 1 << (gpio_nr - 1);
    uint8_t gpio_output_reg;

    switch (gpio_nr) {
        case 3:
            gpio_output_reg = PCM512x_GPIO_OUTPUT_3;
            break;
        case 4:
            gpio_output_reg = PCM512x_GPIO_OUTPUT_4;
            break;
        case 6:
            gpio_output_reg = PCM512x_GPIO_OUTPUT_6;
            break;
        default:
            ret = ERR_INVALID_ARGS;
            goto end;
    }

    /* Enable gpio */
    ret = dac_pcm512x_update_bits(pcm512x_state, PCM512x_GPIO_EN, gpio_bit, gpio_bit);

    /* Set pinmux to gpio */
    ret = dac_pcm512x_update_bits(pcm512x_state, gpio_output_reg, 0xf, 2);

    /* Set gpio value */
    ret = dac_pcm512x_update_bits(pcm512x_state, PCM512x_GPIO_CONTROL_1, gpio_bit, val ? gpio_bit : 0);

end:
    return ret;
}

static status_t dac_pcm512x_set_gpio(struct dac_pcm512x_state *pcm512x_state, int gpio_nr)
{
    return dac_pcm512x_set_gpio_val(pcm512x_state, gpio_nr, 1);
}

static status_t dac_pcm512x_clear_gpio(struct dac_pcm512x_state *pcm512x_state, int gpio_nr)
{
    return dac_pcm512x_set_gpio_val(pcm512x_state, gpio_nr, 0);
}

static status_t dac_pcm512x_init(struct device *dev)
{
    int r;
    status_t ret;
    struct dac_pcm512x_state *pcm512x_state = malloc(sizeof(struct dac_pcm512x_state));
    struct device *i2c_device;

    printlk(LK_VERBOSE, "%s\n", __FUNCTION__);

    ASSERT(pcm512x_state);
    memset(pcm512x_state, 0, sizeof(struct dac_pcm512x_state));

    r = of_device_get_int32(dev, "bus-id-i2c", &pcm512x_state->bus_id);
    if (r) {
        printlk(LK_ERR, "bus-id-i2c fdt node property can't be read, aborting!\n");
        free(pcm512x_state);
        ret = ERR_NOT_FOUND;
        goto end;
    }

    r = of_device_get_int32(dev, "reg", &pcm512x_state->i2c_addr);
    if (r) {
        printlk(LK_ERR, "reg fdt node property can't be read, aborting!\n");
        free(pcm512x_state);
        ret = ERR_NOT_FOUND;
        goto end;
    }

    r = of_device_get_int32(dev, "gpio-led", &pcm512x_state->gpio_led);
    if (r) {
        printlk(LK_NOTICE, "gpio-led fdt node property can't be read\n");
        pcm512x_state->gpio_led = 0xffffffff;
    }

    r = of_device_get_int32(dev, "gpio-osc44", &pcm512x_state->gpio_osc44);
    if (r) {
        printlk(LK_ERR, "gpio-osc44 fdt node property can't be read, aborting!\n");
        free(pcm512x_state);
        ret = ERR_NOT_FOUND;
        goto end;
    }

    r = of_device_get_int32(dev, "gpio-osc48", &pcm512x_state->gpio_osc48);
    if (r) {
        printlk(LK_ERR, "gpio-osc48 fdt node property can't be read, aborting!\n");
        free(pcm512x_state);
        ret = ERR_NOT_FOUND;
        goto end;
    }

    printlk(LK_INFO, "Probing dac_pcm512x using i2c bus id: [%d]\n", pcm512x_state->bus_id);

    i2c_device = class_i2c_get_device_by_id(pcm512x_state->bus_id);
    ASSERT(i2c_device != NULL);

    pcm512x_state->i2c_dev = i2c_device;
    pcm512x_state->device = dev;

    dev->state = pcm512x_state;

    ret = dac_pcm512x_mute(pcm512x_state, true);
    if (ret) goto end;

    /* Reset PCM512x */
    ret = dac_pcm512x_write_reg(pcm512x_state,
            PCM512x_RESET,
            PCM512x_RSTM | PCM512x_RSTR);
    if (ret) goto end;
    ret = dac_pcm512x_write_reg(pcm512x_state,
            PCM512x_RESET,
            0);
    if (ret) goto end;

    /* Default to standby mode */
    ret = dac_pcm512x_update_bits(pcm512x_state,
            PCM512x_POWER,
            PCM512x_RQST,
            PCM512x_RQST);

end:
    return ret;
}

status_t dac_pcm512x_get_capabilities(const struct device * dev, uint32_t *capabilities)
{
    *capabilities = DAC_CAP_PKT_PCM;
    return 0;
}

status_t dac_pcm512x_open(struct device *dev)
{
    status_t ret;
    struct dac_pcm512x_state *pcm512x_state = dev->state;

    printlk(LK_VERBOSE, "%s\n", __FUNCTION__);

    /* Turn LED on  */
    if (pcm512x_state->gpio_led != 0xffffffff) {
        ret = dac_pcm512x_set_gpio(pcm512x_state, pcm512x_state->gpio_led);
        if (ret)
            goto end;
    }

    ret = dac_pcm512x_mute(pcm512x_state, false);

end:
    return ret;
}

status_t dac_pcm512x_close(struct device *dev)
{
    status_t ret;
    struct dac_pcm512x_state *pcm512x_state = dev->state;

    printlk(LK_VERBOSE, "%s\n", __FUNCTION__);

    ret = dac_pcm512x_mute(pcm512x_state, true);

    /* Turn LED off */
    if (pcm512x_state->gpio_led != 0xffffffff)
        ret = dac_pcm512x_clear_gpio(pcm512x_state, pcm512x_state->gpio_led);

    /* Turn oscillators off */
    ret = dac_pcm512x_clear_gpio(pcm512x_state, pcm512x_state->gpio_osc48);
    ret = dac_pcm512x_clear_gpio(pcm512x_state, pcm512x_state->gpio_osc44);

    /* Default to standby mode */
    ret = dac_pcm512x_update_bits(pcm512x_state,
            PCM512x_POWER,
            PCM512x_RQST,
            PCM512x_RQST);

    return ret;
}

#define CLK_48000 (48000 * 512)
#define CLK_44100 (44100 * 512)

static int dac_pcm512x_get_dac_rate(int sample_rate)
{
    int dr;

    switch (sample_rate) {
        case 44100:
            dr = 5644800;
            break;
        case 48000:
        case 96000:
        case 192000:
        case 384000:
            dr = 6144000;
            break;
        default:
            printlk(LK_ERR, "%s: unexpected sample rate (%d)\n", __FUNCTION__, sample_rate);
            dr = -1;
            break;
    }

    return dr;
}

static uint8_t dac_pcm512x_get_dac_div(int sample_rate)
{
    int dd;
    int u, d;

    switch (sample_rate) {
        case 44100:
            u = CLK_44100;
            break;
        case 48000:
        case 96000:
        case 192000:
        case 384000:
            u = CLK_48000;
            break;
        default:
            printlk(LK_ERR, "%s: unexpected sample rate (%d)\n", __FUNCTION__, sample_rate);
            dd = -1;
            goto end;
    };

    d = dac_pcm512x_get_dac_rate(sample_rate);
    dd = (u + d / 2) / d;

end:
    return dd;
}

static int dac_pcm512x_get_idac(int sample_rate, uint8_t dsp_div)
{
    int idac;
    int u;

    switch (sample_rate) {
        case 44100:
            u = CLK_44100;
            break;
        case 48000:
        case 96000:
        case 192000:
        case 384000:
            u = CLK_48000;
            break;
        default:
            printlk(LK_ERR, "%s: unexpected sample rate (%d)\n", __FUNCTION__, sample_rate);
            idac = -1;
            goto end;
    }

    idac = u / (sample_rate * dsp_div);

end:
    return idac;
}

static uint8_t dac_pcm512x_get_bclk_div(int sample_rate, uint8_t lrclk_div)
{
    uint8_t bclk_div;
    int u;

    switch (sample_rate) {
        case 44100:
            u = CLK_44100;
            break;
        case 48000:
        case 96000:
        case 192000:
        case 384000:
            u = CLK_48000;
            break;
        default:
            printlk(LK_ERR, "%s: unexpected sample rate (%d)\n", __FUNCTION__, sample_rate);
            bclk_div = 0;
            goto end;
    }

    bclk_div = (uint8_t )(u / (sample_rate * lrclk_div));

end:
    return bclk_div;
}

static uint8_t dac_pcm512x_get_fssp(int sample_rate)
{
    uint8_t fssp;

    switch (sample_rate) {
        case 44100:
        case 48000:
            fssp = PCM512x_FSSP_48KHZ;
            break;
        case 96000:
            fssp = PCM512x_FSSP_96KHZ;
            break;
        case 192000:
            fssp = PCM512x_FSSP_192KHZ;
            break;
        case 384000:
            fssp = PCM512x_FSSP_384KHZ;
            break;
        default:
            printlk(LK_ERR, "%s: unexpected sample rate (%d)\n", __FUNCTION__, sample_rate);
            fssp = 0;
            goto end;
    }

end:
    return fssp;
}

status_t dac_pcm512x_set_format(struct device *dev, dac_audio_hw_params_t *hw_params)
{
    struct dac_pcm512x_state *pcm512x_state = dev->state;
    uint8_t alen, lrclk_div, bclk_div, dsp_div, dac_div, ncp_div, osr_div, fssp;
    int dac_rate, idac;
    int sample_width, sample_rate;
    status_t ret = NO_ERROR;

    printlk(LK_VERBOSE, "%s\n", __FUNCTION__);

    sample_rate = hw_params->rate;
    if ((sample_rate != 44100) && (sample_rate != 48000) &&
            (sample_rate != 96000) && (sample_rate != 192000) &&
            (sample_rate != 384000)) {
        printlk(LK_ERR, "%s: sample_rate %d not supported!\n", __FUNCTION__, sample_rate);
        ret = ERR_INVALID_ARGS;
        goto end;
    }

    /* Turn DAC on */
    ret = dac_pcm512x_update_bits(pcm512x_state,
            PCM512x_POWER,
            PCM512x_RQPD,
            0);

    /* Enable 44.1kHz or 48kHz oscillator depending on sampling rate */
    if (sample_rate % 8000) {
        /* Enable 44.1kHz oscillator on Hifiberry card */
        ret = dac_pcm512x_set_gpio(pcm512x_state, pcm512x_state->gpio_osc44);
    }
    else {
        /* Enable 48kHz oscillator on Hifiberry card */
        ret = dac_pcm512x_set_gpio(pcm512x_state, pcm512x_state->gpio_osc48);
    }

    switch (hw_params->pcm_fmt) {
        case DAC_AUDIO_PCM_FMT_16:
            alen = PCM512x_ALEN_16;
            sample_width = 16;
            break;
        case DAC_AUDIO_PCM_FMT_24:
            alen = PCM512x_ALEN_24;
            sample_width = 24;
            break;
        case DAC_AUDIO_PCM_FMT_32:
            alen = PCM512x_ALEN_32;
            sample_width = 32;
            break;
        default:
            ret = ERR_INVALID_ARGS;
            goto end;
    }

    ret = dac_pcm512x_update_bits(pcm512x_state,
            PCM512x_I2S_1,
            PCM512x_ALEN,
            alen);

    ret = dac_pcm512x_update_bits(pcm512x_state,
            PCM512x_ERROR_DETECT,
            PCM512x_IDFS | PCM512x_IDBK | PCM512x_IDSK | PCM512x_IDCH
            | PCM512x_IDCM | PCM512x_DCAS | PCM512x_IPLK,
            PCM512x_IDFS | PCM512x_IDBK | PCM512x_IDSK | PCM512x_IDCH
            | PCM512x_DCAS | PCM512x_IPLK);

    ret = dac_pcm512x_update_bits(pcm512x_state,
            PCM512x_PLL_EN,
            PCM512x_PLLE,
            0);

    ret= dac_pcm512x_update_bits(pcm512x_state,
            PCM512x_DAC_REF,
            PCM512x_SDAC,
            PCM512x_SDAC_SCK);

    dsp_div = 1; /* mclk < 50Mhz, so no need to divide DSP clk */
    ret = dac_pcm512x_write_reg(pcm512x_state, PCM512x_DSP_CLKDIV, dsp_div - 1);

    dac_rate = dac_pcm512x_get_dac_rate(sample_rate);
    dac_div = dac_pcm512x_get_dac_div(sample_rate);
    ret = dac_pcm512x_write_reg(pcm512x_state, PCM512x_DAC_CLKDIV, dac_div - 1);

    ncp_div = (uint8_t)((dac_rate + 768000) / 1536000);
    ret = dac_pcm512x_write_reg(pcm512x_state, PCM512x_NCP_CLKDIV, ncp_div - 1);

    osr_div = (uint8_t)(dac_rate / (sample_rate * 16));
    ret = dac_pcm512x_write_reg(pcm512x_state, PCM512x_OSR_CLKDIV, osr_div - 1);

    lrclk_div = sample_width * hw_params->num_slots;
    bclk_div = dac_pcm512x_get_bclk_div(sample_rate, lrclk_div);
    ret = dac_pcm512x_write_reg(pcm512x_state, PCM512x_MASTER_CLKDIV_1, bclk_div - 1);
    ret = dac_pcm512x_write_reg(pcm512x_state, PCM512x_MASTER_CLKDIV_2, lrclk_div - 1);

    idac = dac_pcm512x_get_idac(sample_rate, dsp_div);
    ret = dac_pcm512x_write_reg(pcm512x_state, PCM512x_IDAC_1, idac >> 8);
    ret = dac_pcm512x_write_reg(pcm512x_state, PCM512x_IDAC_2, idac & 0xff);

    fssp = dac_pcm512x_get_fssp(sample_rate);
    ret = dac_pcm512x_update_bits(pcm512x_state,
            PCM512x_FS_SPEED_MODE,
            PCM512x_FSSP,
            fssp);

    ret = dac_pcm512x_update_bits(pcm512x_state,
            PCM512x_BCLK_LRCLK_CFG,
            PCM512x_BCKP | PCM512x_BCKO | PCM512x_LRKO,
            PCM512x_BCKO | PCM512x_LRKO);

    ret = dac_pcm512x_update_bits(pcm512x_state,
            PCM512x_MASTER_MODE,
            PCM512x_RLRK | PCM512x_RBCK,
            PCM512x_RLRK | PCM512x_RBCK);

    ret = dac_pcm512x_update_bits(pcm512x_state,
            PCM512x_SYNCHRONIZE,
            PCM512x_RQSY,
            PCM512x_RQSY_HALT);

    ret = dac_pcm512x_update_bits(pcm512x_state,
            PCM512x_SYNCHRONIZE,
            PCM512x_RQSY,
            PCM512x_RQSY_RESUME);

    /* Normal operation mode */
    ret = dac_pcm512x_update_bits(pcm512x_state,
            PCM512x_POWER,
            PCM512x_RQST,
            0);

end:
    return ret;
}

static struct device_class dac_pcm512x_device_class = {
    .name = "dac",
};

static struct dac_ops the_ops = {
    .std = {
        .device_class = &dac_pcm512x_device_class,
        .init = dac_pcm512x_init,
    },
    .open               = dac_pcm512x_open,
    .close              = dac_pcm512x_close,
    .get_capabilities   = dac_pcm512x_get_capabilities,
    .set_format         = dac_pcm512x_set_format,
};

DRIVER_EXPORT_WITH_LVL(dac_pcm512x, &the_ops.std, DRIVER_INIT_HAL);
