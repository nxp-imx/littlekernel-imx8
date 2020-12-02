/*
 * Copyright 2020 NXP
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "ak5558.h"
#include "debug.h"

#ifndef LK
#define _I2C_WRITE_REG(state, addr, reg, val)   ak5558_i2c_write_reg(state->client, addr, reg, val)
#define _I2C_READ_REG(state, addr, reg, val)    ak5558_i2c_read_reg(state->client, addr, reg, val)
#define GPIO_SET_VALUE(pin, state)  gpio_set_value_cansleep(*pin, state)
#define SLEEP_US(x) usleep_range(x,x)
#else
#define _I2C_WRITE_REG(state, addr, reg, val)   adc_ak5558_write_reg(state, reg, val)
#define _I2C_READ_REG(state, addr, reg, val)    adc_ak5558_read_reg(state, reg, val)
#define GPIO_SET_VALUE(pin, state)  gpio_desc_set_value((struct gpio_desc *)pin, state)
#define SLEEP_US(x) udelay(x)
#endif

int adc_write_reg(struct ak5558_state *state,
                uint8_t reg, uint8_t val)
{
    int ret;

    ret = _I2C_WRITE_REG(state, 0x13, reg, val);
    if (ret < 0)
        return ret;

    SLEEP_US(0);
    return 0;
}

int adc_read_reg(struct ak5558_state *state,
                uint8_t reg, uint8_t *val)
{
    int ret;

    ret = _I2C_READ_REG(state, 0x13, reg, val);
    if (ret < 0)
        return ret;

    SLEEP_US(0);
    return 0;
}

status_t _adc_ak5558_dump_registers(struct ak5558_state *state)
{
    uint8_t reg_val;

    if (adc_read_reg(state, AK5558_00_POWER_MANAGEMENT1,
                            &reg_val))
        return ERR_I2C_NACK;
    printlk(LK_NOTICE, "AK5558: PMGMT1 value: 0x%x\n", reg_val);

    if (adc_read_reg(state, AK5558_01_POWER_MANAGEMENT2,
                            &reg_val))
        return ERR_I2C_NACK;
    printlk(LK_NOTICE, "AK5558: PMGMT2 value: 0x%x\n", reg_val);

    if (adc_read_reg(state, AK5558_02_CONTROL1, &reg_val))
        return ERR_I2C_NACK;
    printlk(LK_NOTICE, "AK5558: CONTROL1 value: 0x%x\n", reg_val);

    if (adc_read_reg(state, AK5558_03_CONTROL2, &reg_val))
        return ERR_I2C_NACK;
    printlk(LK_NOTICE, "AK5558: CONTROL2 value: 0x%x\n", reg_val);

    if (adc_read_reg(state, AK5558_04_CONTROL3, &reg_val))
        return ERR_I2C_NACK;
    printlk(LK_NOTICE, "AK5558: CONTROL3 value: 0x%x\n", reg_val);

    if (adc_read_reg(state, AK5558_05_DSD, &reg_val))
        return ERR_I2C_NACK;
    printlk(LK_NOTICE, "AK5558: DSD value: 0x%x\n", reg_val);

    return NO_ERROR;
}

void adc_ak5558_reset(struct ak5558_state *state)
{
    printlk(LK_NOTICE, "Initiating adc_ak5558 power up sequence...\n");

    /*  power off the chip: per specification, 150ns hold time for PDN */
    GPIO_SET_VALUE(&state->pdn_gpio, 0);
    SLEEP_US(10);
    /* no data on i2c control settling time in specification, use 1ms here */
    GPIO_SET_VALUE(&state->pdn_gpio, 1);
    SLEEP_US(1000);
}

status_t _adc_ak5558_open(struct ak5558_state* state)
{
    printlk(LK_NOTICE, "Open adc_ak5558 device...\n");

    /* Reset ADC */
    adc_ak5558_reset(state);
    SLEEP_US(1000);

    return 0;
}

status_t _adc_ak5558_close(struct ak5558_state* state)
{
    printlk(LK_NOTICE, "Close adc_ak5558 device...\n");

    /* Power down the ADC */
    GPIO_SET_VALUE(&state->pdn_gpio, 0);

    return 0;
}

status_t _adc_ak5558_get_capabilities(void)
{
    uint32_t cap = ADC_CAP_PKT_PCM;
    
#if 0
    /* Enable DSD feature when AK5558 is used in master mode */
    cap |= ADC_CAP_PKT_DSD;
#endif

    return cap;
}

status_t _adc_ak5558_set_format(struct ak5558_state* state)
{
    uint8_t reg_val;
    adc_audio_hw_params_t *hw_params = &state->hw_params;

    printlk(LK_INFO, "%s: ---> pcm fmt: %x, fmt: %x, pkt: %x, num_ch: %d, rate: %d\n",
                    __func__, hw_params->pcm_fmt, hw_params->fmt, hw_params->pkt, hw_params->num_ch, hw_params->rate);


    /* Read POWER MANAGEMENT 1 registers */
    if (adc_read_reg(state, AK5558_00_POWER_MANAGEMENT1, &reg_val))
        goto err;

    /* Power down all channels */
    reg_val &= (uint8_t)~AK5558_PWRMGMT1_PW_MASK;

    /* Write-back POWER MANAGEMENT 1 register */
    if (adc_write_reg(state, AK5558_00_POWER_MANAGEMENT1, reg_val))
        goto err;

#if defined(AK5558_DRIVER_CHANNEL_SUMMATION)
    /* Force RSTN to 0 */
    reg_val = 0;
    if (adc_write_reg(state, AK5558_01_POWER_MANAGEMENT2, reg_val))
        goto err;

#if (AK5558_DRIVER_CHANNEL_SUMMATION == AK5558_DRIVER_CHANNEL_SUMMATION_8TO2)
    /* Enable 8-to-2 channels summation mode */
    reg_val = (1 << 1);
#elif (AK5558_DRIVER_CHANNEL_SUMMATION == AK5558_DRIVER_CHANNEL_SUMMATION_8TO4)
    /* Enable 8-to-4 channels summation mode */
    reg_val = (2 << 1);
#elif (AK5558_DRIVER_CHANNEL_SUMMATION == AK5558_DRIVER_CHANNEL_SUMMATION_8TO1)
    /* Enable 8-to-1 channels summation mode */
    reg_val = (3 << 1);
#else
#error "AK5558 channel summation configuration not supported"
#endif //AK5558_DRIVER_CHANNEL_SUMMATION

    /* Force RSTN to 1 */
    reg_val |= 1;

    /* Write POWER MANAGEMENT 2 register */
    if (adc_write_reg(state, AK5558_01_POWER_MANAGEMENT2, reg_val))
        goto err;
#endif //defined(AK5558_DRIVER_CHANNEL_SUMMATION)

    /* Read CONTROL1 register */
    if (adc_read_reg(state, AK5558_02_CONTROL1, &reg_val))
        goto err;

    /* Set data output mode (L/R justified, I2S,...) */
    reg_val &= ~AK5558_CTRL1_DIF_MASK;
    switch (state->hw_params.fmt) {
    case ADC_AUDIO_FMT_I2S:
        reg_val |= AK5558_CTRL1_DIF_I2S_MODE;
        break;
    case ADC_AUDIO_FMT_LEFT_J:
        reg_val |= AK5558_CTRL1_DIF_MSB_MODE;
        break;
    default:
        goto err;
    }

    /* Set sampling rate */
    switch (state->hw_params.rate) {
    case 32000:
    case 44100:
    case 48000:
        /* MCLK = 512fs */
        reg_val |= AK5558_CTRL1_CKS_512FS_48KHZ;
        break;
    case 88200:
    case 96000:
        /* MCLK = 256fs */
        reg_val |= AK5558_CTRL1_CKS_256FS_96KHZ;
        break;
    case 176400:
    case 192000:
        /* MCLK = 128 */
        reg_val |= AK5558_CTRL1_CKS_128FS_192KHZ;
        break;
    default:
        printlk(LK_ERR, "ERROR: ADC sampling rate %d not supported.\n", state->hw_params.rate);
        goto err;
    }

    /* Check resolution (only 24-bits or 32-bits) */
    switch(state->hw_params.pcm_fmt) {
    case ADC_AUDIO_PCM_FMT_24:
        reg_val |= AK5558_CTRL1_DIF_24BIT_MODE;
        break;
    case ADC_AUDIO_PCM_FMT_32:
        reg_val |= AK5558_CTRL1_DIF_32BIT_MODE;
        break;
    default:
        goto err;
    }

    /* Write-back CONTROL1 register */
    if (adc_write_reg(state, AK5558_02_CONTROL1, reg_val))
        goto err;


    /* Read CONTROL2 register */
    if ( adc_read_reg(state, AK5558_03_CONTROL2, &reg_val) )
        goto err;

    /* Write-back CONTROL2 register */
    if ( adc_write_reg(state, AK5558_03_CONTROL2, reg_val) )
        goto err;

    /* Read CONTROL3 register */
    if (adc_read_reg(state, AK5558_04_CONTROL3, &reg_val))
        goto err;

    /* Set packet mode */
    reg_val &= ~AK5558_CTRL3_DP_MASK;
    switch (state->hw_params.pkt) {
    case ADC_AUDIO_PKT_PCM:
        reg_val |= AK5558_CTRL3_DP_PCM;
        break;
    case ADC_AUDIO_PKT_DSD:
        reg_val |= AK5558_CTRL3_DP_DSD;
        break;
    default:
        goto err;
    }

    /* Write-back CONTROL3 register */
    if (adc_write_reg(state, AK5558_04_CONTROL3, reg_val))
        goto err;

    /* Read POWER MANAGEMENT 1 registers */
    if (adc_read_reg(state, AK5558_00_POWER_MANAGEMENT1, &reg_val))
        goto err;

    /* Power up all channels */
    reg_val |= AK5558_PWRMGMT1_PW_MASK;

    /* Write-back POWER MANAGEMENT 1 register */
    if (adc_write_reg(state, AK5558_00_POWER_MANAGEMENT1, reg_val))
        goto err;

    /* Dump for debug */
    (void) _adc_ak5558_dump_registers(state);

    return 0;

err: 
    return -1;
}