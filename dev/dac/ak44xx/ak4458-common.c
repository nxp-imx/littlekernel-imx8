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

#include "ak4458.h"
#include "debug.h"

#ifdef LK
#define GPIO_SET_VALUE(pin, state)  gpio_desc_set_value((struct gpio_desc *)pin, state)
#define SLEEP_US(x) udelay(x)
#else
#define GPIO_SET_VALUE(pin, state)  gpio_set_value_cansleep(*pin, state)
#define SLEEP_US(x) usleep_range(x,x)
#define dac_ak4458_write_reg(x,y,z) dac_i2c_write_reg(x->client, y, z)
#endif

void dac_ak4458_reset(struct dac_ak4458_state *state)
{
    printlk(LK_INFO,"Initiating dac_ak4458 power up sequence...\n");
    /*  power off the chip */
    GPIO_SET_VALUE(&state->pdn_gpio, 0);
    SLEEP_US(1000);
    GPIO_SET_VALUE(&state->pdn_gpio, 1);
    SLEEP_US(500);
    printlk(LK_INFO,"Dac_ak4458 power up ok!\n");

    /* Reset cached status */
    memset(&state->hw_params_cached_status, 0x00, sizeof(dac_audio_hw_params_t));
}

status_t hw_params_cached_status_check(struct dac_ak4458_state *state, dac_audio_hw_params_t *format)
{
    if ((state->hw_params_cached_status.pcm_fmt == format->pcm_fmt) &&
        (state->hw_params_cached_status.fmt == format->fmt) &&
        (state->hw_params_cached_status.pkt == format->pkt) &&
        (state->hw_params_cached_status.num_ch == format->num_ch))
        return 0;

    printlk(LK_INFO, "DAC change format: pcm_fmt: %d --> %d, format: %d --> %d, pkt: %d --> %d, num_ch: %d --> %d\n",
        state->hw_params_cached_status.pcm_fmt, format->pcm_fmt,
        state->hw_params_cached_status.fmt, format->fmt,
        state->hw_params_cached_status.pkt, format->pkt,
        state->hw_params_cached_status.num_ch, format->num_ch);

    memcpy(&state->hw_params_cached_status, format, sizeof(dac_audio_hw_params_t));
    return 1;
}

status_t _dac_ak4458_open(struct dac_ak4458_state *state)
{
    dac_ak4458_reset(state);
    SLEEP_US(1000);

    return 0;
}

status_t _dac_ak4458_close(struct dac_ak4458_state *state)
{
    return 0;
}

status_t _dac_get_capabilities(uint32_t *capabilities)
{
    *capabilities = DAC_CAP_PKT_PCM;
    return 0;
}

status_t _dac_ak4458_set_format(struct dac_ak4458_state *state,
                                            dac_audio_hw_params_t *hw_params)
{
    uint8_t reg;
    uint8_t tdm_reg = 0x0D;

    /* Leave if format didn't change */
    if (!hw_params_cached_status_check(state, hw_params))
        return 0;

    reg =  AK4458_CTRL1_ACKS | AK4458_CTRL1_RSTN;
    if (hw_params->pkt != DAC_AUDIO_PKT_PCM)
        return ERR_INVALID_ARGS;

    printlk(LK_DEBUG,
            "Dac_ak4458 Assert audio interface RESET while DAC is configured...\n");
    dac_ak4458_write_reg(state, AK4458_00_CONTROL1, 0);

    switch(hw_params->pcm_fmt) {
    case DAC_AUDIO_PCM_FMT_16:
        if (hw_params->fmt == DAC_AUDIO_FMT_I2S)
            reg |= AK4458_CTRL1_DIF_24BIT_I2S;
        else
            reg |= AK4458_CTRL1_DIF_16BIT_LSB;
        break;
    case DAC_AUDIO_PCM_FMT_24:
        if (hw_params->fmt == DAC_AUDIO_FMT_I2S)
            reg |= AK4458_CTRL1_DIF_24BIT_I2S;
        else if (hw_params->fmt == DAC_AUDIO_FMT_LEFT_J)
            reg |= AK4458_CTRL1_DIF_24BIT_MSB;
        else if (hw_params->fmt == DAC_AUDIO_FMT_RIGHT_J)
            reg |= AK4458_CTRL1_DIF_24BIT_LSB;
        else
            return ERR_INVALID_ARGS;

        break;
    case DAC_AUDIO_PCM_FMT_32:
        if (hw_params->fmt == DAC_AUDIO_FMT_I2S)
            reg |= AK4458_CTRL1_DIF_32BIT_I2S;
        else if (hw_params->fmt == DAC_AUDIO_FMT_LEFT_J)
            reg |= AK4458_CTRL1_DIF_32BIT_MSB;
        else if (hw_params->fmt == DAC_AUDIO_FMT_RIGHT_J)
            reg |= AK4458_CTRL1_DIF_32BIT_LSB;
        else
            return ERR_INVALID_ARGS;
        break;
    default:
            return ERR_INVALID_ARGS;
    }

    switch(hw_params->num_slots) {
    case 2:
        printlk(LK_DEBUG, "Dac_ak4458 TDM not used\n");
        /* TDM1-0 = 00b */
        break;
    case 4:
        printlk(LK_DEBUG, "Dac_ak4458 TDM128 used\n");
        tdm_reg |= 0x40; /* TDM1-0 = 01b */
        break;
    case 8:
        printlk(LK_DEBUG, "Dac_ak4458 TDM256 used\n");
        tdm_reg |= 0x80; /* TDM1-0 = 10b */
        break;
    case 16:
        printlk(LK_DEBUG, "Dac_ak4458 TDM512 used\n");
        tdm_reg |= 0xC0; /* TDM1-0 = 11b */
        break;
    default:
        printlk(LK_WARNING,
                "Dac_ak4458 TDM scheme not supported for %d slots\n",
                hw_params->num_slots);
        break;
    }

    dac_ak4458_write_reg(state, AK4458_01_CONTROL2, 0x02);
    dac_ak4458_write_reg(state, AK4458_0A_CONTROL6, tdm_reg);

    /* Bring audio interface out of RESET */
    dac_ak4458_write_reg(state, AK4458_00_CONTROL1, reg);

    printlk(LK_DEBUG, "Dac_ak4458 Audio interface RESET is now released.\n");

#ifdef LK
    state->hw_params = *hw_params;
#endif

    return 0;
}
