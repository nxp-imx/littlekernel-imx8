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

#ifdef LK
#include <platform/interrupts.h>
#include <err.h>
#include <delay.h>
#define delay_us udelay
#include <malloc.h>
#include <string.h>

#include "dev/i2c.h"
#include <dev/class/i2c.h>
#include <dev/class/gpio.h>
#include <dev/class/dac.h>
#include <lib/appargs.h>

#else

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/compiler.h>
#include "rpmsg-rpc.h"
#endif

#define DRIVER_NAME "dac_ak4458"
#define AK4458_00_CONTROL1          0x00
#define AK4458_01_CONTROL2          0x01
#define AK4458_02_CONTROL3          0x02
#define AK4458_03_LCHATT            0x03
#define AK4458_04_RCHATT            0x04
#define AK4458_05_CONTROL4          0x05
#define AK4458_06_DSD1              0x06
#define AK4458_07_CONTROL5          0x07
#define AK4458_08_SOUND_CONTROL     0x08
#define AK4458_09_DSD2              0x09
#define AK4458_0A_CONTROL6          0x0A
#define AK4458_0B_CONTROL7          0x0B
#define AK4458_0C_CONTROL8          0x0C
#define AK4458_0D_CONTROL9          0x0D
#define AK4458_0E_CONTROL10         0x0E
#define AK4458_0F_L2CHATT           0x0F
#define AK4458_10_R2CHATT           0x10
#define AK4458_11_L3CHATT           0x11
#define AK4458_12_R3CHATT           0x12
#define AK4458_13_L4CHATT           0x13
#define AK4458_14_R4CHATT           0x14

#define DAC_AK4458_1_DAC_I2C_ADDR_7BIT 0x10U
#define DAC_AK4458_2_DAC_I2C_ADDR_7BIT 0x12U
#define DAC_AK4458_3_DAC_I2C_ADDR_7BIT 0x11U

/* Control 1 register */
#define AK4458_CTRL1_ACKS             (0x80)
#define AK4458_CTRL1_DIF_16BIT_LSB	(0 << 1)
#define AK4458_CTRL1_DIF_20BIT_LSB	(1 << 1)
#define AK4458_CTRL1_DIF_24BIT_MSB	(2 << 1)
#define AK4458_CTRL1_DIF_24BIT_I2S	(3 << 1)
#define AK4458_CTRL1_DIF_24BIT_LSB	(4 << 1)
#define AK4458_CTRL1_DIF_32BIT_LSB	(5 << 1)
#define AK4458_CTRL1_DIF_32BIT_MSB	(6 << 1)
#define AK4458_CTRL1_DIF_32BIT_I2S	(7 << 1)
#define AK4458_CTRL1_RSTN             (0x1)


#ifndef LK
#define DAC_CAP_PKT_PCM                (1UL << 0)
#define DAC_CAP_PKT_DSD                (1UL << 1) /* Not supported */

#define ERR_INVALID_ARGS -1

typedef int status_t;

int dac_i2c_write_reg(struct i2c_client *client,
			uint8_t reg, uint8_t val);
#endif

#ifndef LK
/**
 * @typedef hdmi_audio_pkt_t
 *   Audio packet type.
 */
typedef enum _dac_audio_pkt_e {
	/*! PCM audio packets */
	DAC_AUDIO_PKT_PCM,
	/*! DSD audio packets */
	DAC_AUDIO_PKT_DSD,
} dac_audio_pkt_t;

typedef enum _dac_audio_fmt_e {
	DAC_AUDIO_FMT_I2S,
	DAC_AUDIO_FMT_LEFT_J,
	DAC_AUDIO_FMT_RIGHT_J,
	DAC_AUDIO_FMT_DSP_A,
	DAC_AUDIO_FMT_DSP_B,
	DAC_AUDIO_FMT_AC97,
	DAC_AUDIO_FMT_PDM,
} dac_audio_fmt_t;

typedef enum _dac_audio_pcm_format_e {
	DAC_AUDIO_PCM_FMT_16,
	DAC_AUDIO_PCM_FMT_24,
	DAC_AUDIO_PCM_FMT_32,
} dac_audio_pcm_format_t;

typedef struct dac_audio_hw_params_s {
	dac_audio_pcm_format_t pcm_fmt;
	dac_audio_fmt_t fmt;
	dac_audio_pkt_t pkt;
	unsigned num_ch;
	unsigned num_slots;
} dac_audio_hw_params_t;


struct rpc_dac_s_format_s {
	dac_audio_pcm_format_t pcm_fmt;
	dac_audio_fmt_t fmt;
	dac_audio_pkt_t pkt;
	unsigned num_ch;
	unsigned num_slots;
};
#endif

#ifdef LK
struct dac_ak4458_state {
    struct device *device;
    struct device *i2c_dev;
    int bus_id;
    struct gpio_desc pdn_gpio;
    dac_audio_hw_params_t hw_params;
    dac_audio_hw_params_t hw_params_cached_status;
};
#else
struct dac_ak4458_state {
	struct i2c_client *client;
	struct regmap *regmap;
	unsigned int pdn_gpio;
	struct rpmsg_rpc_dev *rpcdev;
	dac_audio_hw_params_t hw_params_cached_status;
    unsigned ept_id;
};
#endif


status_t dac_ak4458_write_reg(struct dac_ak4458_state *state, uint8_t reg, uint8_t val);

#ifndef LK
int dac_i2c_write_reg(struct i2c_client *client,
            uint8_t reg, uint8_t val);
#endif

void dac_ak4458_reset(struct dac_ak4458_state *state);
status_t _dac_ak4458_open(struct dac_ak4458_state *state);
status_t _dac_ak4458_close(struct dac_ak4458_state *state);
status_t _dac_get_capabilities(uint32_t *capabilities);
status_t _dac_ak4458_set_format(struct dac_ak4458_state *state,
                                            dac_audio_hw_params_t *hw_params);

