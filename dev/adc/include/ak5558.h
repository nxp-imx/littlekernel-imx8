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
#include <dev/class/adc.h>
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

/* Constants */
#define AK5558_DRIVER_CHANNEL_SUMMATION_8TO2 (1)
#define AK5558_DRIVER_CHANNEL_SUMMATION_8TO4 (2)
#define AK5558_DRIVER_CHANNEL_SUMMATION_8TO1 (3)

/* If defined, AK5558_DRIVER_CHANNEL_SUMMATION must use one of the above values */
//#define AK5558_DRIVER_CHANNEL_SUMMATION AK5558_DRIVER_CHANNEL_SUMMATION_8TO2

/* I2C address */
#define ADC_AK5558_ADC_I2C_ADDR_7BIT    (0x13U)

/*** Registers map ***/
#define AK5558_00_POWER_MANAGEMENT1    0x00
#define AK5558_01_POWER_MANAGEMENT2    0x01
#define AK5558_02_CONTROL1             0x02
#define AK5558_03_CONTROL2             0x03
#define AK5558_04_CONTROL3             0x04
#define AK5558_05_DSD                  0x05

/*** Register bit fields ***/
/* Power Management 1 register */
#define AK5558_PWRMGMT1_PW_MASK         (0xFF)

/* Control 1 register */
#define AK5558_CTRL1_DIF_MASK           (3 << 1)
#define AK5558_CTRL1_DIF_MSB_MODE       (0 << 1)
#define AK5558_CTRL1_DIF_I2S_MODE       (1 << 1)
#define AK5558_CTRL1_DIF_24BIT_MODE     (0 << 2)
#define AK5558_CTRL1_DIF_32BIT_MODE     (1 << 2)
#define AK5558_CTRL1_CKS_MASK           (0xf << 3)
#define AK5558_CTRL1_CKS_128FS_192KHZ   (0 << 3)
#define AK5558_CTRL1_CKS_192FS_192KHZ   (1 << 3)
#define AK5558_CTRL1_CKS_256FS_48KHZ    (2 << 3)
#define AK5558_CTRL1_CKS_256FS_96KHZ    (3 << 3)
#define AK5558_CTRL1_CKS_384FS_96KHZ    (4 << 3)
#define AK5558_CTRL1_CKS_384FS_48KHZ    (5 << 3)
#define AK5558_CTRL1_CKS_512FS_48KHZ    (6 << 3)
#define AK5558_CTRL1_CKS_768FS_48KHZ    (7 << 3)
#define AK5558_CTRL1_CKS_64FS_384KHZ    (8 << 3)
#define AK5558_CTRL1_CKS_32FS_768KHZ    (9 << 3)
#define AK5558_CTRL1_CKS_96FS_384KHZ    (10 << 3)
#define AK5558_CTRL1_CKS_48FS_768KHZ    (11 << 3)
#define AK5558_CTRL1_CKS_64FS_768KHZ    (12 << 3)
#define AK5558_CTRL1_CKS_1024FS_16KHZ   (13 << 3)
#define AK5558_CTRL1_CKS_AUTO           (15 << 3)

/* Control 2 register */
#define AK5558_CTRL2_TDM_MASK           (3 << 5)
#define AK5558_CTRL2_TDM_NORMAL         (0 << 5)
#define AK5558_CTRL2_TDM_TDM128         (1 << 5)
#define AK5558_CTRL2_TDM_TDM256         (2 << 5)
#define AK5558_CTRL2_TDM_TDM512         (3 << 5)

/* Control 3 register */
#define AK5558_CTRL3_SLOW_MASK          (1 << 0)
#define AK5558_CTRL3_SLOW_SHARP         (0 << 0)
#define AK5558_CTRL3_SLOW_SLOW          (1 << 0)
#define AK5558_CTRL3_SD_MASK            (1 << 1)
#define AK5558_CTRL3_SD_NORMAL          (0 << 1)
#define AK5558_CTRL3_SD_SHORT           (1 << 1)
#define AK5558_CTRL3_DP_MASK            (1 << 7)
#define AK5558_CTRL3_DP_PCM             (0 << 7)
#define AK5558_CTRL3_DP_DSD             (1 << 7)

#ifndef LK
typedef int status_t;
/**
 * @typedef hdmi_audio_pkt_t
 *   Audio packet type.
 */
typedef enum _adc_audio_pkt_e {
    /*! PCM audio packets */
    ADC_AUDIO_PKT_PCM,
    /*! DSD audio packets */
    ADC_AUDIO_PKT_DSD,
} adc_audio_pkt_t;

typedef enum _adc_audio_fmt_e {
    ADC_AUDIO_FMT_I2S,
    ADC_AUDIO_FMT_LEFT_J,
    ADC_AUDIO_FMT_RIGHT_J,
    ADC_AUDIO_FMT_DSP_A,
    ADC_AUDIO_FMT_DSP_B,
    ADC_AUDIO_FMT_AC97,
    ADC_AUDIO_FMT_PDM,
} adc_audio_fmt_t;

typedef enum _adc_audio_pcm_format_e {
    ADC_AUDIO_PCM_FMT_16,
    ADC_AUDIO_PCM_FMT_24,
    ADC_AUDIO_PCM_FMT_32,
} adc_audio_pcm_format_t;

typedef struct adc_audio_hw_params_s {
    adc_audio_pcm_format_t pcm_fmt;
    adc_audio_fmt_t fmt;
    adc_audio_pkt_t pkt;
    unsigned num_ch;
    unsigned rate;
} adc_audio_hw_params_t;

struct rpc_adc_s_format_s {
    adc_audio_pcm_format_t pcm_fmt;
    adc_audio_fmt_t fmt;
    adc_audio_pkt_t pkt;
    unsigned num_ch;
    unsigned rate;
};
#endif

#ifdef LK
struct ak5558_state {
    struct device *device;
    struct device *i2c_dev;
    int bus_id;
    struct gpio_desc pdn_gpio;
    adc_audio_hw_params_t hw_params;
};
#else
struct ak5558_state {
    struct i2c_client *client;
    unsigned int pdn_gpio;
    adc_audio_hw_params_t hw_params;
    struct rpmsg_rpc_dev *rpcdev;
    unsigned ept_id;
};
#endif

#ifndef LK
#define ADC_CAP_PKT_PCM                (1UL << 0)
#define ADC_CAP_PKT_DSD                (1UL << 1) /* Not supported */

#define ERR_I2C_NACK -1
#define NO_ERROR 0
#endif


status_t _adc_ak5558_dump_registers(struct ak5558_state *state);
void adc_ak5558_reset(struct ak5558_state *state);
status_t _adc_ak5558_open(struct ak5558_state* state);
status_t _adc_ak5558_close(struct ak5558_state* state);
status_t _adc_ak5558_get_capabilities(void);
status_t _adc_ak5558_set_format(struct ak5558_state* state);
status_t adc_ak5558_read_reg(struct ak5558_state *state, uint8_t reg,
                                    uint8_t *val);
status_t adc_ak5558_write_reg(struct ak5558_state *state,
                                     uint8_t reg, uint8_t val);
int adc_read_reg(struct ak5558_state *state,
                uint8_t reg, uint8_t *val);
int adc_write_reg(struct ak5558_state *state,
                uint8_t reg, uint8_t val);
#ifndef LK
int ak5558_i2c_read_reg(struct i2c_client *client, uint8_t addr, uint8_t reg,
                                    uint8_t *val);
int ak5558_i2c_write_reg(struct i2c_client *client, uint8_t addr, uint8_t reg,
                                    uint8_t val);
#endif
