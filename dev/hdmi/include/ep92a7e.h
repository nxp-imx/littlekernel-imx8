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
#include "platform/interrupts.h"
#include <kernel/spinlock.h>
#include <dev/class/gpio.h>
#include <dev/class/hdmi.h>
#include <dev/i2c.h>
#include <delay.h>
#include "debug.h"
#include <lib/appargs.h>
#include <string.h>
/* FIXME: Remove the pca6416 include */
#include "dev/pca6416.h"

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


/**********************************
 ********* DEFINE SECTION *********
 **********************************/
#ifndef LK
/* Linux */
#define SLEEP_US(x) usleep_range(x,x)
#define DEBUG_ASSERT(cond) BUG_ON(!(cond))
#define ASSERT(cond) BUG_ON(!(cond))
#define ERR_BUSY -EBUSY
#define _I2C_WRITE_REG(bus, addr, reg, val) \
            ep92a7e_i2c_write_reg(bus, addr, reg, val)
#define _I2C_READ_REG(bus, addr, reg, val) \
            ep92a7e_i2c_read_reg(bus, addr, reg, val)
#define _I2C_READ_REGS(bus, addr, reg, array, cnt) \
            raw_i2c_read_block(bus, addr, reg, val, cnt)
#define _I2C_WRITE_REGS(bus, addr, reg, array, cnt) \
            raw_i2c_write_block(bus, addr, reg, val, cnt)
#define CALLBACK(state, event_id, data_p, data_size) \
            rpmsg_rpc_call(state->rpcdev, RPC_HDMI_HANDLE_EVENT_ID, data_p, data_size, NULL, NULL)
#define GPIO_GET_VALUE(pin) \
            gpio_get_value_cansleep(*pin)
#define GPIO_SET_VALUE(pin, state) \
            gpio_set_value_cansleep(*pin, state)

typedef int status_t;

#else
/* LK */
#define _I2C_WRITE_REG(bus, addr, reg, val) \
            i2c_write_reg(*bus, address, reg, val)
#define _I2C_READ_REG(bus, addr, reg, val) \
            i2c_read_reg(*bus, address, reg, val)
#define _I2C_READ_REGS(bus, addr, reg, array, cnt) \
            i2c_read_reg_bytes(bus, addr, reg, val, cnt)
#define _I2C_WRITE_REGS(bus, addr, reg, array, cnt) \
            i2c_write_reg_bytes(bus, addr, reg, val, cnt)
#define CALLBACK(state, event_id, data_p, data_size) \
            state->cb(event_id, data_p.data, state->cb_cookie)
#define GPIO_GET_VALUE(pin) \
            gpio_desc_get_value((struct gpio_desc *)pin)
#define GPIO_SET_VALUE(pin, state) \
            gpio_desc_set_value((struct gpio_desc *)pin, state)
#define SLEEP_US(x) udelay(x)

#endif

#define HDMI_CAP_AUDIO_SAMPLE_RATE_CHANGE   (1UL << 0)
#define HDMI_CAP_AUDIO_STREAM_TYPE_CHANGE   (1UL << 2)
#define HDMI_CAP_AUDIO_CHANNEL_STATUS       (1UL << 3)
#define HDMI_CAP_AUDIO_LINK_CHANGE          (1UL << 4)
#define HDMI_CAP_AUDIO_INFOFRAME            (1UL << 5)
#define HDMI_CAP_AUDIO_LAYOUT_CHANGE        (1UL << 6)
#define HDMI_CAP_AUDIO_FMT_60958            (1UL << 24)
#define HDMI_CAP_AUDIO_FMT_61937            (1UL << 25)
#define HDMI_CAP_AUDIO_FMT_CUSTOM           (1UL << 26)


/**********************************
 ******* Common variables *********
 **********************************/
#define EP_I2C_XFER_WAIT 3000

/* General i2c_bus type */
#ifdef LK
typedef int i2c_bus;
#else
typedef struct i2c_client i2c_bus;
#endif

#define osa_time_delay thread_sleep
/*
mclk_fs_n[2:0]  ->  Description
0x00    ->  128fs
0x01    ->  256fs
0x02    ->  384fs
0x03    ->  512fs
0x04    ->  640fs
0x05    ->  768fs
0x06    ->  Not Valid
0x07    ->  Not Valid
*/
enum mclk_fs_n {
    mclk_128fs = 0x00,
    mclk_256fs = 0x01,
    mclk_384fs = 0x02,
    mclk_512fs = 0x03,
    mclk_640fs = 0x04,
    mclk_768fs = 0x05
};

/*
cs_data[27:24]  ->  Description
0x00    ->  44.1kHz
0x02    ->  48kHz
0x03    ->  32kHz
0x08    ->  88.2kHz
0x0A    ->  96kHz
0X0C    ->  176_4kHz
0x0E    ->  192kHz
*/
enum cs_data {
    EP92A7E_SR44_1KHz = 0x00, /*!< Sample rate 44100 Hz */
    EP92A7E_SR48KHz = 0x02, /*!< Sample rate 48000 Hz */
    EP92A7E_SR32KHz = 0x03, /*!< Sample rate 32000 Hz */
    EP92A7E_SR88_2KHz = 0x08, /*!< Sample rate 88200 Hz */
    EP92A7E_SR96KHz = 0x0A, /*!< Sample rate 96000 Hz */
    EP92A7E_SR176_4KHz = 0x0C, /*!< Sample rate 176400 Hz */
    EP92A7E_SR192KHz = 0x0E, /*!< Sample rate 192000 Hz */
    EP92A7E_SR768KHz = 0x09, /*!< Sample rate 768000 Hz */
};

/*! @brief Audio sample rate */
enum ep92a7e_sample_rate {
    sr_44_1KHz = 44100U,
    sr_48KHz = 48000U,
    sr_32KHz = 32000U,
    sr_88_2KHz = 88200U,
    sr_96KHz = 96000U,
    sr_176_4KHz = 176400U,
    sr_192KHz = 192000U
};

#ifndef LK
/**
 * @typedef iec60958_custom_fmt_layout_t
 *  A type describing a custom iec60958 format layout
 *  A negative value should be considered as invalid.
 */
typedef struct iec60958_custom_fmt_layout_s {
    /*! Bit position of data LSB */
    char data_start;
    /*! Data length */
    char data_length;
    /*! Bit position of validity bit */
    char validity_bit;
    /*! Bit position of parity bit */
    char parity_bit;
    /*! Bit position of user bit */
    char user_bit;
    /*! Bit position of channel status bit */
    char channel_status_bit;
    /*! Bit position of block start bit */
    char block_start_bit;
    /*! Occupied width of complete frame */
    char width;
} iec60958_custom_fmt_layout_t;

/**
 * @typedef iec60958_channel_status_t
 *  A type describing the channel status as described by IEC60958-3
 */

struct iec60958_channel_status_s {
    /*! Professional use of channel status block if set, consumer otherwise */
    uint32_t professional:1;
    /*! Main data field represents linear PCM samples if unset, compressed
     * otherwise*/
    uint32_t lpcm:1;
    /*! Copyright protection asserted */
    uint32_t copyright:1;
    /*! Pre-emphasis */
    uint32_t pre_emphasis:3;
    /*! mode */
    uint32_t mode:2;
    /*! Category code */
    uint8_t category;
    /*! Source number */
    uint32_t source:4;
    /*! Channel number */
    uint32_t channel:4;
    /*! Sampling frequency */
    uint32_t sampling_frequency:4;
    /*! Clock accuracy */
    uint32_t clock_accuracy:2;
    /*! Sampling frequency extenstion */
    uint32_t sampling_frequency_extended:2;
    /*! Maximal word length */
    uint32_t max_word_length:1;
    /*! Sample length */
    uint32_t word_length:3;
    /*! Original Sampling frequency */
    uint32_t original_sampling_frequency:4;
    /*! CGMS-A */
    uint32_t cgms_a:2;
    /*! CGMS-A validity */
    uint32_t cgms_a_validity:1;
    /*! Audio sampling frequency coefficient */
    uint32_t audio_sampling_coefficient:4;
    /*! Hidden information */
    uint32_t hidden:1;
    /*! General channel assignement for A channel */
    uint32_t channel_A_number:6;
    /*! General channel assignement for B channel */
    uint32_t channel_B_number:6;
    uint32_t reserved_1:3;
    uint8_t reserved_2[16];
};

typedef union iec60958_channel_status_u {
    struct iec60958_channel_status_s cs;
    uint8_t raw[24];
} iec60958_channel_status_t;

typedef struct hdmi_channel_status_mask_s {
    uint32_t professional:1;
    uint32_t lpcm:1;
    uint32_t copyright:1;
    uint32_t pre_emphasis:1;
    uint32_t mode:1;
    uint32_t category:1;
    uint32_t source:1;
    uint32_t channel:1;
    uint32_t sampling_frequency:1;
    uint32_t clock_accuracy:1;
    uint32_t sampling_frequency_extended:1;
    uint32_t max_word_length:1;
    uint32_t word_length:1;
    uint32_t original_sampling_frequency:1;
    uint32_t cgms_a:1;
    uint32_t cgms_a_validity:1;
    uint32_t audio_sampling_coefficient:1;
    uint32_t hidden:1;
    uint32_t channel_A_number:1;
    uint32_t channel_B_number:1;
} hdmi_channel_status_mask_t;

typedef struct hdmi_channel_status_s {
    iec60958_channel_status_t channel_status;
    hdmi_channel_status_mask_t mask;
} hdmi_channel_status_t;

/**
 * @typedef hdmi_audio_direction_t
 *   A type indicating the audio stream direction
 */
typedef enum _hdmi_audio_direction_e {
    /*! Audio samples flowing to HDMI switch device */
    HDMI_AUDIO_SOURCE = 0U,
    /*! Audio samples flowing from HDMI switch device */
    HDMI_AUDIO_SINK = 1U,
} hdmi_audio_direction_t;

/**
 * @typedef hdmi_audio_if_t
 *   Physical interfaces transporting audio samples.
 */
typedef enum _hdmi_audio_if_e {
    /*! No interface */
    HDMI_AUDIO_IF_NONE,
    /*! HDMI interface */
    HDMI_AUDIO_IF_HDMI,
    /*! SPDIF interface */
    HDMI_AUDIO_IF_SPDIF,
    /*! ARC interface */
    HDMI_AUDIO_IF_ARC,
    /*! I2S interface */
    HDMI_AUDIO_IF_I2S,
    /*! EARC interface */
    HDMI_AUDIO_IF_EARC,
} hdmi_audio_if_t;

/**
 * @typedef hdmi_audio_pkt_t
 *   Audio packet type.
 */
typedef enum _hdmi_audio_pkt_e {
    /*! No Audio packets */
    HDMI_AUDIO_PKT_NONE = 0,
    /*! Standard audio packets LCPM or 60958/61937 */
    HDMI_AUDIO_PKT_STD,
    /*! HBR packets LCPM or 60958/61937 */
    HDMI_AUDIO_PKT_HBR,
    /*! Direct Stream Digital packets */
    HDMI_AUDIO_PKT_DSD,
    /*! Direct Stream Transfer packets */
    HDMI_AUDIO_PKT_DST,
    /*! Invalid audio packet */
    HDMI_AUDIO_PKT_INVALID,
} hdmi_audio_pkt_t;

/**
 * @typedef hdmi_audio_pkt_t
 *   Audio packet type as defined by HDMI spec 1.4 table 7.6
 */
typedef enum _hdmi_audio_pkt_layout_e {
    /*! Audio packet layout 0 - up to 2 channels */
    HDMI_AUDIO_PKT_LAYOUT_0_2CH = 0,
    /*! Audio packet layout 1 - up to 8 channels */
    HDMI_AUDIO_PKT_LAYOUT_1_8CH,
} hdmi_audio_pkt_layout_t;


/**
 * @typedef hdmi_audio_fmt_t
 *   Audio packet format.
 */
typedef enum _hdmi_audio_fmt_e {
    /*! IEC 60958 */
    HDMI_AUDIO_FMT_60958,
    /*! IEC 60937, channel status outband */
    HDMI_AUDIO_FMT_61937,
    /*! Custom packet */
    HDMI_AUDIO_FMT_CUSTOM,
} hdmi_audio_fmt_t;

typedef struct cea861d_audio_infoframe_pkt_s {
    /*! Channel count
    *   see CEA-861-D table 17 for details
    */
    uint32_t cc:3;
    /*! Coding Type
    *     shall always be set to a value of 0
    */
    uint32_t reserved_1:1;
    uint32_t ct:4;
    /*! Sample Size
    *     shall always be set to a value of 0
    */
    uint32_t ss:2;
    /*! Sample Frequency
     *     See CEA-861-D table 18 for details. For L-PCM and IEC61937
     *     compressed audio streams, the SF bits shall always be set to a value
     *     of 0 ("Refer to Stream Header"). For One birt Audio and DST streams,
     *     the value indicated by the SF bits shall equal the ACR fs value. For
     *     Super Audio CD, the SF bits are typically set to 0,1,0 to indicate
     *     a Sample Frequency of 2.8224 MSamples/s (i.e. 64*44.1kHz)
     */
    uint32_t sf:3;
    uint32_t reserved_2:3;
    uint8_t reserved_3;
    /*! Channel/Speaker allocation
     *     See CEA-861-D Sections 6.6.2 for details. The CA field is not valid
     *     for IEC-61937 compressed audio streams.
     */
    uint8_t ca;
    /*! LFE Playback level information.
     */
    uint32_t lfepbl:2;
    uint32_t reserved_4:1;
    /*! Level Shift Value (for downmixing).
     *    See CEA-861-D section 6.6.2 and Table 21 for details.
     */
    uint32_t lsv:4;
    /*! Downmixing Inhibit. See CEA-861-D section 6.6.2 and table 22 for
     * details. The DM_INH field is to be set only for DVD-Audio applications
     * and corresponds to the value in the DM_INH field of the current audio
     * stream being played from the disk. The DM_INH field value shall be set to
     * zero in all cases other than DVD-Audio applications.
     */
    uint32_t dm_inh:1;
} cea861d_audio_infoframe_pkt_t ;

typedef union hdmi_audio_infoframe_pkt_u {
    cea861d_audio_infoframe_pkt_t value;
    uint8_t raw[5];
}hdmi_audio_infoframe_pkt_t;

/**
 * @typedef hdmi_cb_evt_t
 *   Asynchronous callback events, either stream changes events or
 * completion of non blocking calls. Mutually exclusive */
typedef enum _hdmi_cb_evt {
    HDMI_EVENT_AUDIO_SAMPLE_RATE = 0,   /*!< Audio sampling rate changed */
    HDMI_EVENT_AUDIO_STREAM_TYPE,   /*!< Audio stream type changed */
    HDMI_EVENT_AUDIO_LINK,          /*!< Audio physical connection changed */
    HDMI_EVENT_AUDIO_MCLK,          /*!< Audio mclk status changed */
    HDMI_EVENT_AUDIO_INFOFRAME,     /*!< Audio Infoframe packet received */
    HDMI_EVENT_AUDIO_CHANNEL_STATUS, /*!< Audio channel status packet received */
    HDMI_EVENT_AUDIO_LAYOUT_CHANGE, /*!< Audio channel layout changed */
    HDMI_EVENT_ERROR,         /*!< Hits some error(s) */
    HDMI_EVENT_MAX,         /*!< Number of events */
} hdmi_cb_evt_t;
#endif

/**
 * @typedef class_hdmi_cb_t
 *   Asynchronous callback previously registered through set_callback.
 *
 * @param[in]   event   Event type
 * @param[in]   param   Optional parameter. Semantic depends on event type
 * @param[in]   cookie  Callee cookie, set while calling set_callback
 * @return  The callback status error.
 */

typedef int (*hdmi_cb_t)(hdmi_cb_evt_t , void *, void *);

/* EP Advanced Audio Info registers */
#define EP_ADVANCED_AUDIO_INFO_SIZE 15

struct ep92a7e_state {
    i2c_bus *i2c_bus;

    hdmi_cb_t cb;
    void * cb_cookie;
    unsigned current_samplerate;
    hdmi_audio_pkt_t current_pkt_type;
    hdmi_audio_infoframe_pkt_t current_audio_info_pkt;
    hdmi_channel_status_t current_cs;
    hdmi_audio_pkt_layout_t current_pkt_layout;
    bool cs_pause_transition;
#ifdef LK
    int i2c_bus_id;
    struct device *dev;
    struct device *i2c_dev;
    spin_lock_t cb_lock;
    struct gpio_desc gpio_cs;
    struct gpio_desc gpio_reset;
#else
    struct rpmsg_rpc_dev *rpcdev;
    unsigned ept_id;
    unsigned int gpio_cs;
    unsigned int gpio_reset;
    struct task_struct *thread;
    uint8_t advanced_audio_info[EP_ADVANCED_AUDIO_INFO_SIZE];
#endif

    unsigned hw_state;
};

struct rpc_hdmi_s_format_s {
    hdmi_audio_direction_t direction;
    hdmi_audio_fmt_t fmt;
};

struct rpc_hdmi_s_if_s {
    hdmi_audio_direction_t direction;
    hdmi_audio_if_t in;
    hdmi_audio_if_t out;
};

struct rpc_hdmi_s_pkt_s {
    hdmi_audio_direction_t direction;
    hdmi_audio_pkt_t pkt;
};

/* Common function */
status_t ep_write_reg(i2c_bus *bus, uint8_t address, uint8_t reg, uint8_t val, unsigned wait_us);
status_t ep_read_reg(i2c_bus *bus, uint8_t address, uint8_t reg, uint8_t *val, unsigned wait_us);
status_t ep_read_regs(i2c_bus *bus, uint8_t address, uint8_t reg,
                      uint8_t *val, uint16_t cnt, unsigned wait_us);
void ep92a7e_sourcetype_config(struct ep92a7e_state *state);
void ep92a7e_reset(struct ep92a7e_state *state);
int HDMI_I2C_Config(struct ep92a7e_state *state);
status_t _hdmi_open(struct ep92a7e_state *state);
status_t _hdmi_close(struct ep92a7e_state *state);
status_t _hdmi_get_capabilities(uint32_t *capabilities);
status_t _hdmi_set_audio_format(struct ep92a7e_state *state, hdmi_audio_direction_t direction, hdmi_audio_fmt_t fmt);
status_t _hdmi_set_audio_packet(struct ep92a7e_state *state, hdmi_audio_direction_t direction, hdmi_audio_pkt_t pkt);
status_t _hdmi_set_audio_interface(struct ep92a7e_state *state,
                                    hdmi_audio_direction_t direction,
                                    hdmi_audio_if_t in, hdmi_audio_if_t out);
status_t _hdmi_get_channel_status(struct ep92a7e_state *state, hdmi_channel_status_t *channel_status);
status_t _hdmi_get_audio_infoframe_pkt(struct ep92a7e_state *state,
                                                hdmi_audio_infoframe_pkt_t *pkt);
status_t _hdmi_get_audio_custom_fmt_layout(struct ep92a7e_state *state, iec60958_custom_fmt_layout_t *layout);
status_t _hdmi_get_audio_pkt_type(struct ep92a7e_state *state, hdmi_audio_pkt_t *pkt);
status_t _hdmi_get_audio_pkt_layout(struct ep92a7e_state *state, hdmi_audio_pkt_layout_t *layout);

/* Specific functions */
#ifndef LK
int raw_i2c_read_block(struct i2c_client *client,
            uint8_t addr, uint8_t reg, uint8_t *val, uint16_t num);
int raw_i2c_write_block(struct i2c_client *client,
            uint8_t addr, uint8_t reg, uint8_t *val, uint16_t num);
int ep92a7e_i2c_read_reg(struct i2c_client *client,
            uint8_t addr, uint8_t reg, uint8_t *val);
int ep92a7e_i2c_write_reg(struct i2c_client *client,
            uint8_t addr, uint8_t reg, uint8_t val);
#else

#endif


#define EP_SLAVE_ADDRESS 0x61


#define EP_VAL_TO_FIELD(field, value) \
         (((value) << (field ## _SHIFT)) & (field ## _MASK))
#define EP_FIELD_TO_VAL(field, value) \
         (((value) & (field ## _MASK)) >> (field ## _SHIFT))
#define EP_SET_FIELD(var, field, value) \
         do { \
             var &= ~(field ## _MASK); \
             var |= IVSHM_VAL_TO_FIELD(field, value); \
         } while (0);


/*
 * Registers slave_address
 * EP9472_EP9XA7 SA_IF Register Map v1.05.xlsx
 */

/* Basic info */
#define EP_REG_INFO_VENDOR_ID_0                       0x00U
#define EP_REG_INFO_VENDOR_ID_1                       0x01U
#define EP_REG_INFO_DEVICE_ID_0                       0x02U
#define EP_REG_INFO_DEVICE_ID_1                       0x03U
#define EP_REG_INFO_VERSION_0                         0x04U
#define EP_REG_INFO_VERSION_1                         0x05U
#define EP_REG_INFO_VERSION_2                         0x06U
#define EP_REG_INFO_VERSION_3                         0x07U
#define EP_REG_INFO_GENERAL_INFO_0                    0x08U
#define EP_REG_INFO_GENERAL_INFO_1                    0x09U
/* ISP mode */
#define EP_REG_ENTER_ISP                              0x0FU
/* Basic control */
#define EP_REG_GENERAL_CONTROL_0                      0x10U
#define EP_REG_GENERAL_CONTROL_1                      0x11U
#define EP_REG_GENERAL_CONTROL_2                      0x12U
#define EP_REG_GENERAL_CONTROL_3                      0x13U
#define EP_REG_GENERAL_CONTROL_4                      0x14U
/* CEC */
#define EP_REG_CEC_EVENT                              0x15U
/* Advanced Audio Info */
#define EP_REG_SYSTEM_STATUS_0                        0x20U
#define EP_REG_SYSTEM_STATUS_1                        0x21U
#define EP_REG_AUDIO_STATUS                           0x22U
#define EP_REG_CHANNEL_STATUS_0                       0x23U
#define EP_REG_CHANNEL_STATUS_1                       0x24U
#define EP_REG_CHANNEL_STATUS_2                       0x25U
#define EP_REG_CHANNEL_STATUS_3                       0x26U
#define EP_REG_CHANNEL_STATUS_4                       0x27U
#define EP_REG_ADO_INFO_FRAME_0                       0x28U
#define EP_REG_ADO_INFO_FRAME_1                       0x29U
#define EP_REG_ADO_INFO_FRAME_2                       0x2AU
#define EP_REG_ADO_INFO_FRAME_3                       0x2BU
#define EP_REG_ADO_INFO_FRAME_4                       0x2CU
#define EP_REG_ADO_INFO_FRAME_5                       0x2DU

/* Other Packets */
#define EP_REG_HDMI_VS_0                              0x2EU
#define EP_REG_HDMI_VS_1                              0x2FU
#define EP_REG_ACP_PACKET                             0x30U
#define EP_REG_AVI_INFO_FRAME_0                       0x31U
#define EP_REG_AVI_INFO_FRAME_1                       0x32U
#define EP_REG_AVI_INFO_FRAME_2                       0x33U
#define EP_REG_AVI_INFO_FRAME_3                       0x34U
#define EP_REG_AVI_INFO_FRAME_4                       0x35U
#define EP_REG_GC_PACKET_0                            0x36U
#define EP_REG_GC_PACKET_1                            0x37U
#define EP_REG_GC_PACKET_2                            0x38U
/* Activation key */
#define EP_REG_ACTIVATION_KEY                         0x41U


 /* Basic info */
#define EP_REG_GENERAL_INFO_ARC_ON_SHIFT              0U
#define EP_REG_GENERAL_INFO_ARC_ON_MASK               0x01
#define EP_REG_GENERAL_INFO_EARC_ON_SHIFT             1U
#define EP_REG_GENERAL_INFO_EARC_ON_MASK              0x02
#define EP_REG_GENERAL_INFO_NEED_ACV_SHIFT            5U
#define EP_REG_GENERAL_INFO_NEED_ACV_MASK             0x20
#define EP_REG_GENERAL_INFO_TX_HDMI_SHIFT             6U
#define EP_REG_GENERAL_INFO_TX_HDMI_MASK              0x40
#define EP_REG_GENERAL_INFO_TX_HOT_PLUG_SHIFT         7U
#define EP_REG_GENERAL_INFO_TX_HOT_PLUG_MASK          0x80

#define EP_REG_GENERAL_INFO_ADO_CHF_SHIFT             0U
#define EP_REG_GENERAL_INFO_ADO_CHF_MASK              0x01
#define EP_REG_GENERAL_INFO_CEC_ECF_SHIFT             1U
#define EP_REG_GENERAL_INFO_CEC_ECF_MASK              0x02


/* Basic control */
#define EP_REG_GENERAL_CONTROL_ARC_EN_SHIFT           0U
#define EP_REG_GENERAL_CONTROL_ARC_EN_MASK            0x01
#define EP_REG_GENERAL_CONTROL_CEC_MUTE_SHIFT         1U
#define EP_REG_GENERAL_CONTROL_CEC_MUTE_MASK          0x02
#define EP_REG_GENERAL_CONTROL_CEC_DIS_SHIFT          2U
#define EP_REG_GENERAL_CONTROL_CEC_DIS_MASK           0x04
#define EP_REG_GENERAL_CONTROL_VIDEO_PATH_SHIFT       3U
#define EP_REG_GENERAL_CONTROL_VIDEO_PATH_MASK        0x08
#define EP_REG_GENERAL_CONTROL_A_RESET_SHIFT          4U
#define EP_REG_GENERAL_CONTROL_A_RESET_MASK           0x10
#define EP_REG_GENERAL_CONTROL_AUDIO_PATH_SHIFT       5U
#define EP_REG_GENERAL_CONTROL_AUDIO_PATH_MASK        0x20
#define EP_REG_GENERAL_CONTROL_EARC_EN_SHIFT          6U
#define EP_REG_GENERAL_CONTROL_EARC_EN_MASK           0x40
#define EP_REG_GENERAL_CONTROL_POWER_SHIFT            7U
#define EP_REG_GENERAL_CONTROL_POWER_MASK             0x80

#define EP_REG_GENERAL_CONTROL_PRIMARY_SEL_SHIFT      0U
#define EP_REG_GENERAL_CONTROL_PRIMARY_SEL_MASK       0x0F
#define EP_REG_GENERAL_CONTROL_ARP_FREQ_SHIFT         4U
#define EP_REG_GENERAL_CONTROL_ARP_FREQ_MASK          0xF0

#define EP_REG_GENERAL_CONTROL_SECONDARY_SEL_SHIFT    0U
#define EP_REG_GENERAL_CONTROL_SECONDARY_SEL_MASK     0x0F
#define EP_REG_GENERAL_CONTROL_EARC_DIS_SHIFT         5U
#define EP_REG_GENERAL_CONTROL_EARC_DIS_MASK          0x20
#define EP_REG_GENERAL_CONTROL_ARC_DIS_SHIFT          6U
#define EP_REG_GENERAL_CONTROL_ARC_DIS_MASK           0x40

#define EP_REG_GENERAL_CONTROL_CEC_VOLUME_SHIFT       0U
#define EP_REG_GENERAL_CONTROL_CEC_VOLUME_MASK        0xFF

#define EP_REG_GENERAL_CONTROL_LINKON0_SHIFT          0U
#define EP_REG_GENERAL_CONTROL_LINKON0_MASK           0x01
#define EP_REG_GENERAL_CONTROL_LINKON1_SHIFT          1U
#define EP_REG_GENERAL_CONTROL_LINKON1_MASK           0x02
#define EP_REG_GENERAL_CONTROL_LINKON2_SHIFT          2U
#define EP_REG_GENERAL_CONTROL_LINKON2_MASK           0x04
#define EP_REG_GENERAL_CONTROL_LINKON3_SHIFT          3U
#define EP_REG_GENERAL_CONTROL_LINKON3_MASK           0x08
#define EP_REG_GENERAL_CONTROL_PD_DETECT_SHIFT        4U
#define EP_REG_GENERAL_CONTROL_PD_DETECT_MASK         0x10
#define EP_REG_GENERAL_CONTROL_GPIO_SEL_SHIFT         5U
#define EP_REG_GENERAL_CONTROL_GPIO_SEL_MASK          0x60
#define EP_REG_GENERAL_CONTROL_SPIIS_EN_SHIFT         7U
#define EP_REG_GENERAL_CONTROL_SPIIS_EN_MASK          0x80


/* Advanced Audio Info */
#define EP_REG_SYSTEM_STATUS_LAYOUT_SHIFT             0U
#define EP_REG_SYSTEM_STATUS_LAYOUT_MASK              0x01
#define EP_REG_SYSTEM_STATUS_DST_DOUBLE_SHIFT         1U
#define EP_REG_SYSTEM_STATUS_DST_DOUBLE_MASK          0x02
#define EP_REG_SYSTEM_STATUS_EARC_MUTE_SHIFT          2U
#define EP_REG_SYSTEM_STATUS_EARC_MUTE_MASK           0x04
#define EP_REG_SYSTEM_STATUS_HDMI_SHIFT               4U
#define EP_REG_SYSTEM_STATUS_HDMI_MASK                0x10
#define EP_REG_SYSTEM_STATUS_AVMUTE_SHIFT             5U
#define EP_REG_SYSTEM_STATUS_AVMUTE_MASK              0x20
#define EP_REG_SYSTEM_STATUS_MCLK_OK_SHIFT            6U
#define EP_REG_SYSTEM_STATUS_MCLK_OK_MASK             0x40

#define EP_REG_SYSTEM_STATUS_DE_VALID_SHIFT           6U
#define EP_REG_SYSTEM_STATUS_DE_VALID_MASK            0x40
#define EP_REG_SYSTEM_STATUS_LINK_ON_SHIFT            7U
#define EP_REG_SYSTEM_STATUS_LINK_ON_MASK             0x80

#define EP_REG_AUDIO_STATUS_SAMP_FREQ_SHIFT           0U
#define EP_REG_AUDIO_STATUS_SAMP_FREQ_MASK            0x07
#define EP_REG_AUDIO_STATUS_STD_ADO_SHIFT             3U
#define EP_REG_AUDIO_STATUS_STD_ADO_MASK              0x8
#define EP_REG_AUDIO_STATUS_DSD_ADO_SHIFT             4U
#define EP_REG_AUDIO_STATUS_DSD_ADO_MASK              0x10
#define EP_REG_AUDIO_STATUS_HBR_ADO_SHIFT             5U
#define EP_REG_AUDIO_STATUS_HBR_ADO_MASK              0x20
#define EP_REG_AUDIO_STATUS_DST_ADO_SHIFT             6U
#define EP_REG_AUDIO_STATUS_DST_ADO_MASK              0x40


#define EP_REG_CHANNEL_STATUS_PRO_SHIFT               0U
#define EP_REG_CHANNEL_STATUS_PRO_MASK                0x01
#define EP_REG_CHANNEL_STATUS_PCM_SHIFT               1U
#define EP_REG_CHANNEL_STATUS_PCM_MASK                0x02
#define EP_REG_CHANNEL_STATUS_COPY_SHIFT              2U
#define EP_REG_CHANNEL_STATUS_COPY_MASK               0x04
#define EP_REG_CHANNEL_STATUS_PRE_SHIFT               3U
#define EP_REG_CHANNEL_STATUS_PRE_MASK                0x38
#define EP_REG_CHANNEL_STATUS_MODE_SHIFT              6U
#define EP_REG_CHANNEL_STATUS_MODE_MASK               0xC0

#define EP_REG_CHANNEL_STATUS_CAT_CODE_SHIFT          0U
#define EP_REG_CHANNEL_STATUS_CAT_CODE_MASK           0xFF

#define EP_REG_CHANNEL_STATUS_SRC_NUM_SHIFT           0U
#define EP_REG_CHANNEL_STATUS_SRC_NUM_MASK            0x0F
#define EP_REG_CHANNEL_STATUS_CH_NUM_SHIFT            4U
#define EP_REG_CHANNEL_STATUS_CH_NUM_MASK             0xF0

#define EP_REG_CHANNEL_STATUS_SAMP_FREQ_SHIFT         0U
#define EP_REG_CHANNEL_STATUS_SAMP_FREQ_MASK          0x0F
#define EP_REG_CHANNEL_STATUS_CLK_ADD_SHIFT           4U
#define EP_REG_CHANNEL_STATUS_CLK_ADD_MASK            0x30

#define EP_REG_CHANNEL_STATUS_MAX_LEN_SHIFT           0U
#define EP_REG_CHANNEL_STATUS_MAX_LEN_MASK            0x01
#define EP_REG_CHANNEL_STATUS_SAMP_LEN_SHIFT          1U
#define EP_REG_CHANNEL_STATUS_SAMP_LEN_MASK           0x0E
#define EP_REG_CHANNEL_STATUS_ORG_SAMP_FREQ_SHIFT     4U
#define EP_REG_CHANNEL_STATUS_ORG_SAMP_FREQ_MASK      0xF0


#define EP_REG_ADO_INFO_FRAME_CC_SHIFT                0U
#define EP_REG_ADO_INFO_FRAME_CC_MASK                 0x07
#define EP_REG_ADO_INFO_FRAME_CT_SHIFT                4U
#define EP_REG_ADO_INFO_FRAME_CT_MASK                 0xF0

#define EP_REG_ADO_INFO_FRAME_SS_SHIFT                0U
#define EP_REG_ADO_INFO_FRAME_SS_MASK                 0x03
#define EP_REG_ADO_INFO_FRAME_SF_SHIFT                2U
#define EP_REG_ADO_INFO_FRAME_SF_MASK                 0x1C

#define EP_REG_ADO_INFO_FRAME_CA_SHIFT                0U
#define EP_REG_ADO_INFO_FRAME_CA_MASK                 0xFF

#define EP_REG_ADO_INFO_FRAME_LSV_SHIFT               3U
#define EP_REG_ADO_INFO_FRAME_LSV_MASK                0x78
#define EP_REG_ADO_INFO_FRAME_DM_INH_SHIFT            7U
#define EP_REG_ADO_INFO_FRAME_DM_INH_MASK             0x80
