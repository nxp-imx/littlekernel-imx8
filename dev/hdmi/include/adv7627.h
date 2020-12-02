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
#define SLEEP_US(x) usleep_range(x,x)
#define DEBUG_ASSERT(cond) BUG_ON(!(cond))
#define ASSERT(cond) BUG_ON(!(cond))
#define ERR_BUSY -EBUSY

typedef int status_t;

enum handler_return {
    INT_NO_RESCHEDULE = 0,
    INT_RESCHEDULE,
};
#else
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
#define ADV_I2C_XFER_WAIT 3000

#define HDMI_ADV7627_FORCE_AUDIO_CHANNEL_STATUS 1
#define HDMI_ADV7627_FORCE_AUDIO_INFOFRAME 1

/* interrupts are mapped on both INT pin, registering one is enough
 *  - INT2 is not used with LK
 *  - INT1 is not used with LINUX
 */
#ifdef LK
#define HDMI_ADV7627_REGISTER_INT1
//#define HDMI_ADV7627_REGISTER_INT2
#else
//#define HDMI_ADV7627_REGISTER_INT1
#define HDMI_ADV7627_REGISTER_INT2
#endif

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
    ADV7627_SR44_1KHz = 0x00, /*!< Sample rate 44100 Hz */
    ADV7627_SR48KHz = 0x02, /*!< Sample rate 48000 Hz */
    ADV7627_SR32KHz = 0x03, /*!< Sample rate 32000 Hz */
    ADV7627_SR88_2KHz = 0x08, /*!< Sample rate 88200 Hz */
    ADV7627_SR96KHz = 0x0A, /*!< Sample rate 96000 Hz */
    ADV7627_SR176_4KHz = 0x0C, /*!< Sample rate 176400 Hz */
    ADV7627_SR192KHz = 0x0E, /*!< Sample rate 192000 Hz */
    ADV7627_SR768KHz = 0x09, /*!< Sample rate 768000 Hz */
};

/*! @brief Audio sample rate */
enum adv7627_sample_rate {
    sr_44_1KHz = 44100U,
    sr_48KHz = 48000U,
    sr_32KHz = 32000U,
    sr_88_2KHz = 88200U,
    sr_96KHz = 96000U,
    sr_176_4KHz = 176400U,
    sr_192KHz = 192000U
};

/*! @brief ADV7627 Interrupts */
enum adv7627_irqs {
    adv7627_new_samp_rt_rx2 = 0,
    adv7627_audio_mode_chng_rx2,
    adv7627_audio_info_rx2,
    adv7627_audio_cs_valid_rx2,
    adv7627_audio_ch_mode_rx2,
    adv7627_clock_detect_rx2,
#ifdef ADV7627_WITH_ACR_IRQ
    adv7627_audio_ch_n_rx2,
#endif
    adv7627_int_max
};

typedef struct {
    u8 addr;
    u8 status;
    u8 val;
} adv7627_cached_status_t;

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

struct adv7627_state {
    i2c_bus *i2c_bus;
    unsigned int irq_int1;
    unsigned int irq_int2;

    adv7627_cached_status_t cached_status[adv7627_int_max];
    hdmi_cb_t cb;
    void * cb_cookie;
    unsigned current_samplerate;
    bool audio_info_pkt_is_valid;
    bool audio_channel_status_is_valid;
    hdmi_audio_infoframe_pkt_t audio_info_pkt;
    hdmi_channel_status_t audio_cs;
    hdmi_audio_pkt_layout_t pkt_layout;
#ifdef LK
    int i2c_bus_id;
    struct device *dev;
    struct device *i2c_dev;
    spin_lock_t cb_lock;
    struct gpio_desc gpio_cs;
    struct gpio_desc gpio_reset;
    struct gpio_desc gpio_int1;
    struct gpio_desc gpio_int2;
#else
    struct rpmsg_rpc_dev *rpcdev;
    unsigned int gpio_cs;
    unsigned int gpio_reset;
    unsigned int gpio_int1;
    unsigned int gpio_int2;
    unsigned ept_id;
#endif
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
status_t adv_write_reg(i2c_bus *bus, uint8_t address, uint8_t reg, uint8_t val, unsigned wait_us);
status_t adv_read_reg(i2c_bus *bus, uint8_t address, uint8_t reg, uint8_t *val, unsigned wait_us);
int HDMI_I2C_Config(struct adv7627_state *state);
int adv7627_config_irq_pins(struct adv7627_state *state);
uint32_t adv7627_read_and_update_mclk(struct adv7627_state *state);
status_t _hdmi_open(struct adv7627_state *state);
status_t _hdmi_close(struct adv7627_state *state);
status_t _hdmi_get_capabilities(uint32_t *capabilities);
status_t _hdmi_set_audio_format(struct adv7627_state *state, hdmi_audio_direction_t direction, hdmi_audio_fmt_t fmt);
status_t _hdmi_set_audio_packet(struct adv7627_state *state, hdmi_audio_direction_t direction, hdmi_audio_pkt_t pkt);
status_t _hdmi_set_audio_interface(struct adv7627_state *state,
                                    hdmi_audio_direction_t direction,
                                    hdmi_audio_if_t in, hdmi_audio_if_t out);
status_t _hdmi_get_channel_status(struct adv7627_state *state, hdmi_channel_status_t *channel_status);
status_t _hdmi_get_audio_infoframe_pkt(struct adv7627_state *state,
                                                hdmi_audio_infoframe_pkt_t *pkt);
status_t _hdmi_get_audio_custom_fmt_layout(struct adv7627_state *state, iec60958_custom_fmt_layout_t *layout);
status_t _hdmi_get_audio_pkt_type(struct adv7627_state *state, hdmi_audio_pkt_t *pkt);
status_t _hdmi_get_audio_pkt_layout(struct adv7627_state *state, hdmi_audio_pkt_layout_t *layout);

/* Specific functions */
#ifndef LK
irq_handler_t adv7627_irq_handler(int irq, void *args);
int raw_i2c_read_block(struct i2c_client *client,
            uint8_t addr, uint8_t reg, uint8_t *val, uint16_t num);
int raw_i2c_write_block(struct i2c_client *client,
            uint8_t addr, uint8_t reg, uint8_t *val, uint16_t num);
int adv7627_i2c_read_reg(struct i2c_client *client,
            uint8_t addr, uint8_t reg, uint8_t *val);
int adv7627_i2c_write_reg(struct i2c_client *client,
            uint8_t addr, uint8_t reg, uint8_t val);
int rpc_adv7627_config_irq_pins(struct adv7627_state *state);
void hexdump8_ex(void *ptr, size_t len, uint64_t disp_addr);
#else
enum handler_return adv7627_irq_handler(void *args);
#endif
