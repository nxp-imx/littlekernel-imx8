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

#include "adv7627.h"
#include "debug.h"

char edid[256] =
{
    /* Created by:Advantiv® EEditGold 1.01.0268 by Analog Devices   4/21/2020 3:40:38 PM */
    0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,
    0x3B,0x10,0x00,0x00,0x00,0x00,0x00,0x00,
    0xFF,0x1E,0x01,0x04,0xB2,0x59,0x32,0x78,
    0x1F,0xEE,0x91,0xA3,0x54,0x4C,0x99,0x26,
    0x0F,0x50,0x54,0xBD,0xEF,0x80,0x71,0x4F,
    0x81,0x00,0x81,0x40,0x81,0x80,0x95,0x00,
    0x95,0x0F,0xB3,0x00,0xA9,0x40,0x02,0x3A,
    0x80,0x18,0x71,0x38,0x2D,0x40,0x58,0x2C,
    0x45,0x00,0x7A,0xF4,0x31,0x00,0x00,0x1E,
    0x66,0x21,0x50,0xB0,0x51,0x00,0x1B,0x30,
    0x40,0x70,0x36,0x00,0x7A,0xF4,0x31,0x00,
    0x00,0x1E,0x00,0x00,0x00,0xFD,0x00,0x18,
    0x4B,0x1A,0x51,0x17,0x00,0x0A,0x20,0x20,
    0x20,0x20,0x20,0x20,0x00,0x00,0x00,0xFC,
    0x00,0x4E,0x58,0x50,0x2D,0x49,0x33,0x44,
    0x0A,0x20,0x20,0x20,0x20,0x20,0x01,0x21,
    0x02,0x03,0x4C,0xF3,0x55,0x85,0x04,0x03,
    0x02,0x0E,0x0F,0x07,0x23,0x24,0x10,0x94,
    0x13,0x12,0x11,0x1D,0x1E,0x16,0x25,0x26,
    0x01,0x1F,0x3B,0x09,0x7F,0x07,0x0F,0x7F,
    0x07,0x17,0x07,0x50,0x3F,0x1F,0xC0,0x57,
    0x3E,0x03,0x67,0x7E,0x01,0x5F,0x7E,0x03,
    0x37,0x07,0x28,0x7B,0x07,0x59,0x83,0xFF,
    0x07,0x00,0x67,0x03,0x0C,0x00,0x20,0x00,
    0xB8,0x2D,0xE2,0x00,0x0F,0xE6,0x0E,0x90,
    0x20,0x22,0x04,0x05,0x02,0x3A,0x80,0xD0,
    0x72,0x38,0x2D,0x40,0x10,0x2C,0x45,0x80,
    0xA0,0x5A,0x00,0x00,0x00,0x1E,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCE
};

/*************************
 ******* I2C ACCESS ******
 *************************/

#ifndef LK
#define _I2C_WRITE_REG(bus, addr, reg, val)   adv7627_i2c_write_reg(bus, addr, reg, val)
#define _I2C_READ_REG(bus, addr, reg, val)    adv7627_i2c_read_reg(bus, addr, reg, val) 
#else
#define _I2C_WRITE_REG(bus, addr, reg, val)   i2c_write_reg(*bus, address, reg, val)
#define _I2C_READ_REG(bus, addr, reg, val)    i2c_read_reg(*bus, address, reg, val)
#endif

status_t adv_write_reg(i2c_bus *bus, uint8_t address, uint8_t reg, uint8_t val, unsigned wait_us)
{
    status_t ret;

    ret = _I2C_WRITE_REG(bus, address, reg, val);

    if (ret)
        return ret;

    SLEEP_US(wait_us);

    return 0;
}

status_t adv_read_reg(i2c_bus *bus, uint8_t address, uint8_t reg, uint8_t *val, unsigned wait_us)
{
    status_t ret;

    ret = _I2C_READ_REG(bus, address, reg, val);

    if (ret)
        return ret;

    SLEEP_US(wait_us);

    return 0;

}

static inline status_t adv_io_read_reg(struct adv7627_state *state, uint8_t reg, uint8_t *val)
{
    return adv_read_reg(state->i2c_bus, 0x58, reg, val, 0);
}

static inline status_t adv_io_write_reg(struct adv7627_state *state, uint8_t reg, uint8_t val)
{
    return adv_write_reg(state->i2c_bus, 0x58, reg, val, 0);
}

static inline status_t adv_rx_main_read_reg(struct adv7627_state *state, uint8_t reg, uint8_t *val)
{
    return adv_read_reg(state->i2c_bus, 0x2C, reg, val, 0);
}

static inline status_t adv_rx_main_write_reg(struct adv7627_state *state, uint8_t reg, uint8_t val)
{
    return adv_write_reg(state->i2c_bus, 0x2C, reg, val, 0);
}

/*************************
 *** ADV7627 specifics ***
 *************************/

static void adv7627_rx2_set_mclk(struct adv7627_state *state, uint32_t mclk)
{
    uint16_t slave_address;
    uint8_t reg_val;

    reg_val = mclk;
    slave_address = 0x48;
    adv_write_reg(state->i2c_bus, slave_address, 0xB5, reg_val, ADV_I2C_XFER_WAIT);
}

static void adv_7627_update_mclk(struct adv7627_state *adv_state,
                          uint32_t sample_rate)
{
    if (adv_state->current_samplerate != sample_rate) {
        adv_state->current_samplerate = sample_rate;
        switch (sample_rate) {
            case sr_44_1KHz:
                printlk(LK_DEBUG, "Sample Rate 44.1kHz\r\n");
                adv7627_rx2_set_mclk(adv_state, mclk_512fs);
                break;
            case sr_48KHz:
                printlk(LK_DEBUG, "Sample Rate 48kHz\r\n");
                /* Set ADV7627 MCLKOUT to 512fs. */
                adv7627_rx2_set_mclk(adv_state, mclk_512fs);
                break;
            case sr_32KHz:
                printlk(LK_DEBUG, "Sample Rate 32kHz\r\n");
                /* Set ADV7627 MCLKOUT to 512fs. */
                adv7627_rx2_set_mclk(adv_state, mclk_512fs);
                break;
            case sr_88_2KHz:
                printlk(LK_DEBUG, "Sample Rate 88.2kHz\r\n");
                adv7627_rx2_set_mclk(adv_state, mclk_256fs);
                break;
            case sr_96KHz:
                printlk(LK_DEBUG, "Sample Rate 96kHz\r\n");
                /* Set ADV7627 MCLKOUT to 256fs. */
                adv7627_rx2_set_mclk(adv_state, mclk_256fs);
                break;
            case sr_176_4KHz:
                printlk(LK_DEBUG, "Sample Rate 176_4kHz\r\n");
                adv7627_rx2_set_mclk(adv_state, mclk_128fs);
                break;
            case sr_192KHz:
                printlk(LK_DEBUG, "Sample Rate 192kHz\r\n");
                /* Set ADV7627 MCLKOUT to 128fs. */
                adv7627_rx2_set_mclk(adv_state, mclk_128fs);
                break;
            default:
                printlk(LK_DEBUG, "Undefined Sample Rate\r\n");
                DEBUG_ASSERT(sample_rate != 0);
                break;
        }
    }
}

static hdmi_audio_pkt_layout_t hdmi_adv7627_get_pkt_layout(struct adv7627_state *state)
{
    uint8_t reg_val;
    hdmi_audio_pkt_layout_t pkt_layout;

    adv_io_read_reg(state, 0x7F, &reg_val);

    /*
     * DOC error: Despite AUDIO_CH_MD_RX2_RAW bit description,
     * this bit clearly describes the infoframe layout
     * ( 0 -> layout 0, 1 -> layout 1)
     * */
    if (reg_val & 0x10)
        pkt_layout = HDMI_AUDIO_PKT_LAYOUT_1_8CH;
    else
        pkt_layout = HDMI_AUDIO_PKT_LAYOUT_0_2CH;


    printlk(LK_VERBOSE, "Packet channel layout: [%x]\n", pkt_layout);
    return pkt_layout;
}

static hdmi_audio_pkt_t hdmi_adv7627_get_pkt_type(struct adv7627_state *state)
{
    uint8_t reg_val;
    uint16_t slave_address = 0x2C;
    hdmi_audio_pkt_t pkt;

    adv_read_reg(state->i2c_bus, slave_address, 0x18, &reg_val, 0);
    printlk(LK_VERBOSE, "Packet type detection, raw value: [%x]\n", reg_val);
    
    switch (reg_val & 0xB) {
    case 0x0:
        pkt = HDMI_AUDIO_PKT_NONE;
        printlk(LK_DEBUG, "No audio pkt detected\n");
        break;
    case 0x1:
        pkt = HDMI_AUDIO_PKT_STD;
        printlk(LK_DEBUG, "STD audio pkt detected\n");
        break;
    case 0x2:
        pkt = HDMI_AUDIO_PKT_DSD;
        printlk(LK_DEBUG, "DSD audio pkt detected\n");
        break;
    case 0x8:
        pkt = HDMI_AUDIO_PKT_HBR;
        printlk(LK_DEBUG, "HBR audio pkt detected\n");
        break;
    default:
        panic("Invalid audio packet type detected [%x]\n", reg_val);
    }

    return pkt;
}

static status_t hdmi_adv7627_get_audio_infoframe_pkt(
                        struct adv7627_state *state,
                        hdmi_audio_infoframe_pkt_t *pkt)
{
    uint16_t slave_address = (0xF6 >> 1);
    /* Rx2 InfoFrame map */
    uint8_t reg_val;
    adv_io_read_reg(state, 0x98, &reg_val);

    if (reg_val & (1 << 1)) {
        adv_io_write_reg(state, 0x9A, (1 << 1));
#ifdef LK
        return ERR_BUSY;
#else
        return -EBUSY;
#endif
    }

#ifdef LK
    return i2c_read_reg_bytes(* (int *)state->i2c_bus, slave_address,
                            0x1D, pkt->raw, sizeof(hdmi_audio_infoframe_pkt_t));
#else
    return raw_i2c_read_block((struct i2c_client *)state->i2c_bus, slave_address, 0x1D, pkt->raw, sizeof(hdmi_audio_infoframe_pkt_t));
#endif
}

/* IMXAF-1539: WA: Force sample rate in channel status */
static void adv7627_force_sample_rate_cs_to_768k(hdmi_channel_status_t *cs)
{
    cs->channel_status.cs.sampling_frequency = ADV7627_SR768KHz & 0xF;
    cs->channel_status.cs.sampling_frequency_extended = (ADV7627_SR768KHz >> 4) & 0xF ;
    cs->mask.sampling_frequency = 1;
}

static status_t hdmi_adv7627_get_channel_status(struct adv7627_state *state,
                                                hdmi_channel_status_t *cs)
{
    static hdmi_channel_status_mask_t adv7627_cs_validity_mask = {
        .professional = 1,
        .lpcm = 1,
        .copyright = 1,
        .pre_emphasis = 1,
        .mode = 1,
        .category = 1,
        .source = 1,
        .channel = 1,
        .sampling_frequency = 1,
        .clock_accuracy = 1,
        .sampling_frequency_extended = 1,
        .max_word_length = 1,
        .word_length = 1,
        .original_sampling_frequency = 1,
        .cgms_a = 0,
        .cgms_a_validity = 0,
        .audio_sampling_coefficient = 0,
        .hidden = 0,
        .channel_A_number = 0,
        .channel_B_number = 0,
    };

    uint8_t reg_val;

    /* Read cs_data_valid_rx2_raw */
    adv_io_read_reg(state, 0x7F, &reg_val);
    if (reg_val & (1 << 7)) {
        /* Rx2 Main map */
        uint16_t slave_address = (0x58 >> 1);

        cs->mask = adv7627_cs_validity_mask;
#ifdef LK
        return i2c_read_reg_bytes(* (int *)state->i2c_bus, slave_address,
                0x36, cs->channel_status.raw, 5);
#else
        return raw_i2c_read_block((struct i2c_client *)state->i2c_bus, slave_address, 0x36, cs->channel_status.raw, 5);
#endif
    }

    if (hdmi_adv7627_get_pkt_type(state) == HDMI_AUDIO_PKT_HBR) {
        /* IMXAF-1539: Workaround when on startup the source is not playing but previously configured for HBR,
         * on next HBR playback(s), channel status IRQ may not occur, so sample rate may be unknown.
         * pkt_type is known so we hard code the corresponding channel status fields to expected frequency + valid mask
         */
        printlk(LK_DEBUG, "Invalid channel status but HBR detected on startup: Init sample rate to 768kHz\n");
        adv7627_force_sample_rate_cs_to_768k(cs);
        return 0;
    }

    cs->mask = (hdmi_channel_status_mask_t){0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    return ERR_BUSY;
}

static uint32_t adv7627_rx2_get_samplerate(struct adv7627_state *state)
{
    uint16_t slave_address;
    uint8_t reg_val;
    uint32_t sample_rate = 0;

    slave_address = 0x2C;
    adv_read_reg(state->i2c_bus, slave_address, 0x39, &reg_val, 0);
    printlk(LK_DEBUG, "reg_val %#x\n", reg_val);

    switch (reg_val & 0x0F) {
        case ADV7627_SR44_1KHz:
            sample_rate = sr_44_1KHz;
            break;
        case ADV7627_SR48KHz:
            sample_rate = sr_48KHz;
            break;
        case ADV7627_SR32KHz:
            sample_rate = sr_32KHz;
            break;
        case ADV7627_SR88_2KHz:
            sample_rate = sr_88_2KHz;
            break;
        case ADV7627_SR96KHz:
            sample_rate = sr_96KHz;
            break;
        case ADV7627_SR176_4KHz:
            sample_rate = sr_176_4KHz;
            break;
        case ADV7627_SR192KHz:
        case ADV7627_SR768KHz:
            sample_rate = sr_192KHz;
            break;
        default:
            break;
    }

    return sample_rate;
}

uint32_t adv7627_read_and_update_mclk(struct adv7627_state *state)
{
    uint32_t sample_rate = adv7627_rx2_get_samplerate(state);

    /* Note:
     * Sending pause/play from Linux host using mpv at very short time intervals,
     * it is observed that New sample rate reading gives undefined values.
     * As per ADV7627 chip manual: "The first 40 of the channel status bits sent
     * by the upstream Tx (Linux host using mpv) have been collected correctly.
     * This bit does not indicate if the content of the channel status bit is
     * corrupted as this is indeterminable."
     * As it is "indeterminable" by ADV7627 that reading new sample rate value
     * gives undefined or any known values, we should skip this DEBUG_ASSERT from here.
     * We will only set proper MCLK when ADV7627 chip gives correct sample rate.
     */

    if (sample_rate != 0)
        adv_7627_update_mclk(state, sample_rate);

    return sample_rate;
}

/* Callbacks */
#ifdef LK
#define CALLBACK(state, event_id, data_p, data_size)    state->cb(event_id, data_p.data, state->cb_cookie)
#else
#define CALLBACK(state, event_id, data_p, data_size)    rpmsg_rpc_call(state->rpcdev, RPC_HDMI_HANDLE_EVENT_ID, data_p, data_size, NULL, NULL)
#endif

static int adv7627_new_samp_rt_rx2_cb(struct adv7627_state *state)
{
    struct param {
        hdmi_cb_evt_t id;
        uint32_t data;
    } param;

    /* Get ADV7627 RX2 Sample rate */
    printlk(LK_DEBUG,"Audio Sample Rate IRQ\n");
    param.data = adv7627_read_and_update_mclk(state);

    param.id = HDMI_EVENT_AUDIO_SAMPLE_RATE;
    CALLBACK(state, HDMI_EVENT_AUDIO_SAMPLE_RATE, &param, sizeof(struct param));

    return 0;
}

static int adv7627_audio_mode_chng_cb(struct adv7627_state *state)
{
    struct param {
        hdmi_cb_evt_t id;
        hdmi_audio_pkt_t data;
    } param;
    hdmi_channel_status_t cs;

    printlk(LK_DEBUG,"Audio Mode change IRQ\n");

    /* Get ADV7627 RX2 HBR AUDIO PCKT DETECTION */
    param.data = hdmi_adv7627_get_pkt_type(state);
    param.id = HDMI_EVENT_AUDIO_STREAM_TYPE;

    CALLBACK(state, HDMI_EVENT_AUDIO_STREAM_TYPE, &param, sizeof(struct param));

    if (param.data == HDMI_AUDIO_PKT_HBR) {
        if (state->current_samplerate  < sr_176_4KHz) {
            struct param_cs {
                hdmi_cb_evt_t id;
                hdmi_channel_status_t data;
            } param_cs;
            /*
             * We likely have missing a frequency transition
             * For now, assume the source is always 48Khz..
             * so we need to setup the stream to 192Khz
             * TODO: Proove we were missing an interrupt
             * */
            printlk(LK_INFO,"Force sample Rate 192kHz transition from %d\n",
                                    state->current_samplerate);
            adv_7627_update_mclk(state, sr_192KHz);

            /* IMXAF-1539: Force sample rate in CS as well if invalid as the CS IRQ may never occur */
            cs = state->audio_cs;
            if (cs.mask.sampling_frequency == 0) {
                printlk(LK_DEBUG, "Invalid channel status but HBR detected: force sample rate to 768kHz\n");
                adv7627_force_sample_rate_cs_to_768k(&cs);
                param_cs.id = HDMI_EVENT_AUDIO_CHANNEL_STATUS;
                param_cs.data = cs;
                CALLBACK(state, HDMI_EVENT_AUDIO_CHANNEL_STATUS, &param_cs, sizeof(struct param_cs));
            }
        }
    }

    return 0;
}

static int adv7627_audio_infoframe_cb(struct adv7627_state *state)
{
    struct param {
        hdmi_cb_evt_t id;
        hdmi_audio_infoframe_pkt_t data;
    } param;

    printlk(LK_INFO,"Audio Infoframe IRQ\n");

    if (hdmi_adv7627_get_audio_infoframe_pkt(state, &state->audio_info_pkt)) {
        state->audio_info_pkt_is_valid = false;
        printlk(LK_DEBUG, "Invalid audio info pkt\n");
        return ERR_BUSY;
    }
    printlk(LK_INFO,"Audio infoframe valid\n");

    state->audio_info_pkt_is_valid = true;

    param.id = HDMI_EVENT_AUDIO_INFOFRAME;
    param.data = state->audio_info_pkt;
    CALLBACK(state, HDMI_EVENT_AUDIO_INFOFRAME, &param, sizeof(struct param));

    return 0;
}

static int adv7627_cs_cb(struct adv7627_state *state)
{
    struct param {
        hdmi_cb_evt_t id;
        hdmi_channel_status_t data;
    } param;

    printlk(LK_INFO, "Audio Channel status IRQ\n");
    if (hdmi_adv7627_get_channel_status(state, &state->audio_cs)) {
        state->audio_channel_status_is_valid = false;
        printlk(LK_DEBUG, "Invalid audio channel status\n");
        return ERR_BUSY;
    }

    printlk(LK_INFO, "Audio Channel status is valid\n");

    state->audio_channel_status_is_valid = true;

    adv7627_read_and_update_mclk(state);

    param.id = HDMI_EVENT_AUDIO_CHANNEL_STATUS;
    param.data = state->audio_cs;
    CALLBACK(state, HDMI_EVENT_AUDIO_CHANNEL_STATUS, &param, sizeof(struct param));

    return 0;
}

static int adv7627_layout_cb(struct adv7627_state *state)
{
    struct param {
        hdmi_cb_evt_t id;
        hdmi_audio_pkt_layout_t data;
    } param;

    printlk(LK_INFO, "Audio Channel layout change IRQ\n");

    state->pkt_layout = hdmi_adv7627_get_pkt_layout(state);

    printlk(LK_INFO, "Audio Channel layout [%x]\n", state->pkt_layout);

    param.id = HDMI_EVENT_AUDIO_LAYOUT_CHANGE;
    param.data = state->pkt_layout;
    CALLBACK(state, HDMI_EVENT_AUDIO_LAYOUT_CHANGE, &param, sizeof(struct param));
    
    return 0;
}

static int adv7627_clock_detect_cb(struct adv7627_state *state)
{
    struct param {
        hdmi_cb_evt_t id;
        bool data;
    } param;
    uint8_t regval;
    bool is_connected;

    adv_io_read_reg(state, (uint8_t)0x4d, &regval);
    printlk(LK_DEBUG, "ADV7627 register 0x4D value: %#x\n", regval);

    is_connected = (regval & 0x1) ? true : false;

    printlk(LK_INFO, "HDMI RxC clock detection IRQ: clock %s\n", \
        (is_connected == true) ? "detected" : "not detected");

    param.id = HDMI_EVENT_AUDIO_LINK;
    param.data = is_connected;
    CALLBACK(state, HDMI_EVENT_AUDIO_LINK, &param, sizeof(struct param));

    return 0;
}

#ifdef ADV7627_WITH_ACR_IRQ
static int adv7627_acr_cb(struct adv7627_state *state)
{
    printlk(LK_INFO, "Audio Clock Regeneration Packet Detection Change IRQ\n");
    /* TODO: Update upper layer ? */
    return 0;
}
#endif

/* IRQ MAPPING */
struct adv7627_irq_mapping_s {
    const char *name;
    int (*cb)(struct adv7627_state *);
    int (*install)(struct adv7627_state *);
    int (*handler)(struct adv7627_state *, uint32_t irq_no);
    u8 addr;
    u8 mask;
    u8 status;
    u8 clear;
    u8 bit;
} adv7627_irq_mapping[] = {
    {
        .name = "RX2: New sampling rate",
        .handler = NULL,
        .cb = adv7627_new_samp_rt_rx2_cb,
        .addr= 0x58,
        .mask = 0x96,
        .status = 0x94,
        .clear = 0x95,
        .bit = 3,
    },
    {
        .name = "RX2: Audio mode change",
        .handler = NULL,
        .cb = adv7627_audio_mode_chng_cb,
        .addr = 0x58,
        .mask = 0x96,
        .status = 0x94,
        .clear = 0x95,
        .bit = 5,
    },
    {
        .name = "RX2: Audio Infoframe",
        .handler = NULL,
        .cb = adv7627_audio_infoframe_cb,
        .addr = 0x58,
        .mask = 0x8c,
        .status = 0x8a,
        .clear = 0x8b,
        .bit = 1,
    },
    {
        .name = "RX2: Audio channel status",
        .handler = NULL,
        .cb = adv7627_cs_cb,
        .addr = 0x58,
        .mask = 0x82,
        .status = 0x80,
        .clear = 0x81,
        .bit = 7,
    },
    {
        .name = "RX2: Audio channel layout change",
        .handler = NULL,
        .cb = adv7627_layout_cb,
        .addr = 0x58,
        .mask = 0x82,
        .status = 0x80,
        .clear = 0x81,
        .bit = 4,
    },
    {
        .name = "RX2: TMDS clock detection (cable connect)",
        .handler = NULL,
        .cb = adv7627_clock_detect_cb,
        .addr = 0x58,
        .mask = 0x50,
        .status = 0x4e,
        .clear = 0x4f,
        .bit = 0,
    },
#ifdef ADV7627_WITH_ACR_IRQ
    {
        .name = "RX2: Audio Change N",
        .handler = NULL,
        .cb = adv7627_acr_cb,
        .addr = 0x58,
        .mask = 0x91,
        .status = 0x8F,
        .clear = 0x90,
        .bit = 3,
    },
#endif
};

static int adv7627_enable_irq(struct adv7627_state *state, uint32_t irq_no,
            uint8_t enable)
{
    struct adv7627_irq_mapping_s *irq;
    uint8_t reg_val;

    if (irq_no >= adv7627_int_max)
        return -1;

    irq = &adv7627_irq_mapping[irq_no];
    adv_read_reg(state->i2c_bus, irq->addr, irq->mask, &reg_val, 0);
    if (enable)
        reg_val |= (1 << irq->bit);
    else
        reg_val &= ~(1 << irq->bit);
    adv_write_reg(state->i2c_bus, irq->addr, irq->mask, reg_val, ADV_I2C_XFER_WAIT);

    return 0;
}

static bool __attribute__((unused)) adv7627_is_irq_pending(struct adv7627_state *state, uint32_t irq_no)
{
    struct adv7627_irq_mapping_s *irq;
    uint8_t reg_val;

    if (irq_no >= adv7627_int_max)
        return false;

    irq = &adv7627_irq_mapping[irq_no];
    adv_read_reg(state->i2c_bus, irq->addr, irq->status, &reg_val, 0);

    return !!(reg_val &= (1 << irq->bit));
}

static int adv7627_get_cached_irq_status(struct adv7627_state *state, u8 addr, u8 status, u8 *val)
{
    int i = 0;
    adv7627_cached_status_t *cached_status = state->cached_status;

    for (i = 0; i < adv7627_int_max; i++) {
        if (addr == cached_status[i].addr && status == cached_status[i].status) {
            *val = cached_status[i].val;
            return 0;
        }
    }
    return -1;
}

static int adv7627_cache_irq_status(struct adv7627_state *state, u8 addr, u8 status, u8 val)
{
    int i = 0;
    adv7627_cached_status_t *cached_status = state->cached_status;

    while (i < adv7627_int_max) {
        if (cached_status[i].addr == 0) {
            /* Create entry */
            cached_status[i].addr = addr;
            cached_status[i].status = status;
            cached_status[i].val = val;
            return 0;
        }
        i++;
    }
    return -1;
}

static void adv7627_clear_cached_irq_status(struct adv7627_state *state)
{
    adv7627_cached_status_t *cached_status = state->cached_status;

    printlk(LK_VERBOSE, "Clear cached IRQ status\n");
    memset(cached_status, 0, adv7627_int_max * sizeof(adv7627_cached_status_t));
}

static bool adv7627_is_irq_pending_cache(struct adv7627_state *state, uint32_t irq_no)
{
    uint8_t reg_val;
    int ret;

    struct adv7627_irq_mapping_s *irq = &adv7627_irq_mapping[irq_no];

    if (irq_no >= adv7627_int_max)
        return false;


    ret = adv7627_get_cached_irq_status(state, irq->addr, irq->status, &reg_val);
    if (ret != -1) {
        printlk(LK_VERBOSE, "Read cached status 0x%0x (0x%x)\n", irq->status, reg_val);
    } else {
        adv_read_reg(state->i2c_bus, irq->addr, irq->status, &reg_val, 0);
        printlk(LK_VERBOSE, "Caching status 0x%x (0x%x)\n", irq->status, reg_val);
        adv7627_cache_irq_status(state, irq->addr, irq->status, reg_val);
    }

    return !!(reg_val &= (1 << irq->bit));
}

static void adv7627_clear_irq(struct adv7627_state *state, uint32_t irq_no)
{
    struct adv7627_irq_mapping_s *irq;
    uint8_t reg_val;

    if (irq_no >= adv7627_int_max)
        return;

    irq = &adv7627_irq_mapping[irq_no];
    reg_val = (1 << irq->bit);
    adv_write_reg(state->i2c_bus, irq->addr, irq->clear, reg_val, ADV_I2C_XFER_WAIT);

    return;
}

#ifdef LK
enum handler_return adv7627_irq_handler(void *args)
#else
irq_handler_t adv7627_irq_handler(int irq, void *args)
#endif
{
    uint32_t irq_no;
    int irq_remaining = false;
    struct adv7627_state *state = args;

    DEBUG_ASSERT(state);

    printlk(LK_VERBOSE, "-------->\n");

    do {
        irq_remaining = false;
        adv7627_clear_cached_irq_status(state);
        for (irq_no = 0; irq_no < ARRAY_SIZE(adv7627_irq_mapping); irq_no++) {
            struct adv7627_irq_mapping_s *irq = &adv7627_irq_mapping[irq_no];
            if (adv7627_is_irq_pending_cache(state, irq_no)) {
                adv7627_clear_irq(state, irq_no);
                irq_remaining = true;
                if (irq->handler != NULL) {
                    irq->handler(state, irq_no);
                }
                break;
            }
        }
    } while (irq_remaining);

    printlk(LK_VERBOSE, "<--------\n");

    return INT_NO_RESCHEDULE;
}

static int hdmi_adv7627_irq_handler(struct adv7627_state *state,
                uint32_t irq_no)
{
    struct adv7627_irq_mapping_s *irq = &adv7627_irq_mapping[irq_no];
    int ret = false;

    printlk(LK_VERBOSE, "Interrupt %s[%d]\n", irq->name, irq_no);
    if (irq->cb)
        ret = irq->cb(state);

    return ret;
}

typedef int (*ADV7627IrqHandler)(struct adv7627_state *, uint32_t irq_no);
static int adv7627_register_irq(struct adv7627_state *state,
                                    ADV7627IrqHandler handler, uint32_t irq_no)
{
    struct adv7627_irq_mapping_s *irq;

    if (handler == NULL || irq_no >= adv7627_int_max)
        return -1;

    irq = &adv7627_irq_mapping[irq_no];
    if (irq->handler != NULL) {
        printlk(LK_INFO, "handler already installed on irq_type %d\n", irq_no);
        return -1;
    }
    irq->handler = handler;

    if (0 != adv7627_enable_irq(state, irq_no, true))
        return -1;

    return 0;
}

static int adv7627_unregister_irq(struct adv7627_state *state, uint32_t irq_no)
{
    struct adv7627_irq_mapping_s *irq;

    if (irq_no >= adv7627_int_max)
        return -1;

    irq = &adv7627_irq_mapping[irq_no];
    if (irq->handler == NULL) {
        printlk(LK_INFO, "No handler installed on irq_type %d\n", irq_no);
        return -1;
    }
    irq->handler = NULL;

    if (0 != adv7627_enable_irq(state, irq_no, false))
        return -1;

    return 0;
}

#ifdef LK
#define GET_PIN_VALUE(pin)  gpio_desc_get_value((struct gpio_desc *)pin)
#define SET_PIN_VALUE(pin, state)  gpio_desc_set_value((struct gpio_desc *)pin, state)
#define SLEEP_THREAD(x) thread_sleep(x)
#else
#define GET_PIN_VALUE(pin)  gpio_get_value_cansleep(*pin)
#define SET_PIN_VALUE(pin, state)  gpio_set_value_cansleep(*pin, state)
#define SLEEP_THREAD(x) usleep_range(x*1000, x*1000)
#endif

static int adv7627_reset(struct adv7627_state *state)
{
    uint32_t inp;
    int ret;

    printlk(LK_INFO, "Waiting for HDMI interrupts signals getting stable...\n");
    /* dummy read is needed to start getting interrupts whenever any input signal changes */

#ifdef HDMI_ADV7627_REGISTER_INT1
    do {
        ret = GET_PIN_VALUE(&state->gpio_int1);
        ASSERT(ret != -1);
        inp = ret;
        SLEEP_THREAD(125);
        printlk(LK_INFO, "Waiting for INT1...\n");
    } while (inp != 1);
    printlk(LK_INFO, "INT1 ok!\n");
#endif

#ifdef HDMI_ADV7627_REGISTER_INT2
    do {
        inp = 0;
        ret = GET_PIN_VALUE(&state->gpio_int2);
        ASSERT(ret != -1);
        inp |= (ret << 1);
        SLEEP_THREAD(125);
        printlk(LK_INFO, "Waiting for INT2...\n");
    } while (inp != 2);
    printlk(LK_INFO, "INT2 ok!\n");
#endif

    SLEEP_THREAD(500);

    /* HDMI power down */
    printlk(LK_INFO, "Initiating HDMI switch power up sequence...\n");
    SET_PIN_VALUE(&state->gpio_reset, 0);
    SLEEP_THREAD(20);
    SET_PIN_VALUE(&state->gpio_reset, 1);
    /* power up */
    SLEEP_THREAD(20);

    /* Set CBS to low */
    SET_PIN_VALUE(&state->gpio_cs, 0);
    SLEEP_THREAD(20);

    printlk(LK_INFO, "HDMI switch power up sequence done\n");
    #ifdef LK
    if (adv7627_config_irq_pins(state) != 0) {
    #else
    if (rpc_adv7627_config_irq_pins(state) != 0) {
    #endif
        return -1;
    }

    return 0;
}

static void adv7627_software_reset(struct adv7627_state *state)
{
    uint8_t slave_address = 0x58;
    uint8_t reg_val = 0x0;

    reg_val = 0XFF;
    adv_write_reg(state->i2c_bus, slave_address, 0xFF, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0xFF, 0xFF) # default=0x00, Reset
    SLEEP_US(10000);
//    bus.write_byte_data(0x58, 0xE5, 0x82) # default=0x00, ADI Required Write
    reg_val = 0X82;
    adv_write_reg(state->i2c_bus, slave_address, 0xE5, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0xE7, 0x90) # 0x48 default=0x00, Set DPLL_B_Map address
    reg_val = 0X90;
    adv_write_reg(state->i2c_bus, slave_address, 0xE7, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0xE9, 0x94) # 0x4A default=0x00, Set CP_B_Map address
    reg_val = 0X94;
    adv_write_reg(state->i2c_bus, slave_address, 0xE9, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0xEA, 0xF0) # 0x68 default=0x00, Set OSD_Map address
    reg_val = 0XF0;
    adv_write_reg(state->i2c_bus, slave_address, 0xEA, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0xED, 0x6C) # 0x36 default=0x00, Set EDID
    reg_val = 0X6C;
    adv_write_reg(state->i2c_bus, slave_address, 0xED, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0xEE, 0x64) # 0x32 default=0x00, Set EDID_Config_Map address
    reg_val = 0X64;
    adv_write_reg(state->i2c_bus, slave_address, 0xEE, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0xF0, 0x58) # 0x2C default=0x00, Set HDMI_Rx2_Map address
    reg_val = 0X58;
    adv_write_reg(state->i2c_bus, slave_address, 0xF0, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0xF2, 0xF4) # 0x7A default=0x00, Set Rx2_Repeater_Map_Address
    reg_val = 0XF4;
    adv_write_reg(state->i2c_bus, slave_address, 0xF2, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0xF3, 0xF6) # 0x6B default=0x00, Set Rx2_Infoframe_Map address
    reg_val = 0XF6;
    adv_write_reg(state->i2c_bus, slave_address, 0xF3, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0xF9, 0xF8) # 0x6C default=0x00, Set TxB_Main_Map address
    reg_val = 0XF8;
    adv_write_reg(state->i2c_bus, slave_address, 0xF9, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0xFA, 0x84) # 0x42 default=0x00, Set TxB_Edid_Map address
    reg_val = 0X84;
    adv_write_reg(state->i2c_bus, slave_address, 0xFA, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0xFB, 0xC0) # 0x60 default=0x00, Set TxB_Test_Map address
    reg_val = 0XC0;
    adv_write_reg(state->i2c_bus, slave_address, 0xFB, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0xFC, 0xFA) # 0x6D default=0x00, Set TxB_Packet_Memory_Map address
    reg_val = 0XFA;
    adv_write_reg(state->i2c_bus, slave_address, 0xFC, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0xFD, 0x8C) # 0x46 default=0x00, Set TxB_CEC_Map address
    reg_val = 0X8C;
    adv_write_reg(state->i2c_bus, slave_address, 0xFD, reg_val, ADV_I2C_XFER_WAIT);
}

static void adv7627_transceiver_mode(struct adv7627_state *state)
{
    uint8_t slave_address;
    uint8_t reg_val;

    slave_address = 0x58;
    reg_val = 0xE0;
    adv_write_reg(state->i2c_bus, slave_address, 0x00, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0x00, 0xE0) # default=0xF3, Power Up DPLL, xtal

    slave_address = 0x7C;
    reg_val = 0x10;
    adv_write_reg(state->i2c_bus, slave_address, 0x41, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x41, 0x10) # default=0x50, Power up Tx

    slave_address = 0x48;
    reg_val = 0x00;
    adv_write_reg(state->i2c_bus, slave_address, 0xB5, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x48, 0xB5, 0x00) # default=0x01, DPLL Set 128Fs MCLK Out

    slave_address = 0x58;
    reg_val = 0x78;
    adv_write_reg(state->i2c_bus, slave_address, 0x01, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0x01, 0x78) # default=0x01, HDMI Receiver enabled, RxA-Rx

//    reg_val = 0x02;
    reg_val = 0x00;
    adv_write_reg(state->i2c_bus, slave_address, 0x08, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0x08, 0x02) # default=0x00, Tx Audio Insert Config - Audio Rx-Tx

    reg_val = 0x51;
    adv_write_reg(state->i2c_bus, slave_address, 0x09, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0x09, 0x51) # default=0x55, ADI Required Write

    reg_val = 0x02;
    adv_write_reg(state->i2c_bus, slave_address, 0x0A, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0x0A, 0x02) # default=0x00# Rx Audio Extract Config - Audio Rx-AP_OUT

    reg_val = 0x0C;
    adv_write_reg(state->i2c_bus, slave_address, 0x0F, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0x0F, 0x0C) # default=0x08, Audio Input Port Config - Use AP_IN as the

    reg_val = 0x1A;
    adv_write_reg(state->i2c_bus, slave_address, 0xB1, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0xB1, 0x1A) # default=0x9A, Hot Plug Config - configure as required in system
    reg_val = 0xDF;
    adv_write_reg(state->i2c_bus, slave_address, 0xB3, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0xB3, 0xDF) # default=0xDF, ADI Required Write
    reg_val = 0x00;
    adv_write_reg(state->i2c_bus, slave_address, 0x9E, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0x9E, 0x00) # default=0xFF, Turn off tristate on AP_OUT
    slave_address = 0x4A;
    reg_val = 0x2D;
    adv_write_reg(state->i2c_bus, slave_address, 0xC9, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x4A, 0xC9, 0x2D) # default=0x2C, CP_Lite Disable Auto Parameter Buffering -

}

static void adv7627_power_up_rx2_transceiver_mode(struct adv7627_state *state)
{
//#03-08 Power Up Rx2 after Rx Config - ADV7627 Transceiver Mode:
    uint8_t slave_address;
    uint8_t reg_val;

    slave_address = 0x58;
    reg_val = 0xA0;
    adv_write_reg(state->i2c_bus, slave_address, 0x00, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0x00, 0xA0) # default=0xF3, Power Up Rx, DPLL, xtal

}

static void adv7625_26_27_fixed_edid(struct adv7627_state *state)
{
//  def adv7625_26_27_fixed_edid():
    uint8_t slave_address;
    uint8_t reg_val;

    slave_address = 0x58;
    reg_val = 0x00;
    adv_write_reg(state->i2c_bus, slave_address, 0xB2, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x58, 0xB2, 0x00)   # Disable all ports */
    slave_address = 0x32;
    reg_val = 0x00;
    adv_write_reg(state->i2c_bus, slave_address, 0x74, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x32, 0x74, 0x00)   # Disable all edids */
    reg_val = 0x20;
    adv_write_reg(state->i2c_bus, slave_address, 0x70, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x32, 0x70, 0x20)   # Primary 256 byte */
    reg_val = 0x00;
    adv_write_reg(state->i2c_bus, slave_address, 0x7E, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x32, 0x7E, 0x00)   # Secondary 0 byte */
    reg_val = 0x40;
    adv_write_reg(state->i2c_bus, slave_address, 0x72, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x32, 0x72, 0x40)   # Ext eeprom tristate */
    slave_address = 0x7A;
    reg_val = 0x00;
    adv_write_reg(state->i2c_bus, slave_address, 0x74, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x7a, 0x74, 0x00)
    reg_val = 0x20;
    adv_write_reg(state->i2c_bus, slave_address, 0x70, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x7a, 0x70, 0x20)
    reg_val = 0x00;
    adv_write_reg(state->i2c_bus, slave_address, 0x7E, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x7a, 0x7E, 0x00)
    reg_val = 0x40;
    adv_write_reg(state->i2c_bus, slave_address, 0x72, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x7a, 0x72, 0x40)

    slave_address = 0x36;
#ifdef LK
    i2c_write_reg_bytes(* (int *)state->i2c_bus, slave_address, 0, (const uint8_t *)edid, sizeof(edid));
#else
    raw_i2c_write_block((struct i2c_client *)state->i2c_bus, slave_address, 0, (uint8_t *)edid, sizeof(edid));
#endif

    slave_address = 0x7A;
    reg_val = 0xB8;
    adv_write_reg(state->i2c_bus, slave_address, 0x71, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x7A, 0x71, 0xB8)
    reg_val = 0x20;
    adv_write_reg(state->i2c_bus, slave_address, 0x52, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x7A, 0x52, 0x20)
    reg_val = 0x00;
    adv_write_reg(state->i2c_bus, slave_address, 0x53, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x7a, 0x53, 0x00)
    reg_val = 0x30;
    adv_write_reg(state->i2c_bus, slave_address, 0x54, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x7a, 0x54, 0x30)
    reg_val = 0x00;
    adv_write_reg(state->i2c_bus, slave_address, 0x55, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x7a, 0x55, 0x00)
    reg_val = 0x40;
    adv_write_reg(state->i2c_bus, slave_address, 0x56, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x7a, 0x56, 0x40)
    reg_val = 0x00;
    adv_write_reg(state->i2c_bus, slave_address, 0x57, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x7a, 0x57, 0x00)
    reg_val = 0x50;
    adv_write_reg(state->i2c_bus, slave_address, 0x58, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x7a, 0x58, 0x50)
    reg_val = 0x00;
    adv_write_reg(state->i2c_bus, slave_address, 0x59, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x7a, 0x59, 0x00)
    reg_val = 0x04;
    adv_write_reg(state->i2c_bus, slave_address, 0x77, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x7a, 0x77, 0x04)
    reg_val = 0x1F;
    adv_write_reg(state->i2c_bus, slave_address, 0x74, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x7a, 0x74, 0x1f)

    slave_address = 0x32;
    reg_val = 0xB8;
    adv_write_reg(state->i2c_bus, slave_address, 0x71, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x32, 0x71, 0xB8)   # Set the SPA location (Primary Port) */
    reg_val = 0x20;
    adv_write_reg(state->i2c_bus, slave_address, 0x52, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x32, 0x52, 0x20)   # Set the SPA for port B */
    reg_val = 0x00;
    adv_write_reg(state->i2c_bus, slave_address, 0x53, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x32, 0x53, 0x00)   # Set the SPA for port B */
    reg_val = 0x30;
    adv_write_reg(state->i2c_bus, slave_address, 0x54, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x32, 0x54, 0x30)   # Set the SPA for port C */
    reg_val = 0x00;
    adv_write_reg(state->i2c_bus, slave_address, 0x55, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x32, 0x55, 0x00)   # Set the SPA for port C */
    reg_val = 0x40;
    adv_write_reg(state->i2c_bus, slave_address, 0x56, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x32, 0x56, 0x40)   # Set the SPA for port D */
    reg_val = 0x00;
    adv_write_reg(state->i2c_bus, slave_address, 0x57, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x32, 0x57, 0x00)   # Set the SPA for port D */
    reg_val = 0x50;
    adv_write_reg(state->i2c_bus, slave_address, 0x58, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x32, 0x58, 0x50)   # Set the SPA for port E */
    reg_val = 0x00;
    adv_write_reg(state->i2c_bus, slave_address, 0x59, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x32, 0x59, 0x00)   # Set the SPA for port E */
    reg_val = 0x04;
    adv_write_reg(state->i2c_bus, slave_address, 0x77, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x32, 0x77, 0x04)   # Recalculate checksums */
    reg_val = 0x1F;
    adv_write_reg(state->i2c_bus, slave_address, 0x74, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x32, 0x74, 0x1F)   # Enable the Internal EDID for all 5 Ports */

    slave_address = 0x58;
    reg_val = 0x1F;
    adv_write_reg(state->i2c_bus, slave_address, 0xB2, reg_val, ADV_I2C_XFER_WAIT);
//      bus.write_byte_data(0x58, 0xB2, 0x1F)   # Enable all 5 ports *
//
}

static void adv7625_26_27_hdmi_txb_ycbcr_165mhz_tx_source_on(struct adv7627_state *state)
{
//#05-04 ADV7625/26/27 HDMI TxB YCbCr Out - TMDS Clock greater than 165MHz - Tx Source Term ON:
//def adv7625_26_27_hdmi_txb_ycbcr_165mhz_tx_source_on():
    uint16_t slave_address;
    uint8_t reg_val;

    slave_address = 0x60;
    reg_val = 0x42;
    adv_write_reg(state->i2c_bus, slave_address, 0x24, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x60, 0x24, 0x42) # default=0x00, ADI Required Write - HDMI TxB Config
    slave_address = 0x7C;
    reg_val = 0x00;
    adv_write_reg(state->i2c_bus, slave_address, 0x01, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x01, 0x00) # default=0x00, Set N Value(644) used for Audio 48kHz
    reg_val = 0x18;
    adv_write_reg(state->i2c_bus, slave_address, 0x02, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x02, 0x18) # default=0x00, Set N Value(644) used for Audio 48kHz
    reg_val = 0x00;
    adv_write_reg(state->i2c_bus, slave_address, 0x03, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x03, 0x00) # default=0x00, Set N Value(644) used for Audio 48kHz
//    #bus.write_byte_data(0x7c, 0x01, 0x00)
//    #bus.write_byte_data(0x7c, 0x02, 0x60)
//    #bus.write_byte_data(0x7c, 0x03, 0x00)
//    #bus.write_byte_data(0x7c, 0x0a, 0x34)
//    #bus.write_byte_data(0x7c, 0x0c, 0xbc)
//    #bus.write_byte_data(0x7c, 0x15, 0xe0)
//    #bus.write_byte_data(0x7c, 0x47, 0x00)
//    #bus.write_byte_data(0x7c, 0x47, 0x40)
//    #bus.write_byte_data(0x7c, 0x47, 0x00)
//    #bus.write_byte_data(0x7c, 0x73, 0x07)
//    #bus.write_byte_data(0x7c, 0x76, 0x1f)

    reg_val = 0xFF;
    adv_write_reg(state->i2c_bus, slave_address, 0x13, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x13, 0xFF) # default=0x00, Set Tx Category Code
    reg_val = 0x20;
    adv_write_reg(state->i2c_bus, slave_address, 0x15, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x15, 0x20) # default=0x00, CS I2S Fs = 48kHz, Video Input Format = RGB/YCbCr 444
    reg_val = 0x61;
    adv_write_reg(state->i2c_bus, slave_address, 0x16, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x16, 0x61) # default=0x00, Output video format YCbCr 444, Input Video YCbCr
    reg_val = 0x80;
    adv_write_reg(state->i2c_bus, slave_address, 0x40, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x40, 0x80) # default=0x00, Enable Global Control Packet
    reg_val = 0x04;
    adv_write_reg(state->i2c_bus, slave_address, 0x4C, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x4C, 0x04) # default=0x00, 8-bit Deep Colour Mode
    reg_val = 0x40;
    adv_write_reg(state->i2c_bus, slave_address, 0x55, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x55, 0x40) # default=0x00, AVI Infoframe - Y1Y0 = YCbCr 444
    reg_val = 0x08;
    adv_write_reg(state->i2c_bus, slave_address, 0x56, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x56, 0x08) # default=0x00, AVI Infoframe - R[3:0] = Active Format Apsect Ratio - Same as aspect ratio
    reg_val = 0x01;
    adv_write_reg(state->i2c_bus, slave_address, 0x73, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x73, 0x01) # default=0x00, Audio IF CC = 001, 2 channels
    reg_val = 0x20;
    adv_write_reg(state->i2c_bus, slave_address, 0x96, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x96, 0x20) # default=0x00, Configure TxB Interrupts
    reg_val = 0x16;
    adv_write_reg(state->i2c_bus, slave_address, 0xAF, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0xAF, 0x16) # default=0x14, Set HDMI Mode Output
    reg_val = 0x70;
    adv_write_reg(state->i2c_bus, slave_address, 0xBA, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0xBA, 0x70) # default=0x00, ADI Required Write - HDMI TxB Config
    reg_val = 0x44;
    adv_write_reg(state->i2c_bus, slave_address, 0xD0, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0xD0, 0x44) # default=0x4C, ADI Required Write - HDMI TxB Config
    reg_val = 0x3C;
    adv_write_reg(state->i2c_bus, slave_address, 0xD1, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0xD1, 0x3C) # default=0x38, ADI Required Write - HDMI TxB Config
    reg_val = 0x07;
    adv_write_reg(state->i2c_bus, slave_address, 0xD3, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0xD3, 0x07) # default=0x06, ADI Required Write - HDMI TxB Config
    reg_val = 0x02;
    adv_write_reg(state->i2c_bus, slave_address, 0xD6, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0xD6, 0x02) # default=0x06, ADI Required Write - HDMI TxB Config
    reg_val = 0x0B;
    adv_write_reg(state->i2c_bus, slave_address, 0xDB, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0xDB, 0x0B) # default=0x0A, ADI Required Write - HDMI TxB Config
    reg_val = 0x90;
    adv_write_reg(state->i2c_bus, slave_address, 0xE0, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0xE0, 0x90) # default=0x00, ADI Required Write - HDMI TxB Config
    reg_val = 0xFC;
    adv_write_reg(state->i2c_bus, slave_address, 0xE1, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0xE1, 0xFC) # default=0x0C, ADI Required Write - HDMI TxB Config
    reg_val = 0xD0;
    adv_write_reg(state->i2c_bus, slave_address, 0xE3, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0xE3, 0xD0) # default=0x00, ADI Required Write - HDMI TxB Config
    reg_val = 0xF0;
    adv_write_reg(state->i2c_bus, slave_address, 0xE8, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0xE8, 0xF0) # default=0x10, ADI Required Write - HDMI TxB Config
    reg_val = 0x01;
    adv_write_reg(state->i2c_bus, slave_address, 0xF3, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0xF3, 0x01) # default=0x00, ADI Required Write - HDMI TxB Config
    reg_val = 0xCC;
    adv_write_reg(state->i2c_bus, slave_address, 0xF5, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0xF5, 0xCC) # default=0xCC, ADI Required Write - HDMI TxB Config
    reg_val = 0x08;
    adv_write_reg(state->i2c_bus, slave_address, 0xF6, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0xF6, 0x08) # default=0x00, ADI Required Write - HDMI TxB Config
    reg_val = 0xF0;
    adv_write_reg(state->i2c_bus, slave_address, 0xF7, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0xF7, 0xF0) # default=0xFF, ADI Required Write - HDMI TxB Config
    reg_val = 0x40;
    adv_write_reg(state->i2c_bus, slave_address, 0xDA, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0xDA, 0x40) # default=0x00, ADI Required Write - HDMI TxB Config
    reg_val = 0xD4;
    adv_write_reg(state->i2c_bus, slave_address, 0xF5, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0xF5, 0xD4) # default=0xCC, ADI Required Write - HDMI TxB Config
    reg_val = 0x33;
    adv_write_reg(state->i2c_bus, slave_address, 0x81, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x81, 0x33) # default=0x88, TxB Charge Injection Ch0 = 3, TxB Charge Injection Ch1 = 3
    reg_val = 0x33;
    adv_write_reg(state->i2c_bus, slave_address, 0x82, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x82, 0x33) # default=0x88, TxB Charge Injection Ch2 = 3, TxB Charge Injection Clk = 3
    reg_val = 0x03;
    adv_write_reg(state->i2c_bus, slave_address, 0x83, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x83, 0x03) # default=0x00, ADI Required Write - HDMI TxB Source Termination ON
    reg_val = 0x03;
    adv_write_reg(state->i2c_bus, slave_address, 0x84, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x84, 0x03) # default=0x00, ADI Required Write - HDMI TxB Source Termination ON
    reg_val = 0x03;
    adv_write_reg(state->i2c_bus, slave_address, 0x85, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x85, 0x03) # default=0x00, ADI Required Write - HDMI TxB Source Termination ON
    reg_val = 0x03;
    adv_write_reg(state->i2c_bus, slave_address, 0x86, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x86, 0x03) # default=0x00, ADI Required Write - HDMI TxB Source Termination ON
    reg_val = 0x3D;
    adv_write_reg(state->i2c_bus, slave_address, 0xEA, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0xEA, 0x3D) # default=0x00, ADI Required Write - HDMI TxB Source Termination ON
    reg_val = 0x38;
    adv_write_reg(state->i2c_bus, slave_address, 0xED, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0xED, 0x38) # default=0x80, ADI Required Write - HDMI TxB Source Termination ON
    reg_val = 0x38;
    adv_write_reg(state->i2c_bus, slave_address, 0xEE, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0xEE, 0x38) # default=0x80, ADI Required Write - HDMI TxB Source Termination ON
    reg_val = 0x39;
    adv_write_reg(state->i2c_bus, slave_address, 0xEF, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0xEF, 0x39) # default=0x82, ADI Required Write - HDMI TxB Source Termination ON
    reg_val = 0x55;
    adv_write_reg(state->i2c_bus, slave_address, 0xFC, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0xFC, 0x55) # default=0x05, ADI Required Write - HDMI TxB Config
    reg_val = 0x30;
    adv_write_reg(state->i2c_bus, slave_address, 0x41, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x41, 0x30) # default=0x50, ADI Required Write - HDMI TxB Config
    reg_val = 0x10;
    adv_write_reg(state->i2c_bus, slave_address, 0x41, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x41, 0x10) # default=0x50, ADI Required Write - HDMI TxB Config

}
static void adv7625_26_27_clear_spi_tristate(struct adv7627_state *state)
{
//##15 ADV7625/26/27 Clear SPI##
//#15-01 ADV7625/26/27 Clear SPI Tristate:
//def adv7625_26_27_clear_spi_tristate():
    uint16_t slave_address;
    uint8_t reg_val;

    slave_address = 0x41;
    reg_val = 0x00;
    adv_write_reg(state->i2c_bus, slave_address, 0x1A, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x41, 0x1A, 0x00) # default=0x04, Clear SPI Line Tristate

}
static void adv7625_26_27_txb_ycrcb_444_ycrcb_444(struct adv7627_state *state)
{
//##09 ADV7625/26/27 TxB Video Modes##
//#09-01 TxB YCrCb 444 input-YCrCb 444 output:
//def adv7625_26_27_txb_ycrcb_444_ycrcb_444():
    uint16_t slave_address;
    uint8_t reg_val;

    slave_address = 0x7C;
    reg_val = 0x20;
    adv_write_reg(state->i2c_bus, slave_address, 0x15, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x15, 0x20) # default=0x00, CS I2S Fs = 48kHz, Video Input Format = RGB/YCbCr 444
    reg_val = 0x61;
    adv_write_reg(state->i2c_bus, slave_address, 0x16, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x16, 0x61) # default=0x00, Output video format YCbCr 444, Input Video YCbCr
    reg_val = 0x40;
    adv_write_reg(state->i2c_bus, slave_address, 0x55, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x55, 0x40) # default=0x00, AVI Infoframe - Y1Y0 = YCbCr 444
//

}
void adv7625_26_27_txb_rgb_444_rgb_444(struct adv7627_state *state)
{
//##09 ADV7625/26/27 TxB Video Modes##
//#09-01 TxB RGB 444 input-RGB 444 output:
//def adv7625_26_27_txb_rgb_444_rgb_444():
    uint16_t slave_address;
    uint8_t reg_val;

    slave_address = 0x7C;
    reg_val = 0x20;
    adv_write_reg(state->i2c_bus, slave_address, 0x15, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x15, 0x20) # default=0x00, CS I2S Fs = 48kHz, Video Input Format = RGB/YCbCr 444
    reg_val = 0x20;
    adv_write_reg(state->i2c_bus, slave_address, 0x16, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x16, 0x20) # default=0x00, Output video format RGB 444, Input Video RGB
    reg_val = 0x20;
    adv_write_reg(state->i2c_bus, slave_address, 0x55, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x55, 0x20) # default=0x00, AVI Infoframe - Y1Y0 = RGB RGB
//

}

#if 0
static void adv7625_26_27_txb_8ch_pcm_hbr_i2s_192khz(struct adv7627_state *state)
{
//#07-29 TxB 8-ch HBR I2S 192kHz:
//def adv7625_26_27_txb_8ch_pcm_hbr_i2s_192khz():
    uint16_t slave_address;
    uint8_t reg_val;

    slave_address = 0x7C;
    reg_val = 0x00;
    adv_write_reg(state->i2c_bus, slave_address, 0x01, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x01, 0x00) # default=0x00, Set N Value(24576) used for Audio 192kHz
    reg_val = 0x60;
    adv_write_reg(state->i2c_bus, slave_address, 0x02, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x02, 0x60) # default=0x00, Set N Value(24576) used for Audio 192kHz
    reg_val = 0x00;
    adv_write_reg(state->i2c_bus, slave_address, 0x03, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x03, 0x00) # default=0x00, Set N Value(24576) used for Audio 192kHz
    reg_val = 0x34;
    adv_write_reg(state->i2c_bus, slave_address, 0x0A, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x0A, 0x34) # default=0x01, HBR mode, internally generated CTS, audio_mode = 4 HBR I2S streams, mclk_ratio = 128Fs
    reg_val = 0xBC;
    adv_write_reg(state->i2c_bus, slave_address, 0x0C, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x0C, 0xBC) # default=0xBC, Use sampling frequency from registers, use CS data from i2s stream, 4 x I2S line enabled
    reg_val = 0x90;
    adv_write_reg(state->i2c_bus, slave_address, 0x15, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x15, 0x90) # default=0x00, CS I2S Fs = 768kHz for HBR, Video Input Format = RGB/YCbCr 444
    reg_val = 0x00;
    adv_write_reg(state->i2c_bus, slave_address, 0x47, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x47, 0x00) # default=0x00, PaPb_sync = 0b - Synchronizes the Pa and Pb syncwords with subpacket 0 in HBR Audio
    reg_val = 0x40;
    adv_write_reg(state->i2c_bus, slave_address, 0x47, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x47, 0x40) # default=0x00, PaPb_sync = 1b - Synchronizes the Pa and Pb syncwords with subpacket 0 in HBR Audio
    reg_val = 0x00;
    adv_write_reg(state->i2c_bus, slave_address, 0x47, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x47, 0x00) # default=0x00, PaPb_sync = 0b - Synchronizes the Pa and Pb syncwords with subpacket 0 in HBR Audio
    reg_val = 0x07;
    adv_write_reg(state->i2c_bus, slave_address, 0x73, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x73, 0x07) # default=0x00, Audio IF CC = 111, 8 channels
    reg_val = 0x1F;
    adv_write_reg(state->i2c_bus, slave_address, 0x76, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x7C, 0x76, 0x1F) # default=0x00, Audio IF CA=0x1F, 8 channels
}
#endif

static void adv7625_26_27_hdmi_rx2_297mhz(struct adv7627_state *state,  hdmi_audio_fmt_t fmt)
{
//#03-04 ADV7625/26/27 HDMI Rx2 Configuration - input TMDS Clock equal to 297MHz:
//def adv7625_26_27_hdmi_rx2_297mhz():
    uint8_t slave_address;
    uint8_t reg_val;

    slave_address = 0x2C;
    reg_val = 0x04;
    adv_write_reg(state->i2c_bus, slave_address, 0x01, reg_val, ADV_I2C_XFER_WAIT);

    if (fmt == HDMI_AUDIO_FMT_61937)
        reg_val = 0x98; // i2s_out_mode = I2S Mode
    else
        reg_val = 0xf8; // i2s_out_mode = Raw SPDIF (IEC60958) Mode

    adv_write_reg(state->i2c_bus, slave_address, 0x03, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x2C, 0x03, 0x98) # default=0x18, Disables the zeroing of I2S data
    reg_val = 0x79;
    adv_write_reg(state->i2c_bus, slave_address, 0x3E, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x2C, 0x3E, 0x79) # default=0x79, ADI Required Write - HDMI Rx2 Config
    reg_val = 0x04;
    adv_write_reg(state->i2c_bus, slave_address, 0x3F, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x2C, 0x3F, 0x04) # default=0x63, ADI Required Write - HDMI Rx2 Config
    reg_val = 0xFE;
    adv_write_reg(state->i2c_bus, slave_address, 0x4E, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x2C, 0x4E, 0xFE) # default=0x7B, ADI Required Write - HDMI Rx2 Config
    reg_val = 0x18;
    adv_write_reg(state->i2c_bus, slave_address, 0x4F, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x2C, 0x4F, 0x18) # default=0x63, ADI Required Write - HDMI Rx2 Config
    reg_val = 0xF5;
    adv_write_reg(state->i2c_bus, slave_address, 0x57, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x2C, 0x57, 0xF5) # default=0x30, ADI Required Write - HDMI Rx2 Config
    reg_val = 0x1C;
    adv_write_reg(state->i2c_bus, slave_address, 0x58, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x2C, 0x58, 0x1C) # default=0x01, ADI Required Write - HDMI Rx2 Config
    reg_val = 0x04;
    adv_write_reg(state->i2c_bus, slave_address, 0x6F, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x2C, 0x6F, 0x04) # default=0x40, ADI Required Write - HDMI Rx2 Config
    reg_val = 0x00;
    adv_write_reg(state->i2c_bus, slave_address, 0x75, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x2C, 0x75, 0x00) # default=0x00, ADI Required Write - HDMI Rx2 Config
    reg_val = 0x10;
    adv_write_reg(state->i2c_bus, slave_address, 0x85, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x2C, 0x85, 0x10) # default=0x16, ADI Required Write - HDMI Rx2 Config
    reg_val = 0xC0;
    adv_write_reg(state->i2c_bus, slave_address, 0x97, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x2C, 0x97, 0xC0) # default=0x80, ADI Required Write - HDMI Rx2 Config
    reg_val = 0x3F;
    adv_write_reg(state->i2c_bus, slave_address, 0x98, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x2C, 0x98, 0x3F) # default=0xFF, ADI Required Write - HDMI Rx2 Config - for input TMDS Clock 297MHz
    reg_val = 0xE3;
    adv_write_reg(state->i2c_bus, slave_address, 0x99, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x2C, 0x99, 0xE3) # default=0xA3, ADI Required Write - HDMI Rx2 Config - for input TMDS Clock 297MHz
    reg_val = 0x9F;
    adv_write_reg(state->i2c_bus, slave_address, 0x9A, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x2C, 0x9A, 0x9F) # default=0xFF, ADI Required Write - HDMI Rx2 Config - for input TMDS Clock 297MHz
    reg_val = 0x01;
    adv_write_reg(state->i2c_bus, slave_address, 0x9B, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x2C, 0x9B, 0x01) # default=0x03, ADI Required Write - HDMI Rx2 Config - for input TMDS Clock 297MHz
    reg_val = 0x10;
    adv_write_reg(state->i2c_bus, slave_address, 0x9C, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x2C, 0x9C, 0x10) # default=0x08, ADI Required Write - HDMI Rx2 Config
    reg_val = 0x01;
    adv_write_reg(state->i2c_bus, slave_address, 0xCB, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x2C, 0xCB, 0x01) # default=0x00, ADI Required Write - HDMI Rx2 Config

}

int HDMI_I2C_Config(struct adv7627_state *state)
{
    uint8_t slave_address, reg_val;
    uint8_t rev_id1, rev_id2;
    uint8_t tx_edid_stat1, tx_edid_stat2, tx_edid_read;
    unsigned char edid_read[256];

    printlk(LK_INFO, "\r\n\tDefault config for adv7627\r\n");

    if(adv7627_reset(state) < 0) {
        printlk(LK_INFO, "\r\n ADV7627: Failed to Reset\r\n");
        return -1;
    }

    adv7627_software_reset(state);

    /* TODO: read main slave address from register */
    slave_address = 0x58;
    adv_read_reg(state->i2c_bus, slave_address, 0xDF, &rev_id1, 0);
//    rev_id1 = bus.read_byte_data(0x58, 0xDF)
    adv_read_reg(state->i2c_bus, slave_address, 0xE0, &rev_id2, 0);
//    rev_id2 = bus.read_byte_data(0x58, 0xE0)
    printlk(LK_INFO, "\r\n Rev. %d , %d \r\n", rev_id1, rev_id2);

    adv7627_transceiver_mode(state);
    adv7627_power_up_rx2_transceiver_mode(state);
    adv7625_26_27_fixed_edid(state);

    adv7625_26_27_hdmi_txb_ycbcr_165mhz_tx_source_on(state);
    adv7625_26_27_clear_spi_tristate(state);
    adv7625_26_27_txb_ycrcb_444_ycrcb_444(state);

    reg_val = 0x7A;
    adv_write_reg(state->i2c_bus, slave_address, 0x01, reg_val, ADV_I2C_XFER_WAIT);
//    bus.write_byte_data(0x58, 0x01, 0x7a) #set to RXC default

    slave_address = 0x32;
    adv_read_reg(state->i2c_bus, slave_address, 0x7A, &tx_edid_stat1, 0);
//    tx_edid_stat1 = bus.read_byte_data(0x32, 0x7a)
    slave_address = 0x7C;
    adv_read_reg(state->i2c_bus, slave_address, 0x7D, &tx_edid_stat2, 0);
//    tx_edid_stat2 = bus.read_byte_data(0x7c, 0x7D)
    printlk(LK_INFO, "\r\n tx_edid_stat1 %d \r\n", tx_edid_stat1);
    printlk(LK_INFO, "\r\n tx_edid_stat2 %d \r\n", tx_edid_stat2);

    /* print out TX EDID content for debug */
    slave_address = 0x42;
#ifdef LK
    i2c_read_reg_bytes(* (int *)state->i2c_bus, slave_address, 0, edid_read, 256);
#else
    raw_i2c_read_block((struct i2c_client *)state->i2c_bus, slave_address, 0, edid_read, 256);
#endif
    printlk(LK_INFO, "EDID:\n");
    hexdump8_ex(edid_read, 256, 0);

    printlk(LK_INFO, "\r\n\t HDMI configured..\r\n");
    slave_address = 0x58;
    adv_read_reg(state->i2c_bus, slave_address, 0x37, &tx_edid_read, 0);
    printlk(LK_INFO, "interrupt status at 0x37 = %02X\n", tx_edid_read);
    adv_read_reg(state->i2c_bus, slave_address, 0x43, &tx_edid_read, 0);
    printlk(LK_INFO, "interrupt status at 0x43 = %02X\n", tx_edid_read);
    adv_read_reg(state->i2c_bus, slave_address, 0x44, &tx_edid_read, 0);
    printlk(LK_INFO, "interrupt status at 0x44 = %02X\n", tx_edid_read);

    slave_address = 0x7a;
    adv_read_reg(state->i2c_bus, slave_address, 0x42, &tx_edid_read, 0);
    printlk(LK_INFO, "repeater interrupt status at 0x42 = %02X\n", tx_edid_read);
    adv_read_reg(state->i2c_bus, slave_address, 0x41, &tx_edid_read, 0);
    printlk(LK_INFO, "repeater interrupt status at 0x41 = %02X\n", tx_edid_read);

    slave_address = 0x7c;
    adv_read_reg(state->i2c_bus, slave_address, 0x42, &tx_edid_read, 0);
    printlk(LK_INFO, "TXB HPD status at 0x42 = %02X\n", tx_edid_read);
    adv_read_reg(state->i2c_bus, slave_address, 0xc8, &tx_edid_read, 0);
    printlk(LK_INFO, "TXB EDID interrupt status at 0xc8 = %02X\n", tx_edid_read);

    adv_read_reg(state->i2c_bus, slave_address, 0xE4, &tx_edid_read, 0);
    printlk(LK_INFO, "TXB interrupt status at 0xE4 = %02X\n", tx_edid_read);

    return 0;
}

/************************
 **** HDMI Specific *****
 ************************/

/* HDMI open */
status_t _hdmi_open(struct adv7627_state *state)
{
    unsigned irq_no;
    int ret;
    for (irq_no = 0; irq_no < ARRAY_SIZE(adv7627_irq_mapping); irq_no++) {
        struct adv7627_irq_mapping_s *irq = &adv7627_irq_mapping[irq_no];
        printlk(LK_INFO, "Installing irq %s\n", irq->name);
        ret = adv7627_register_irq(state,
                               hdmi_adv7627_irq_handler,
                               irq_no);
        if (ret) {
            printlk(LK_INFO, "Error while installing %s irq\n", irq->name);
            return -1;
        }
    }
    return 0;
}

/* HDMI close */
status_t _hdmi_close(struct adv7627_state *state)
{
    unsigned irq_no;
    int ret;
    for (irq_no = 0; irq_no < ARRAY_SIZE(adv7627_irq_mapping); irq_no++) {
        struct adv7627_irq_mapping_s *irq = &adv7627_irq_mapping[irq_no];
        printlk(LK_INFO, "De-installing irq %s\n", irq->name);
        ret = adv7627_unregister_irq(state, irq_no);
        if (ret) {
            printlk(LK_INFO, "Error while unregistering %s irq\n", irq->name);
            return -1;
        }
    }
    return 0;
}

/* HDMI get capabilities */
status_t _hdmi_get_capabilities(uint32_t *capabilities)
{
    *capabilities = \
            HDMI_CAP_AUDIO_SAMPLE_RATE_CHANGE \
            | HDMI_CAP_AUDIO_CHANNEL_STATUS \
            | HDMI_CAP_AUDIO_LINK_CHANGE \
            | HDMI_CAP_AUDIO_INFOFRAME \
            | HDMI_CAP_AUDIO_FMT_61937 \
            | HDMI_CAP_AUDIO_FMT_CUSTOM;

    return 0;
}

/* Set audio format */
status_t _hdmi_set_audio_format(struct adv7627_state *state, hdmi_audio_direction_t direction, hdmi_audio_fmt_t fmt)
{
    /* Support only Audio output port configuration */
    if (direction != HDMI_AUDIO_SOURCE) {
        printlk(LK_INFO, "Only audio source supported: Extract audio sample from HDMI\n");
        return -1;
    }

    switch(fmt) {
    case HDMI_AUDIO_FMT_60958:
        printlk(LK_INFO, "Configuring audio output format to 60958\n");
        printlk(LK_INFO, "IEC90958 is not supported as audio output format\n");
        break;

    case HDMI_AUDIO_FMT_61937:
        printlk(LK_INFO, "Configuring audio output format to 61937\n");
        adv7625_26_27_hdmi_rx2_297mhz(state, fmt);
        break;

    case HDMI_AUDIO_FMT_CUSTOM:
        printlk(LK_INFO, "Configuring audio output format to custom format (AES3)\n");
        adv7625_26_27_hdmi_rx2_297mhz(state, fmt);
        break;

    default:
        printlk(LK_INFO, "Invalid audio output format requested (%x)\n", fmt);
        return -1;
    }
    return 0;
}

/* Set audio packet */
status_t _hdmi_set_audio_packet(struct adv7627_state *state, hdmi_audio_direction_t direction, hdmi_audio_pkt_t pkt)
{
    return 0;
}

/* Set audio interface */
status_t _hdmi_set_audio_interface(struct adv7627_state *state,
                                    hdmi_audio_direction_t direction, 
                                    hdmi_audio_if_t in, hdmi_audio_if_t out)
{
    if ((direction != HDMI_AUDIO_SINK) 
        && (direction != HDMI_AUDIO_SOURCE)) {
        return -1;
    }

    if (direction == HDMI_AUDIO_SOURCE) {
        /* HDMI_AUDIO_SOURCE */
        switch(in) {
        case HDMI_AUDIO_IF_NONE:
            printlk(LK_INFO, "Configuring audio output source interface to None\n");
            /* AP2_OUT_SOURCE_SEL : No audio */
            adv_io_write_reg(state, 0xA, 0x0);
            break;
        case HDMI_AUDIO_IF_HDMI:
            printlk(LK_INFO, "Configuring audio output source interface to HDMI\n");
            /* AP2_OUT_SOURCE_SEL : Audio from HDMI RX2 */
            adv_io_write_reg(state, 0xA, 0x2);
            break;
        case HDMI_AUDIO_IF_SPDIF:
            printlk(LK_INFO, "Configuring audio output source interface to SPDIF\n");
            printlk(LK_INFO, "SPDIF is not supported as audio output source\n");
            return -1;
        case HDMI_AUDIO_IF_ARC:
            printlk(LK_INFO, "Configuring audio output source interface to ARC\n");
            /* AP2_OUT_SOURCE_SEL : ARC Audio from HDMI TxB */
            adv_io_write_reg(state, 0xA, 0x4);
            break;
        case HDMI_AUDIO_IF_I2S:
            printlk(LK_INFO, "Configuring audio output source interface to I2S\n");
            /* AP2_OUT_SOURCE_SEL : Audio from Audio Input Port 1 */
            adv_io_write_reg(state, 0xA, 0x5);
            break;
        default:
            printlk(LK_INFO, "Invalid audio output source interface requested (%x)\n", in);
            return -1;
        }
        switch(out) {
        case HDMI_AUDIO_IF_NONE:
            printlk(LK_INFO, "Configuring audio output sink interface to None\n");
            /* Audio output tristate ctrl 1: Everything in Hi-z */
            adv_io_write_reg(state, 0x9e, 0xff);
            break;
        case HDMI_AUDIO_IF_HDMI:
            printlk(LK_INFO, "HDMI is not supported as audio output sink\n");
            return -1;
            break;
        case HDMI_AUDIO_IF_SPDIF:
            printlk(LK_INFO, "SPDIF is not supported as audio output sink\n");
            return -1;
        case HDMI_AUDIO_IF_ARC:
            printlk(LK_INFO, "ARC is not supported as audio output sink\n");
            break;
        case HDMI_AUDIO_IF_I2S:
            printlk(LK_INFO, "Configuring audio output sink interface to I2S\n");
            /* hdmi_register_01: mux_hbr_out, ovr_mux_hbr - generate SPDIF */
            adv_rx_main_write_reg(state, 0x01, 0x1);

            /* hdmi_register_03: I2S_out_mode, 24bit */
            adv_rx_main_write_reg(state, 0x03, 0x98);

            break;
        default:
            printlk(LK_INFO, "Invalid audio output sink interface requested (%x)\n", out);
            return -1;
        }


    } else {
        /* HDMI_AUDIO_SINK */

        switch(in) {
        case HDMI_AUDIO_IF_NONE:
            printlk(LK_INFO, "Configuring audio input source interface to None\n");
            /* TXA_AUDIO_SOURCE_SEL : No audio */
            adv_io_write_reg(state, 0x8, 0x0);
            break;
        case HDMI_AUDIO_IF_HDMI:
            printlk(LK_INFO, "Configuring audio input source interface to HDMI\n");
            /* TXA_AUDIO_SOURCE_SEL : Audio from HDMI Rx2 */
            adv_io_write_reg(state, 0x8, 0x2);
            break;
        case HDMI_AUDIO_IF_SPDIF:
            printlk(LK_INFO, "SPDIF is not supported as audio input source\n");
            return -1;
            break;
        case HDMI_AUDIO_IF_ARC:
            printlk(LK_INFO, "ARC is not supported as audio input source\n");
            return -1;
            break;
        case HDMI_AUDIO_IF_I2S:
            printlk(LK_INFO, "Configuring audio input source interface to I2S\n");
            adv_io_write_reg(state, 0x8, 0x5);
            break;
        default:
            printlk(LK_INFO, "Invalid audio input source interface requested (%x)\n", in);
            return -1;
        }

        switch(out) {
        case HDMI_AUDIO_IF_NONE:
            printlk(LK_INFO, "Configuring audio input sink interface to None\n");
            break;
        case HDMI_AUDIO_IF_HDMI:
            printlk(LK_INFO, "Configuring audio input sink interface to HDMI\n");
            break;
        case HDMI_AUDIO_IF_SPDIF:
            printlk(LK_INFO, "SPDIF is not supported as audio input sink\n");
            return -1;
            break;
        case HDMI_AUDIO_IF_ARC:
            printlk(LK_INFO, "ARC is not supported as audio input sink\n");
            return -1;
            break;
        case HDMI_AUDIO_IF_I2S:
            printlk(LK_INFO, "I2S is not supported as audio input sink\n");
            return -1;
            break;
        default:
            printlk(LK_INFO, "Invalid audio input sink interface requested (%x)\n", out);
            return -1;
        }
    }
    return 0;
}

/* HDMI get channel status */
status_t _hdmi_get_channel_status(struct adv7627_state *state, hdmi_channel_status_t *channel_status)
{
    printlk(LK_INFO, "Get audio Channel status\n");

#ifdef HDMI_ADV7627_FORCE_AUDIO_CHANNEL_STATUS
    return hdmi_adv7627_get_channel_status(state, channel_status);
#else
    if (state->audio_channel_status_is_valid) {
        *channel_status = state->audio_cs;
        return 0;
    }
    /* We might have a change stream type, but infoframe has not yet been
     * updated. Return busy so the caller can request it later.
     */
    return -1;
#endif
}

/* HDMI get audio infoframe pkt */
status_t _hdmi_get_audio_infoframe_pkt(struct adv7627_state *state,
                                                hdmi_audio_infoframe_pkt_t *pkt)
{
    printlk(LK_INFO, "Get audio infoframe packet\n");

#ifdef HDMI_ADV7627_FORCE_AUDIO_INFOFRAME
    return hdmi_adv7627_get_audio_infoframe_pkt(state, pkt);
#else
    if (state->audio_info_pkt_is_valid) {
        *pkt = state->audio_info_pkt;
        return 0;
    }
    /* We might have a change stream type, but infoframe has not yet been
     * updated. Return busy so the caller can request it later.
     */
    return -1;
#endif
}

/* HDMI get audio custom fmt layout */
status_t _hdmi_get_audio_custom_fmt_layout(struct adv7627_state *state, iec60958_custom_fmt_layout_t *layout)
{
    static iec60958_custom_fmt_layout_t adv7627_custom_fmt_layout = {
        .data_start = 0,
        .data_length = 24,
        .validity_bit = 24,
        .user_bit = 25,
        .channel_status_bit = 26,
        .block_start_bit = 27,
        .parity_bit = -1,
        .width = 32,
    };

    printlk(LK_INFO, "Get audio custom format layout\n");

    *layout = adv7627_custom_fmt_layout;
    return 0;
}

/* HDMI get audio pkt type */
status_t _hdmi_get_audio_pkt_type(struct adv7627_state *state, hdmi_audio_pkt_t *pkt)
{
    printlk(LK_INFO, "Get audio packet type\n");
    *pkt = hdmi_adv7627_get_pkt_type(state);
    return 0;
}

/* HDMI get audio pkt layout */
status_t _hdmi_get_audio_pkt_layout(struct adv7627_state *state, hdmi_audio_pkt_layout_t *layout)
{
    printlk(LK_INFO, "Get audio packet layout\n");
    *layout = hdmi_adv7627_get_pkt_layout(state);
    return 0;
}