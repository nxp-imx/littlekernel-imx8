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

#include "ep92a7e.h"
#include "debug.h"


status_t ep_write_reg(i2c_bus *bus, uint8_t address, uint8_t reg, uint8_t val, unsigned wait_us)
{
    status_t ret;

    ret = _I2C_WRITE_REG(bus, address, reg, val);

    if (ret)
        return ret;

    SLEEP_US(wait_us);

    return 0;
}

status_t ep_read_reg(i2c_bus *bus, uint8_t address, uint8_t reg, uint8_t *val, unsigned wait_us)
{
    status_t ret;

    ret = _I2C_READ_REG(bus, address, reg, val);

    if (ret)
        return ret;

    SLEEP_US(wait_us);

    return 0;

}

status_t ep_read_regs(i2c_bus *bus, uint8_t address, uint8_t reg,
                            uint8_t *val, uint16_t cnt, unsigned wait_us)
{
    status_t ret;

    ret = _I2C_READ_REGS(bus, address, reg, val, cnt);

    if (ret)
        return ret;

    SLEEP_US(wait_us);

    return 0;

}
status_t ep_write_regs(i2c_bus *bus, uint8_t address, uint8_t reg,
                            uint8_t *val, uint16_t cnt, unsigned wait_us)
{
    status_t ret;

    ret = _I2C_WRITE_REGS(bus, address, reg, val, cnt);

    if (ret)
        return ret;

    SLEEP_US(wait_us);

    return 0;
}

static inline status_t ep_io_read_reg(struct ep92a7e_state *state, uint8_t reg, uint8_t *val)
{
    return ep_read_reg(state->i2c_bus, EP_SLAVE_ADDRESS, reg, val, 0);
}

static inline status_t ep_io_write_reg(struct ep92a7e_state *state, uint8_t reg, uint8_t val)
{
    return ep_write_reg(state->i2c_bus, EP_SLAVE_ADDRESS, reg, val, 0);
}

static inline status_t ep_rx_main_read_reg(struct ep92a7e_state *state, uint8_t reg, uint8_t *val)
{
    return ep_read_reg(state->i2c_bus, EP_SLAVE_ADDRESS, reg, val, 0);
}

static inline status_t ep_rx_main_write_reg(struct ep92a7e_state *state, uint8_t reg, uint8_t val)
{
    return ep_write_reg(state->i2c_bus, EP_SLAVE_ADDRESS, reg, val, 0);
}

/*************************
 *** EP92A7E specifics ***
 *************************/

#ifdef HDMI_EP92A7E_FORCE_AUDIO_PKT_LAYOUT
static hdmi_audio_pkt_layout_t hdmi_ep92a7e_get_pkt_layout(struct ep92a7e_state *state)
{
    uint8_t reg_val;
    uint16_t slave_address = EP_SLAVE_ADDRESS;
    hdmi_audio_pkt_layout_t pkt_layout;

    ep_read_reg(state->i2c_bus, slave_address, EP_REG_SYSTEM_STATUS_0, &reg_val, 0);

    reg_val = EP_FIELD_TO_VAL(EP_REG_SYSTEM_STATUS_LAYOUT, reg_val);
    if (reg_val)
        pkt_layout = HDMI_AUDIO_PKT_LAYOUT_1_8CH;
    else
        pkt_layout = HDMI_AUDIO_PKT_LAYOUT_0_2CH;

    printlk(LK_VERBOSE, "Packet channel layout: [%x]\n", pkt_layout);
    return pkt_layout;
}
#endif /* HDMI_EP92A7E_FORCE_AUDIO_PKT_LAYOUT */

#ifdef HDMI_EP92A7E_FORCE_AUDIO_PKT_TYPE
static hdmi_audio_pkt_t hdmi_ep92a7e_get_pkt_type(struct ep92a7e_state *state)
{
    uint8_t reg_val;
    uint16_t slave_address = EP_SLAVE_ADDRESS;
    hdmi_audio_pkt_t pkt;

    ep_read_reg(state->i2c_bus, slave_address, EP_REG_AUDIO_STATUS, &reg_val, 0);
    printlk(LK_VERBOSE, "Packet type detection, raw value: [%x]\n", reg_val);

    if (EP_FIELD_TO_VAL(EP_REG_AUDIO_STATUS_STD_ADO, reg_val)) {
        pkt = HDMI_AUDIO_PKT_STD;
        printlk(LK_DEBUG, "STD audio pkt detected\n");
    } else if (EP_FIELD_TO_VAL(EP_REG_AUDIO_STATUS_DSD_ADO, reg_val)) {
        pkt = HDMI_AUDIO_PKT_DSD;
        printlk(LK_DEBUG, "DSD audio pkt detected\n");
    } else if (EP_FIELD_TO_VAL(EP_REG_AUDIO_STATUS_HBR_ADO, reg_val)) {
        pkt = HDMI_AUDIO_PKT_HBR;
        printlk(LK_DEBUG, "HBR audio pkt detected\n");
    } else if (EP_FIELD_TO_VAL(EP_REG_AUDIO_STATUS_DST_ADO, reg_val)) {
        pkt = HDMI_AUDIO_PKT_DST;
		printlk(LK_DEBUG, "DST audio pkt detected\n");
    } else {
        pkt = HDMI_AUDIO_PKT_NONE;
        printlk(LK_DEBUG, "No audio pkt detected\n");
    }

    return pkt;
}
#endif /* HDMI_EP92A7E_FORCE_AUDIO_PKT_TYPE */

#ifdef HDMI_EP92A7E_FORCE_AUDIO_INFOFRAME
static status_t hdmi_ep92a7e_get_audio_infoframe_pkt(
                        struct ep92a7e_state *state,
                        hdmi_audio_infoframe_pkt_t *pkt)
{
    uint16_t slave_address = EP_SLAVE_ADDRESS;

#ifdef LK
    return i2c_read_reg_bytes(* (int *)state->i2c_bus, slave_address,
                              EP_REG_ADO_INFO_FRAME_1, pkt->raw,
                              sizeof(hdmi_audio_infoframe_pkt_t));
#else
    return raw_i2c_read_block((struct i2c_client *)state->i2c_bus, slave_address,
                               EP_REG_ADO_INFO_FRAME_1, pkt->raw,
                               sizeof(hdmi_audio_infoframe_pkt_t));
#endif
}
#endif /* HDMI_EP92A7E_FORCE_AUDIO_INFOFRAME */

#ifdef HDMI_EP92A7E_FORCE_AUDIO_CHANNEL_STATUS
static void ep92a7e_force_sample_rate_cs_to_768k(hdmi_channel_status_t *cs)
{
    cs->channel_status.cs.sampling_frequency = EP92A7E_SR768KHz & 0xF;
    cs->channel_status.cs.sampling_frequency_extended = (EP92A7E_SR768KHz >> 4) & 0xF ;
    cs->mask.sampling_frequency = 1;
}

static status_t hdmi_ep92a7e_get_channel_status(struct ep92a7e_state *state,
                                                hdmi_channel_status_t *cs)
{
    static hdmi_channel_status_mask_t ep92a7e_cs_validity_mask = {
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

    uint16_t slave_address = EP_SLAVE_ADDRESS;

    cs->mask = ep92a7e_cs_validity_mask;
#ifdef LK
    return i2c_read_reg_bytes(* (int *)state->i2c_bus, slave_address,
            EP_REG_CHANNEL_STATUS_0, cs->channel_status.raw, 5);
#else
    return raw_i2c_read_block((struct i2c_client *)state->i2c_bus, slave_address, EP_REG_CHANNEL_STATUS_0, cs->channel_status.raw, 5);
#endif

    if (hdmi_ep92a7e_get_pkt_type(state) == HDMI_AUDIO_PKT_HBR) {
        printlk(LK_DEBUG, "Invalid channel status but HBR detected on startup: Init sample rate to 768kHz\n");
        ep92a7e_force_sample_rate_cs_to_768k(cs);
        return 0;
    }

    cs->mask = (hdmi_channel_status_mask_t){0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    return ERR_BUSY;
}
#endif /* HDMI_EP92A7E_FORCE_AUDIO_CHANNEL_STATUS */

/* Check current source: hdmi, ARC or eARC */
void ep92a7e_sourcetype_config(struct ep92a7e_state *state)
{
    uint8_t source, gctrl0;
    ep_read_reg(state->i2c_bus, state->i2c_bus->addr,
                EP_REG_INFO_GENERAL_INFO_0, &source, 0);

    source &= (EP_REG_GENERAL_INFO_ARC_ON_MASK |
               EP_REG_GENERAL_INFO_EARC_ON_MASK);
    gctrl0 = (EP_REG_GENERAL_CONTROL_POWER_MASK |
              EP_REG_GENERAL_CONTROL_AUDIO_PATH_MASK);

    switch(source) {
    /* HDMI */
    case 0:
        ep_write_reg(state->i2c_bus,state->i2c_bus->addr,
                     EP_REG_GENERAL_CONTROL_0, gctrl0, 0);
        break;
    /* ARC */
    case EP_REG_GENERAL_INFO_ARC_ON_MASK:
        ep_write_reg(state->i2c_bus,state->i2c_bus->addr,
                     EP_REG_GENERAL_CONTROL_0,
                     (gctrl0 | EP_REG_GENERAL_CONTROL_ARC_EN_MASK), 0);
        break;
    /* eARC */
    case EP_REG_GENERAL_INFO_EARC_ON_MASK:
        ep_write_reg(state->i2c_bus,state->i2c_bus->addr,
                     EP_REG_GENERAL_CONTROL_0,
                     (gctrl0 | EP_REG_GENERAL_CONTROL_EARC_EN_MASK), 0);
        break;
    default:
        printlk(LK_ERR, "Invalid source type detected [%x]\n", source);
        return;
    }
}

void ep92a7e_reset(struct ep92a7e_state *state)
{
    printlk(LK_DEBUG,"Assert a reset on HDMI ep92a7e\n");
    /*  power off the chip */
    GPIO_SET_VALUE(&state->gpio_reset, 0);
    SLEEP_US(100);
    GPIO_SET_VALUE(&state->gpio_reset, 1);
    SLEEP_US(100000);
}

static int ep92a7e_power_up(struct ep92a7e_state *state)
{
    uint8_t slave_address = EP_SLAVE_ADDRESS;
    uint8_t reg_val;
    uint16_t vid = 0, pid = 0;
    uint8_t version[4];
    status_t ret;
    int retries = 15;

    printlk(LK_NOTICE,"Initiating HDMI ep92a7e up sequence...\n");
    do {
        ret = ep_read_regs(state->i2c_bus, slave_address,
                           EP_REG_INFO_VENDOR_ID_0, (uint8_t *) &vid, 2, 20);
        SLEEP_US(100000);
    } while (retries-- && ret);

    if ((retries == 0) && ret)
        goto error;

    ret =  ep_read_regs(state->i2c_bus, slave_address,
                        EP_REG_INFO_DEVICE_ID_0, (uint8_t *) &pid, 2, 20);
    if (ret)
        goto error;

    printlk(LK_NOTICE, "VID/PID: %x:%x\n", vid, pid);

    ret = ep_read_regs(state->i2c_bus, slave_address,
                       EP_REG_INFO_VERSION_0, (uint8_t *) &version, 4, 20);
    if (ret)
        goto error;

    printlk(LK_NOTICE, "Version: %x Date: %d/%d/%d\n",
            version[0], version[3], version[2], version[1]);

    reg_val = (EP_REG_GENERAL_CONTROL_POWER_MASK |
               EP_REG_GENERAL_CONTROL_AUDIO_PATH_MASK);
    ret = ep_write_reg(state->i2c_bus, slave_address,
                       EP_REG_GENERAL_CONTROL_0, reg_val, EP_I2C_XFER_WAIT);
    if (ret)
        goto error;

    return 0;

error:
    printlk(LK_ERR,"%s error\n", __func__);
    return -1;
}

int HDMI_I2C_Config(struct ep92a7e_state *state)
{
    int ret;
    printlk(LK_DEBUG, "\r\n\tDefault config for ep92a7e\r\n");

    /* Power on */
    ret = ep92a7e_power_up(state);
    if (ret)
        goto exit;

    /* Chenk current source: hdmi,ARC or eARC */
    /* FIXME: 1.It's ok for Samsung TV,but when testing it with sony TV,
     * bit0-"ARC_ON" of register 0x08 sometimes cannot update correctly as TV's setting,
     * so should contact ep to change fw to adapt different tv
     * or delete below config and change this bit manually.
     * 2.If connect eARC and hdmi input cable at the same time,
     * it's always eARC mode with this configuration
     */
    ep92a7e_sourcetype_config(state);

    printlk(LK_DEBUG, "\r\n\t HDMI configured..\r\n");

exit:
    return ret;
}

/************************
 **** HDMI Specific *****
 ************************/

/* HDMI open */
status_t _hdmi_open(struct ep92a7e_state *state)
{
    return 0;
}

/* HDMI close */
status_t _hdmi_close(struct ep92a7e_state *state)
{
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
status_t _hdmi_set_audio_format(struct ep92a7e_state *state, hdmi_audio_direction_t direction, hdmi_audio_fmt_t fmt)
{
    /* Support only Audio output port configuration */
    if (direction != HDMI_AUDIO_SOURCE) {
        printlk(LK_DEBUG, "Only audio source supported: Extract audio sample from HDMI\n");
        return -1;
    }

    switch(fmt) {
    case HDMI_AUDIO_FMT_60958:
        printlk(LK_DEBUG, "Configuring audio output format to 60958\n");
        printlk(LK_DEBUG, "IEC90958 is not supported as audio output format\n");
        break;

    case HDMI_AUDIO_FMT_61937:
        printlk(LK_DEBUG, "Configuring audio output format to 61937\n");
        break;

    case HDMI_AUDIO_FMT_CUSTOM:
        printlk(LK_DEBUG, "Configuring audio output format to custom format (AES3)\n");
        break;

    default:
        printlk(LK_DEBUG, "Invalid audio output format requested (%x)\n", fmt);
        return -1;
    }
    return 0;
}

/* Set audio packet */
status_t _hdmi_set_audio_packet(struct ep92a7e_state *state, hdmi_audio_direction_t direction, hdmi_audio_pkt_t pkt)
{
    return 0;
}

/* Set audio interface */
status_t _hdmi_set_audio_interface(struct ep92a7e_state *state,
                                    hdmi_audio_direction_t direction,
                                    hdmi_audio_if_t in, hdmi_audio_if_t out)
{
    uint8_t gctrl0;

    gctrl0 = (EP_REG_GENERAL_CONTROL_POWER_MASK |
              EP_REG_GENERAL_CONTROL_AUDIO_PATH_MASK);

    if ((direction != HDMI_AUDIO_SINK)
        && (direction != HDMI_AUDIO_SOURCE)) {
        return -1;
    }

    if (direction == HDMI_AUDIO_SOURCE) {
        /* HDMI_AUDIO_SOURCE */
        switch(in) {
        case HDMI_AUDIO_IF_NONE:
            printlk(LK_DEBUG, "Configuring audio output source interface to None\n");
            break;
        case HDMI_AUDIO_IF_HDMI:
            printlk(LK_DEBUG, "Configuring audio output source interface to HDMI\n");
            /* OUT_SOURCE_SEL : Audio from HDMI RX0 */
            ep_io_write_reg(state, EP_REG_GENERAL_CONTROL_0, gctrl0);
            break;
        case HDMI_AUDIO_IF_SPDIF:
            printlk(LK_DEBUG, "Configuring audio output source interface to SPDIF\n");
            printlk(LK_DEBUG, "SPDIF is not supported as audio output source\n");
            return -1;
        case HDMI_AUDIO_IF_ARC:
            printlk(LK_DEBUG, "Configuring audio output source interface to ARC\n");
            /* OUT_SOURCE_SEL : ARC Audio from HDMI Tx */
            ep_io_write_reg(state, EP_REG_GENERAL_CONTROL_0,
                            (gctrl0 | EP_REG_GENERAL_CONTROL_ARC_EN_MASK));
            break;
        case HDMI_AUDIO_IF_EARC:
            printlk(LK_DEBUG, "Configuring audio output source interface to I2S\n");
            /* OUT_SOURCE_SEL : Audio from Audio Input Port 1 */
            ep_io_write_reg(state, EP_REG_GENERAL_CONTROL_0,
                            (gctrl0 | EP_REG_GENERAL_CONTROL_EARC_EN_SHIFT));
            break;
        default:
            printlk(LK_DEBUG, "Invalid audio output source interface requested (%x)\n", in);
            return -1;
        }
        switch(out) {
        case HDMI_AUDIO_IF_NONE:
            printlk(LK_DEBUG, "Configuring audio output sink interface to None\n");
            break;
        case HDMI_AUDIO_IF_HDMI:
            printlk(LK_DEBUG, "HDMI is not supported as audio output sink\n");
            return -1;
            break;
        case HDMI_AUDIO_IF_SPDIF:
            printlk(LK_DEBUG, "SPDIF is not supported as audio output sink\n");
            return -1;
        case HDMI_AUDIO_IF_ARC:
            printlk(LK_DEBUG, "ARC is not supported as audio output sink\n");
            break;
        case HDMI_AUDIO_IF_EARC:
            printlk(LK_DEBUG, "EARC is not supported as audio output sink\n");
            break;
        default:
            printlk(LK_DEBUG, "Invalid audio output sink interface requested (%x)\n", out);
            return -1;
        }


    } else {
        /* HDMI_AUDIO_SINK */
        switch(in) {
        case HDMI_AUDIO_IF_NONE:
            printlk(LK_DEBUG, "Configuring audio input source interface to None\n");
            break;
        case HDMI_AUDIO_IF_HDMI:
            printlk(LK_DEBUG, "Configuring audio input source interface to HDMI\n");
            break;
        case HDMI_AUDIO_IF_SPDIF:
            printlk(LK_DEBUG, "SPDIF is not supported as audio input source\n");
            return -1;
            break;
        case HDMI_AUDIO_IF_ARC:
            printlk(LK_DEBUG, "ARC is not supported as audio input source\n");
            return -1;
            break;
        case HDMI_AUDIO_IF_I2S:
            printlk(LK_DEBUG, "Configuring audio input source interface to I2S\n");
            break;
        default:
            printlk(LK_DEBUG, "Invalid audio input source interface requested (%x)\n", in);
            return -1;
        }

        switch(out) {
        case HDMI_AUDIO_IF_NONE:
            printlk(LK_DEBUG, "Configuring audio input sink interface to None\n");
            break;
        case HDMI_AUDIO_IF_HDMI:
            printlk(LK_DEBUG, "Configuring audio input sink interface to HDMI\n");
            break;
        case HDMI_AUDIO_IF_SPDIF:
            printlk(LK_DEBUG, "SPDIF is not supported as audio input sink\n");
            return -1;
            break;
        case HDMI_AUDIO_IF_ARC:
            printlk(LK_DEBUG, "ARC is not supported as audio input sink\n");
            return -1;
            break;
        case HDMI_AUDIO_IF_I2S:
            printlk(LK_DEBUG, "I2S is not supported as audio input sink\n");
            return -1;
            break;
        default:
            printlk(LK_DEBUG, "Invalid audio input sink interface requested (%x)\n", out);
            return -1;
        }
    }
    return 0;
}

/* HDMI get channel status */
status_t _hdmi_get_channel_status(struct ep92a7e_state *state, hdmi_channel_status_t *channel_status)
{
    printlk(LK_DEBUG, "Get audio Channel status\n");

#ifdef HDMI_EP92A7E_FORCE_AUDIO_CHANNEL_STATUS
    return hdmi_ep92a7e_get_channel_status(state, channel_status);
#else
    *channel_status = state->current_cs; /* struct copy */
    return 0;
#endif
}

/* HDMI get audio infoframe pkt */
status_t _hdmi_get_audio_infoframe_pkt(struct ep92a7e_state *state,
                                                hdmi_audio_infoframe_pkt_t *pkt)
{
    printlk(LK_DEBUG, "Get audio infoframe packet\n");

#ifdef HDMI_EP92A7E_FORCE_AUDIO_INFOFRAME
    return hdmi_ep92a7e_get_audio_infoframe_pkt(state, pkt);
#else
    *pkt = state->current_audio_info_pkt; /* struct copy */
    return 0;
#endif
}

/* HDMI get audio custom fmt layout */
status_t _hdmi_get_audio_custom_fmt_layout(struct ep92a7e_state *state, iec60958_custom_fmt_layout_t *layout)
{
    static iec60958_custom_fmt_layout_t ep92a7e_custom_fmt_layout = {
        .data_start = 0,
        .data_length = 24,
        .validity_bit = 24,
        .user_bit = 25,
        .channel_status_bit = 26,
        .block_start_bit = 27,
        .parity_bit = -1,
        .width = 32,
    };

    printlk(LK_DEBUG, "Get audio custom format layout\n");

    *layout = ep92a7e_custom_fmt_layout;
    return 0;
}

/* HDMI get audio pkt type */
status_t _hdmi_get_audio_pkt_type(struct ep92a7e_state *state, hdmi_audio_pkt_t *pkt)
{
    printlk(LK_DEBUG, "Get audio packet type\n");
#ifdef HDMI_EP92A7E_FORCE_AUDIO_PKT_TYPE
    *pkt = hdmi_ep92a7e_get_pkt_type(state);
#else
    *pkt = state->current_pkt_type;
#endif
    return 0;
}

/* HDMI get audio pkt layout */
status_t _hdmi_get_audio_pkt_layout(struct ep92a7e_state *state, hdmi_audio_pkt_layout_t *layout)
{
    printlk(LK_DEBUG, "Get audio packet layout\n");
#ifdef HDMI_EP92A7E_FORCE_AUDIO_PKT_LAYOUT
    *layout = hdmi_ep92a7e_get_pkt_layout(state);
#else
    *layout = state->current_pkt_layout;
#endif
    return 0;
}
