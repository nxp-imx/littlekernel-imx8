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

int adv7627_config_irq_pins(struct adv7627_state *state)
{
#ifdef HDMI_ADV7627_REGISTER_INT1
    pca6416_register_int(g_pca6416_device, 0x8, adv7627_irq_handler, state);
#endif
#ifdef HDMI_ADV7627_REGISTER_INT2
    pca6416_register_int(g_pca6416_device, 0x9, adv7627_irq_handler, state);
#endif
    return 0;
}

static void adv7627_enable_irq_pins(struct adv7627_state *state, uint8_t enable)
{
    uint16_t slave_address;
    uint8_t reg_val;

    slave_address = 0x58;

    if (!enable) {
        /* Disabling Interrupt Output Pins */
        // int1 is always on
        // int2_en, IO Map, Address 0x41[2]
        adv_read_reg(state->i2c_bus, slave_address, 0x41, &reg_val, 0);
        reg_val &= ~(1 << 2);
        adv_write_reg(state->i2c_bus, slave_address, 0x41, reg_val, ADV_I2C_XFER_WAIT);
        return;
    }

    /* Set Interrupt Drive Level to Open drain */
    //intrq_op_sel[1:0], IO Map, Address 0x40[1:0]
    adv_read_reg(state->i2c_bus, slave_address, 0x40, &reg_val, 0);
    reg_val &= ~(3 << 0);
    adv_write_reg(state->i2c_bus, slave_address, 0x40, reg_val, ADV_I2C_XFER_WAIT);

    //intrq2_op_sel[1:0], IO Map, Address 0x41[1:0]
    adv_read_reg(state->i2c_bus, slave_address, 0x41, &reg_val, 0);
    reg_val &= ~(3 << 0);
    adv_write_reg(state->i2c_bus, slave_address, 0x41, reg_val, ADV_I2C_XFER_WAIT);

    /* Set Interrupt Duration to 4 Xtal periods */
    //intrq_dur_sel[1:0], IO Map, Address 0x40[7:6]
    adv_read_reg(state->i2c_bus, slave_address, 0x40, &reg_val, 0);
    reg_val &= ~(3 << 6);
    adv_write_reg(state->i2c_bus, slave_address, 0x40, reg_val, ADV_I2C_XFER_WAIT);

    //intrq2_dur_sel[1:0], IO Map, Address 0x41[7:6]
    adv_read_reg(state->i2c_bus, slave_address, 0x41, &reg_val, 0);
    reg_val &= ~(3 << 6);
    adv_write_reg(state->i2c_bus, slave_address, 0x41, reg_val, ADV_I2C_XFER_WAIT);

    /* Enabling Interrupt Output Pins */
    // int1 is always on
    // int2_en, IO Map, Address 0x41[2]
    adv_read_reg(state->i2c_bus, slave_address, 0x41, &reg_val, 0);
    reg_val &= ~(1 << 2);
    reg_val |= (1 << 2);
    adv_write_reg(state->i2c_bus, slave_address, 0x41, reg_val, ADV_I2C_XFER_WAIT);
}


#include <dev/class/i2c.h>
#include <malloc.h>
static status_t adv7627_hdmi_init(struct device *dev)
{
    uint32_t i2c_bus_id;
    int ret;
    const struct device_config_data *config = dev->config;

    ret = of_device_get_int32(dev, "bus-id-i2c", &i2c_bus_id);
    if (ret) {
        printlk(LK_INFO, "Bus-id-i2c fdt node property can\t' be read, aborting!\n");
        return ERR_NOT_FOUND;
    }

    printlk(LK_INFO, "Probing adv7627 using i2c bus id: [%d]\n", i2c_bus_id);

    devcfg_set_pins_by_name(config->pins_cfg, config->pins_cfg_cnt, "default");

    struct device *i2c_device;

    i2c_device = class_i2c_get_device_by_id(i2c_bus_id);
    ASSERT(i2c_device != NULL);

    struct adv7627_state *adv_state = malloc(sizeof(struct adv7627_state));

    memset(adv_state, 0, sizeof(struct adv7627_state));

    spin_lock_init(&adv_state->cb_lock);

    adv_state->i2c_dev = i2c_device;
    adv_state->i2c_bus_id = i2c_bus_id;
    adv_state->i2c_bus = (i2c_bus *)&adv_state->i2c_bus_id;

    dev->state = adv_state;
    adv_state->dev = dev;

    ret = gpio_request_by_name(dev, "reset-gpio",
                        (struct gpio_desc *) &adv_state->gpio_reset,
                        GPIO_DESC_OUTPUT | GPIO_DESC_OUTPUT_ACTIVE);
    ASSERT(ret == 0);

    ret = gpio_request_by_name(dev, "cs-gpio",
                        (struct gpio_desc *) &adv_state->gpio_cs,
                        GPIO_DESC_OUTPUT | GPIO_DESC_OUTPUT_ACTIVE);
    ASSERT(ret == 0);

#ifdef HDMI_ADV7627_REGISTER_INT1
    ret = gpio_request_by_name(dev, "int1-gpio",
                        (struct gpio_desc *) &adv_state->gpio_int1,
                        GPIO_DESC_INPUT);
    ASSERT(ret == 0);
#endif

#ifdef HDMI_ADV7627_REGISTER_INT2
    ret = gpio_request_by_name(dev, "int2-gpio",
                        (struct gpio_desc *) &adv_state->gpio_int2,
                        GPIO_DESC_INPUT);
    ASSERT(ret == 0);
#endif
    /* deprecate me */
    adv_state->current_samplerate = 0;
    /* end of deprecate me */

    ret = HDMI_I2C_Config(adv_state);
    if (ret != 0) {
        printf("Error : Failed to configure adv7627\r\n");
        return ret;
    }

    adv7627_read_and_update_mclk(adv_state);

    /* Configure and Enable ADV7627 Interrupts */
    adv7627_enable_irq_pins(adv_state, 1);

    return 0;
}

static status_t adv7627_hdmi_open(struct device *dev)
{
    struct adv7627_state *adv_state = dev->state;
    return _hdmi_open(adv_state);
}

static status_t adv7627_hdmi_close(struct device *dev)
{
    struct adv7627_state *adv_state = dev->state;
    return _hdmi_close(adv_state);
}

static status_t adv7627_hdmi_get_capabilities(const struct device * dev, uint32_t *capabilities)
{
    return _hdmi_get_capabilities(capabilities);
}

static status_t adv7627_hdmi_set_callback(struct device *dev, hdmi_cb_t cb, void *cookie)
{
    struct adv7627_state *state = dev->state;
    status_t ret = 0;

    printlk(LK_INFO, "Set HDMI callback/cookie: [%p/%p]\n", cb, cookie);
    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->cb_lock, lock_state);

    state->cb = cb;
    state->cb_cookie = cookie;

    spin_unlock_irqrestore(&state->cb_lock, lock_state);

    return ret;
}

static status_t adv7627_hdmi_set_audio_format(struct device *dev, hdmi_audio_direction_t direction, hdmi_audio_fmt_t fmt)
{
    struct adv7627_state *state = dev->state;
    return _hdmi_set_audio_format(state, direction, fmt);
}

static status_t adv7627_hdmi_set_audio_packet(struct device *dev, hdmi_audio_direction_t direction, hdmi_audio_pkt_t pkt)
{
    return _hdmi_set_audio_packet(dev->state, direction, pkt);
}

static status_t adv7627_hdmi_set_audio_interface(struct device *dev,
                                    hdmi_audio_direction_t direction,
                                    hdmi_audio_if_t in, hdmi_audio_if_t out)
{
    struct adv7627_state *state = dev->state;
    return _hdmi_set_audio_interface(state, direction, in, out);
}

static status_t adv7627_hdmi_get_channel_status(struct device *dev, hdmi_channel_status_t *channel_status)
{
    struct adv7627_state *state = dev->state;
    return _hdmi_get_channel_status(state, channel_status);
}

static status_t adv7627_hdmi_get_audio_infoframe_pkt(struct device *dev,
                                                hdmi_audio_infoframe_pkt_t *pkt)
{
    struct adv7627_state *state = dev->state;
    return _hdmi_get_audio_infoframe_pkt(state, pkt);
}

static status_t adv7627_hdmi_get_audio_custom_fmt_layout(
                            struct device *dev,
                            iec60958_custom_fmt_layout_t *layout)
{
    return _hdmi_get_audio_custom_fmt_layout(dev->state, layout);
}

static status_t adv7627_hdmi_get_audio_pkt_type(struct device *dev, hdmi_audio_pkt_t *pkt)
{
    struct adv7627_state *state = dev->state;
    _hdmi_get_audio_pkt_type(state, pkt);
    return 0;
}

static status_t adv7627_hdmi_get_audio_pkt_layout(struct device *dev, hdmi_audio_pkt_layout_t *layout)
{
    struct adv7627_state *state = dev->state;
    _hdmi_get_audio_pkt_layout(state, layout);
    return 0;
}

static struct device_class adv7627_hdmi_device_class = {
    .name = "hdmi",
};
static struct hdmi_ops the_ops = {
    .std = {
        .device_class = &adv7627_hdmi_device_class,
        .init = adv7627_hdmi_init,
    },
    .open = adv7627_hdmi_open,
    .close = adv7627_hdmi_close,
    .get_capabilities = adv7627_hdmi_get_capabilities,
    .set_callback = adv7627_hdmi_set_callback,
    .set_audio_format = adv7627_hdmi_set_audio_format,
    .set_audio_packet = adv7627_hdmi_set_audio_packet,
    .set_audio_interface = adv7627_hdmi_set_audio_interface,
    .get_channel_status = adv7627_hdmi_get_channel_status,
    .get_audio_infoframe_pkt = adv7627_hdmi_get_audio_infoframe_pkt,
    .get_audio_custom_fmt_layout = adv7627_hdmi_get_audio_custom_fmt_layout,
    .get_audio_pkt_type = adv7627_hdmi_get_audio_pkt_type,
    .get_audio_pkt_layout = adv7627_hdmi_get_audio_pkt_layout,
};

DRIVER_EXPORT_WITH_LVL(hdmi_adv7627, &the_ops.std, DRIVER_INIT_HAL);
