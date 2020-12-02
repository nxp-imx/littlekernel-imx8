/*
 * The Clear BSD License
 * Copyright 2017-2020 NXP
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

#include "ak4458.h"
#include "debug.h"

static inline status_t ak_write_reg(int bus, uint8_t address, uint8_t reg, uint8_t val, unsigned wait_us)
{
    status_t ret;

    printlk(LK_NOTICE, "i2c_write_reg bus=%d, address=0x%02x, reg=0x%x, val=0x%x\n", bus, address, reg, val);
    ret = i2c_write_reg(bus, address, reg, val);

    if (ret)
        return ret;

    udelay(wait_us);

    return 0;
}

status_t dac_ak4458_write_reg(struct dac_ak4458_state *state, uint8_t reg, uint8_t val)
{
    status_t ret;

    ret = ak_write_reg(state->bus_id, DAC_AK4458_1_DAC_I2C_ADDR_7BIT,
                                                                reg, val, 0);
    if (ret)
        return ret;

    ret = ak_write_reg(state->bus_id, DAC_AK4458_2_DAC_I2C_ADDR_7BIT,
                                                                reg, val, 0);

    if (ret)
        return ret;

    ret = ak_write_reg(state->bus_id, DAC_AK4458_3_DAC_I2C_ADDR_7BIT,
                                                                reg, val, 0);

    return ret;

}

static status_t dac_ak4458_init(struct device *dev)
{
    uint32_t bus_id_i2c;
    int ret;

    struct dac_ak4458_state *ak4458_state = malloc(sizeof(struct dac_ak4458_state));

    ASSERT(ak4458_state);
    memset(ak4458_state, 0, sizeof(struct dac_ak4458_state));

    ret = of_device_get_int32(dev, "bus-id-i2c", &bus_id_i2c);
    if (ret) {
        printlk(LK_INFO, "Bus-id-i2c fdt node property can\t' be read, aborting!\n");
        free(ak4458_state);
        return ERR_NOT_FOUND;
    }

    printlk(LK_INFO, "Probing dac_ak4458 using i2c bus id: [%d]\n", bus_id_i2c);

    struct device * i2c_device = class_i2c_get_device_by_id(bus_id_i2c);
    ASSERT(i2c_device != NULL);

    ak4458_state->i2c_dev = i2c_device;
    ak4458_state->bus_id = bus_id_i2c;
    ak4458_state->device = dev;

    dev->state = ak4458_state;

    ret = gpio_request_by_name(dev, "pdn-gpio",
                        &ak4458_state->pdn_gpio,
                        GPIO_DESC_OUTPUT | GPIO_DESC_OUTPUT_ACTIVE);

    dac_ak4458_reset(ak4458_state);

    return 0;
}

status_t dac_ak4458_open(struct device *dev)
{
    struct dac_ak4458_state *ak4458_state = dev->state;
    return _dac_ak4458_open(ak4458_state);
}

status_t dac_ak4458_close(struct device *dev)
{
    struct dac_ak4458_state *ak4458_state = dev->state;
    return _dac_ak4458_close(ak4458_state);
}

status_t dac_ak4458_get_capabilities(const struct device * dev,
                                                        uint32_t *capabilities)
{
    _dac_get_capabilities(capabilities);
    return 0;
}

status_t dac_ak4458_set_format(struct device *dev,
                                            dac_audio_hw_params_t *hw_params)
{
    struct dac_ak4458_state *ak4458_state = dev->state;

    return _dac_ak4458_set_format(ak4458_state, hw_params);
}

static struct device_class dac_ak4458_device_class = {
    .name = "dac",
};
static struct dac_ops the_ops = {
        .std = {
            .device_class = &dac_ak4458_device_class,
            .init = dac_ak4458_init,
        },
        .open               = dac_ak4458_open,
        .close              = dac_ak4458_close,
        .get_capabilities   = dac_ak4458_get_capabilities,
        .set_format         = dac_ak4458_set_format,
};

DRIVER_EXPORT_WITH_LVL(dac_ak4458, &the_ops.std, DRIVER_INIT_HAL);
