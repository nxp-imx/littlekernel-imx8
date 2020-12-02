/*
 * The Clear BSD License
 * Copyright 2019-2021 NXP
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

#include "ak5558.h"
#include "debug.h"

static inline status_t ak_read_reg(int bus, uint8_t address, uint8_t reg,
                                   uint8_t *val, unsigned wait_us)
{
    status_t ret;

    ret = i2c_read_reg(bus, address, reg, val);

    printlk(LK_NOTICE, "ak_read_reg bus=%d, address=0x%02x, reg=0x%x, val=0x%x\n",
                  bus, address, reg, *val);

    if (ret)
        return ret;

    udelay(wait_us);

    return 0;
}

status_t adc_ak5558_read_reg(struct ak5558_state *state, uint8_t reg,
                                    uint8_t *val)
{
    return ak_read_reg(state->bus_id, ADC_AK5558_ADC_I2C_ADDR_7BIT, reg, val, 0);
}

static inline status_t ak_write_reg(int bus, uint8_t address, uint8_t reg,
                                    uint8_t val, unsigned wait_us)
{
    status_t ret;

    printlk(LK_NOTICE,
                  "i2c_write_reg bus=%d, address=0x%02x, reg=0x%x, val=0x%x\n",
                  bus, address, reg, val);
    ret = i2c_write_reg(bus, address, reg, val);

    if (ret)
        return ret;

    udelay(wait_us);

    return 0;
}

status_t adc_ak5558_write_reg(struct ak5558_state *state,
                                     uint8_t reg, uint8_t val)
{
    return ak_write_reg(state->bus_id, ADC_AK5558_ADC_I2C_ADDR_7BIT, reg, val,
                        0);
}

static status_t adc_ak5558_dump_registers(struct device *dev)
{
    struct ak5558_state *ak5558_state = dev->state;
    return _adc_ak5558_dump_registers(ak5558_state);
}

static status_t adc_ak5558_init(struct device *dev)
{
    uint32_t bus_id_i2c;
    int ret;

    struct ak5558_state *ak5558_state = malloc(
            sizeof(struct ak5558_state));

    ASSERT(ak5558_state);
    memset(ak5558_state, 0, sizeof(struct ak5558_state));

    ret = of_device_get_int32(dev, "bus-id-i2c", &bus_id_i2c);
    if (ret) {
        printlk(LK_INFO, "Bus-id-i2c fdt node property can\t' be read, aborting!\n");
        free(ak5558_state);
        return ERR_NOT_FOUND;
    }

    printlk(LK_INFO, "Probing adc_ak5558 using i2c bus id: [%d]\n", bus_id_i2c);

    struct device *i2c_device = class_i2c_get_device_by_id(bus_id_i2c);
    ASSERT(i2c_device != NULL);

    ak5558_state->i2c_dev = i2c_device;
    ak5558_state->bus_id = bus_id_i2c;
    ak5558_state->device = dev;

    dev->state = ak5558_state;

    ret = gpio_request_by_name(dev, "pdn-gpio", &ak5558_state->pdn_gpio,
    GPIO_DESC_OUTPUT | GPIO_DESC_OUTPUT_ACTIVE);
    if (ret) {
        printlk(LK_INFO, "Could not request pdn-gpio for adc, aborting!\n");
        free(ak5558_state);
        return ERR_NOT_FOUND;
    }

    /* ADC initial reset */
    adc_ak5558_reset(ak5558_state);

    /* Basic sanity check on reset values */
    do {
        uint8_t reg;
        ret = ERR_NOT_READY;

        adc_ak5558_read_reg(dev->state, AK5558_00_POWER_MANAGEMENT1, &reg);
        if (reg != 0xff)
            break;

        adc_ak5558_read_reg(dev->state, AK5558_01_POWER_MANAGEMENT2, &reg);
        if (reg != 0x01)
            break;

        ret = NO_ERROR;
    } while (0);

    return (status_t) ret;
}

status_t adc_ak5558_open(struct device *dev)
{
    struct ak5558_state *ak5558_state = dev->state;
    return _adc_ak5558_open(ak5558_state);
}

status_t adc_ak5558_close(struct device *dev)
{
    struct ak5558_state *ak5558_state = dev->state;
    return _adc_ak5558_close(ak5558_state);
}

status_t adc_ak5558_get_capabilities(const struct device *dev,
                                     uint32_t *capabilities)
{
    *capabilities = _adc_ak5558_get_capabilities();
    return 0;
}

status_t adc_ak5558_set_format(struct device *dev,
                               adc_audio_hw_params_t *hw_params)
{
    struct ak5558_state *ak5558_state = dev->state;
    ak5558_state->hw_params = *hw_params;

    return _adc_ak5558_set_format(ak5558_state);
}

status_t adc_ak5558_reset_power(struct device *dev)
{
    struct ak5558_state *ak5558_state = dev->state;
    uint8_t reg_val;

    /* Power down all channels */
    reg_val &= (uint8_t)~AK5558_PWRMGMT1_PW_MASK;

    /* Write-back POWER MANAGEMENT 1 register */
    if (adc_ak5558_write_reg(ak5558_state, AK5558_00_POWER_MANAGEMENT1, reg_val))
        return ERR_I2C_NACK;

    /* Power up all channels */
    reg_val |= AK5558_PWRMGMT1_PW_MASK;

    /* Write-back POWER MANAGEMENT 1 register */
    if (adc_ak5558_write_reg(ak5558_state, AK5558_00_POWER_MANAGEMENT1, reg_val))
        return ERR_I2C_NACK;

    return 0;
}

static struct device_class adc_ak5558_device_class = {
    .name = "adc",
};

static struct adc_ops the_ops = {
        .std = {
            .device_class = &adc_ak5558_device_class,
            .init = adc_ak5558_init,
        },
        .open               = adc_ak5558_open,
        .close              = adc_ak5558_close,
        .get_capabilities   = adc_ak5558_get_capabilities,
        .set_format         = adc_ak5558_set_format,
        .reset_power        = adc_ak5558_reset_power,
};

DRIVER_EXPORT_WITH_LVL(adc_ak5558, &the_ops.std, DRIVER_INIT_HAL);
