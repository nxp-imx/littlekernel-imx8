/*
 * The Clear BSD License
 * Copyright 2019-2020 NXP
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
#include <sys/types.h>

#include <lib/appargs.h>
#include <dev/driver.h>
#include <assert.h>
#include <err.h>

#include <dev/gpr_ioctl.h>

#define GPR_MAX_INIT_TUPLES (32)

//#define __DRY_RUN

static void imx_gpr_write_reg(struct device *dev, uint32_t offset, uint32_t value)
{
    struct device_cfg_reg *reg = dev->state;

    ASSERT(reg);

    volatile uint32_t *addr = (volatile uint32_t *) reg->vbase;
    addr += offset;
    printlk(LK_NOTICE, "%s:%d: Set register %p to %x\n", __PRETTY_FUNCTION__,
            __LINE__, addr, value);
#ifndef __DRY_RUN
    *addr = value;
#endif
}

static uint32_t imx_gpr_read_reg(struct device *dev, uint32_t offset)
{
    struct device_cfg_reg *reg = dev->state;

    ASSERT(reg);

    volatile uint32_t *addr = (volatile uint32_t *) reg->vbase;
    addr += offset;
    printlk(LK_NOTICE, "%s:%d: Read register %p : %x\n", __PRETTY_FUNCTION__,
            __LINE__, addr, *addr);
    return *addr;
}

status_t imx_gpr_init(struct device *dev)
{
    printlk(LK_NOTICE, "%s: entry\n", __PRETTY_FUNCTION__);
    const struct device_config_data *config = dev->config;

    union _gpr_init_value{
        unsigned raw[2];
        struct {
            unsigned offset;
            unsigned value;
        } s;
    } init_values[GPR_MAX_INIT_TUPLES];

    struct device_cfg_reg *reg =
                        device_config_get_register_by_name(config, "core");
    ASSERT(reg);
    dev->state = reg;

    int count = of_device_get_int32_array(dev, "init", (uint32_t *) init_values,
                                    ARRAY_SIZE(init_values)/sizeof(uint32_t));

    if (count % 2) {
        printlk(LK_NOTICE, "%s:%d: init values length (%d) is not even. skipping.\n",
                __PRETTY_FUNCTION__, __LINE__, count);
    } else {
        if (count > 0) {
            int i;
            printlk(LK_NOTICE, "%s:%d: %d GPR register to initialize..\n",
                    __PRETTY_FUNCTION__, __LINE__, count / 2);
            for ( i = 0; i < (count / 2); i++) {
                volatile uint32_t *addr = (volatile uint32_t *) reg->vbase;
                addr += init_values[i].s.offset;
                unsigned value = init_values[i].s.value;
                printlk(LK_NOTICE, "%s:%d: Set register %p to %x\n",
                        __PRETTY_FUNCTION__, __LINE__, addr, value);
                *addr = value;
            }
        }
    }
    return 0;
}

static status_t imx_gpr_ioctl(struct device *dev, int request, void *argp)
{
    status_t ret = NO_ERROR;

    if (!dev)
        return ERR_NOT_FOUND;

    if ((IOCGROUP(request) != IOCTL_DEV_GPR) || (!argp))
        return ERR_INVALID_ARGS;

    int cmd = IOCBASECMD(request);
    struct gpr_ioc_cmd_reg *gpr_reg;

    switch (request) {
    case GPR_IOC_WRITE:
        gpr_reg = (struct gpr_ioc_cmd_reg *)argp;
        imx_gpr_write_reg(dev, gpr_reg->offset, gpr_reg->value);
        break;

    case GPR_IOC_READ:
        gpr_reg = (struct gpr_ioc_cmd_reg *)argp;
        gpr_reg->value = imx_gpr_read_reg(dev, gpr_reg->offset);
        break;

    default:
        printlk(LK_ERR, "%s:%d: Invalid gpr ioctl(%d)\n", __PRETTY_FUNCTION__,
                __LINE__, cmd);
        ret = ERR_NOT_FOUND;
    }

    return ret;
}

static struct driver_ops the_ops = {
    .device_class = NULL,
    .init = imx_gpr_init,
    .ioctl = imx_gpr_ioctl,
};
DRIVER_EXPORT_WITH_LVL(gpr, &the_ops, DRIVER_INIT_CORE);

