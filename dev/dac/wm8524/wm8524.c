/*
 * The Clear BSD License
 * Copyright 2020 NXP
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

#include <platform/interrupts.h>
#include <delay.h>
#define delay_us udelay
#include <malloc.h>
#include <string.h>
#include "trace.h"

#define LOCAL_TRACE 0
#include <dev/class/gpio.h>
#include <dev/class/dac.h>

#define DRIVER_NAME "dac_wm8524"

struct dac_wm8524_state {
    struct device *device;
    struct gpio_desc mute_gpio;
};


static void dac_wm8524_mute(struct dac_wm8524_state *state, bool mute)
{
    gpio_desc_set_value(&state->mute_gpio, !mute);
}

#include <lib/appargs.h>
static status_t dac_wm8524_init(struct device *dev)
{
    int ret;
    struct dac_wm8524_state *wm8524_state = malloc(sizeof(struct dac_wm8524_state));

    ASSERT(wm8524_state);
    memset(wm8524_state, 0, sizeof(struct dac_wm8524_state));

    wm8524_state->device = dev;

    dev->state = wm8524_state;

    ret = gpio_request_by_name(dev, "mute-gpio",
                        &wm8524_state->mute_gpio,
                        GPIO_DESC_OUTPUT | GPIO_DESC_OUTPUT_ACTIVE);
    if (ret) {
        printf("Failed to request mute gpio for wm8524: %d\n", ret);
        return ret;
    }

    dac_wm8524_mute(wm8524_state, true);

    return 0;
}

status_t dac_wm8524_open(struct device *dev)
{
    struct dac_wm8524_state *wm8524_state = dev->state;
    dac_wm8524_mute(wm8524_state, false);
    return 0;
}

status_t dac_wm8524_close(struct device *dev)
{
    struct dac_wm8524_state *wm8524_state = dev->state;
    dac_wm8524_mute(wm8524_state, true);
    return 0;
}

status_t dac_wm8524_get_capabilities(const struct device * dev,
                                                        uint32_t *capabilities)
{
    *capabilities = DAC_CAP_PKT_PCM;
    return 0;
}

static struct device_class dac_wm8524_device_class = {
    .name = "dac",
};
static struct dac_ops the_ops = {
        .std = {
            .device_class = &dac_wm8524_device_class,
            .init = dac_wm8524_init,
        },
        .open               = dac_wm8524_open,
        .close              = dac_wm8524_close,
        .get_capabilities   = dac_wm8524_get_capabilities,
};

DRIVER_EXPORT_WITH_LVL(dac_wm8524, &the_ops.std, DRIVER_INIT_HAL);
