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
#include <stdlib.h>
#include <dev/driver.h>
#include <assert.h>
#include <err.h>
#include <dev/class/gpio.h>
#include <lib/appargs.h>


static struct device_class dummy_device_class = {
    .name = "dummy",
};

static status_t dummy_init(struct device *dev)
{
    printlk(LK_NOTICE, "%s: entry\n", __PRETTY_FUNCTION__);

    struct gpio_desc desc;

    gpio_request_by_name(dev, "enable-gpio", &desc, 0);
    return 0;
}

static struct dummy_ops {
    struct driver_ops std;
} the_ops = {
    .std = {
        .device_class = &dummy_device_class,
        .init = dummy_init,
    },
};

DRIVER_EXPORT_WITH_LVL(dummy, &the_ops.std, DRIVER_INIT_PLATFORM);


struct dummy_gpio_state {
    unsigned foo;
};

static int dummy_gpio_request(struct device *dev,
                                    unsigned offset, const char *label)
{
    printlk(LK_NOTICE, "%s: entry\n", __PRETTY_FUNCTION__);
    return 0;
}

static int dummy_gpio_free(struct device *dev, unsigned offset)
{
    printlk(LK_NOTICE, "%s: entry\n", __PRETTY_FUNCTION__);
    return 0;
}
static int dummy_direction_input(struct device *dev, unsigned offset)
{
    printlk(LK_NOTICE, "%s: entry\n", __PRETTY_FUNCTION__);
    return 0;
}

static int dummy_direction_output(struct device *dev, unsigned offset, int value)
{
    printlk(LK_NOTICE, "%s: entry\n", __PRETTY_FUNCTION__);
    return 0;
}

static int dummy_get_value(struct device *dev, unsigned offset)
{
    printlk(LK_NOTICE, "%s: entry\n", __PRETTY_FUNCTION__);
    return 0;
}

static int dummy_set_value(struct device *dev, unsigned offset, int value)
{
    printlk(LK_NOTICE, "%s: entry\n", __PRETTY_FUNCTION__);
    return 0;
}

static int dummy_get_open_drain(struct device *dev, unsigned offset)
{
    printlk(LK_NOTICE, "%s: entry\n", __PRETTY_FUNCTION__);
    return 0;
}

static int dummy_set_open_drain(struct device *dev, unsigned offset, int value)
{
    printlk(LK_NOTICE, "%s: entry\n", __PRETTY_FUNCTION__);
    return 0;
}

static status_t dummy_gpio_init(struct device *dev)
{
    printlk(LK_NOTICE, "%s: entry\n", __PRETTY_FUNCTION__);
    uint32_t ngpios = 32;
    int ret = of_device_get_int32(dev, "ngpios", &ngpios);
    struct gpio_controller *ctrl = gpio_controller_add(
                                dev, ngpios, sizeof(struct dummy_gpio_state));

    printlk(LK_NOTICE, "%s:%d: Gpio %s: %d pins, base %d\n",
            __PRETTY_FUNCTION__, __LINE__, dev->name, ctrl->count, ctrl->base);

    dev->state = ctrl;
    return 0;
}

static struct device_class gpio_device_class = {
    .name = "gpio",
};

struct gpio_ops dummy_gpio_ops = {
    .std = {
        .device_class = &gpio_device_class,
        .init = dummy_gpio_init,
    },
    .request = dummy_gpio_request,
    .free = dummy_gpio_free,
    .direction_input = dummy_direction_input,
    .direction_output = dummy_direction_output,
    .get_value = dummy_get_value,
    .set_value = dummy_set_value,
    .get_open_drain = dummy_get_open_drain,
    .set_open_drain = dummy_set_open_drain,
};

DRIVER_EXPORT_WITH_CFG_LVL(gpio, &dummy_gpio_ops.std, DRIVER_INIT_CORE, sizeof(struct dummy_gpio_state));
