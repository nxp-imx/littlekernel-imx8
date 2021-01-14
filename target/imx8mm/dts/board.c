/*
 * Copyright 2019-2020 NXP
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
#include <lib/appargs.h>
#include <dev/class/gpio.h>
#include <kernel/thread.h>
#include <assert.h>

static void nxp_audioboard_rev1(const void *fdt, int node)
{
    struct gpio_desc desc;
    status_t ret = of_gpio_request_by_name(node, "enable-gpio", &desc,
                                    GPIO_DESC_OUTPUT);
    ASSERT(ret == 0);

    /* Power cycle the audio board */
    gpio_desc_set_value(&desc, 0);
    thread_sleep(1000);
    gpio_desc_set_value(&desc, 1);
    thread_sleep(1000);

}

BOARD_EXPORT(nxp_audioboard_rev1, "nxp,audioboard,rev1", nxp_audioboard_rev1);

static void nxp_audioboard_rev2(const void *fdt, int node)
{
    printlk(LK_NOTICE, "i.MX8MM AB2 Nothing to do...\n");
}

BOARD_EXPORT(nxp_audioboard_rev2, "nxp,audioboard,rev2", nxp_audioboard_rev2);

static void nxp_generic_target(const void *fdt, int node)
{
    /* nothing to do ... */
}
BOARD_EXPORT(nxp_generic_target, "nxp,generic,rev1", nxp_generic_target);
