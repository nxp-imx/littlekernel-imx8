/*
 * The Clear BSD License
 * Copyright 2018-2020 NXP
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
#include <dev/driver.h>
#include <lib/appargs.h>
#include <libfdt.h>
#include <dev/class/gpio.h>
#include "assert.h"
#include "err.h"


struct device_cfg_latencies {
    unsigned cfg_cnt;
    unsigned *events;
    struct gpio_desc *gpios;
    const char **name;
};

struct device_cfg_latencies * of_device_get_latency_cfg(struct device *dev)
{
    int lenp, i, str_len, names_count;
    const char *strings;
    struct device_cfg_latencies *cfgs;

    const void *fdt = main_fdt_base;
    if (!fdt)
        return NULL;

    int offset = dev->node_offset;

    strings = fdt_getprop(fdt, offset, "event-names", &str_len);
    if (!strings)
        return NULL;

    names_count = fdt_stringlist_count(fdt, offset, "event-names");
    if (names_count == 0)
        return NULL;

    const fdt32_t *cell = fdt_getprop(fdt, offset, "event-ids", &lenp);
    if (cell == NULL)
        return NULL;


    cfgs = malloc(sizeof(struct device_cfg_latencies));
    ASSERT(cfgs);

    cfgs->name = malloc(sizeof(char *) * names_count);
    ASSERT(cfgs->name);
    cfgs->events = malloc(lenp);
    ASSERT(cfgs->events);
    cfgs->gpios = malloc(sizeof(struct gpio_desc) * names_count);
    ASSERT(cfgs->gpios);
    cfgs->cfg_cnt = names_count;

    fdtdec_get_string_list(fdt, offset, "event-names", cfgs->name, names_count);

    fdt_get_int_array(fdt, offset, "event-ids",
                        (uint32_t*) cfgs->events, lenp / sizeof(uint32_t));


    for (i = 0; i < names_count; i++) {
        char gpio_name[32];
        status_t ret;
        sprintf(gpio_name, "%s-gpio", cfgs->name[i]);
        ret = gpio_request_by_name(dev, gpio_name, &cfgs->gpios[i],
                                                            GPIO_DESC_OUTPUT);
        ASSERT(ret == 0);
    }

    for (i = 0; i < names_count; i++) {
        printlk(LK_INFO, "%s:%d: Latency events config %s: gpio: %d event: %d\n",
                __PRETTY_FUNCTION__, __LINE__, cfgs->name[i],
                cfgs->gpios[i].nr, cfgs->events[i]);
    }

    return cfgs;
}

struct device_cfg_latencies * g_device_cfg_latencies;

void BOARD_LatencySetEvent(int event)
{
    unsigned i;

    if (g_device_cfg_latencies == NULL)
        return;

    for ( i = 0; i < g_device_cfg_latencies->cfg_cnt; i++ ) {
        int value;
        if (g_device_cfg_latencies->events[i] == (unsigned) event) {
            value = gpio_desc_get_value(&g_device_cfg_latencies->gpios[i]);
            gpio_desc_set_value(&g_device_cfg_latencies->gpios[i], !value);
            return;
        }
    }
}

static status_t latency_init(struct device *dev)
{
    printlk(LK_NOTICE, "%s:%d: Initializing latency module...\n",
            __PRETTY_FUNCTION__, __LINE__);
    g_device_cfg_latencies = of_device_get_latency_cfg(dev);

    if (!g_device_cfg_latencies)
        return ERR_NOT_FOUND;

    return 0;
}

static struct device_class latency_device_class = {
    .name = "none",
};
static struct driver_ops the_ops = {
        .device_class = &latency_device_class,
        .init = latency_init,
};

DRIVER_EXPORT_WITH_LVL(latency, &the_ops, DRIVER_INIT_TARGET);

