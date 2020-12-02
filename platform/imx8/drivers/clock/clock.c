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

#include <lib/appargs.h>
#include <stdlib.h>
#include <dev/driver.h>
#include <err.h>
#include <string.h>
#include <assert.h>
#include <dev/class/sai.h>
#include <dev/sai_ioctl.h>
#include <dev/gpr_ioctl.h>
#include "fsl_clock.h"
#include <platform/clock.h>
#include <debug.h>


//#define __DRY_RUN

#define MAX_CONFIG      16
#define CLOCK_AUDIO_BUS_RATE 200000000

struct clock_audio_config_s {
    struct list_node list;
    int cfg_count;
    struct device_cfg_config *active_cfg;
    struct device *dev_sai;
    struct device *dev_gpr;
    uint32_t bus_rate;
    /* pll */
    struct device_cfg_dplls *plls;
};

static struct clock_audio_config_s *g_clock_audio_configs;

static status_t _clock_audio_set_config(const char *);
static status_t _clock_audio_get_pll_config(const char *, struct device_cfg_dpll **);
static status_t _clock_audio_set_pll_config(struct device_cfg_dpll *, unsigned *);

/* API */
status_t clock_audio_set_config(struct device * dev, const char *config_name)
{
    return _clock_audio_set_config(config_name);
}

status_t clock_audio_adjust_pll(struct device * dev, unsigned id, int ppb)
{
    clock_pll_ctrl_t pll;
    switch (id) {
    case 0:
        pll = kCLOCK_AudioPll1Ctrl;
        break;
    case 1:
        pll = kCLOCK_AudioPll2Ctrl;
        break;
    default:
        return ERR_INVALID_ARGS;
    };

    CLOCK_AdjustPllFreq(pll, ppb);

    return 0;
}

status_t clock_audio_set_pll_config(struct device *dev, const char *config_name,
                                    unsigned *freq, unsigned *k)
{
    status_t ret;
    struct device_cfg_dpll *pll;

    do {
        /* Look for matching configuration name */
        printlk(LK_INFO, "%s: Look for %s config\n", __func__, config_name);
        ret = _clock_audio_get_pll_config(config_name, &pll);
        if (ret)
            break;

        /* Fine-tune k factor - if provided */
        if (k) {
            pll->kdiv = *k;
        }

        /* Apply configuration */
        printlk(LK_INFO, "%s: apply %s config\n", __func__, config_name);
        ret = _clock_audio_set_pll_config(pll, freq);
    } while (0);

    return ret;
}

#define kCLOCK_RootInvalid ((clock_root_control_t)NULL)

#if ((defined SOC_IMX8MM) && (SOC_IMX8MM == 1))
#define IMX_CLOCKROOT_MAP { \
    kCLOCK_RootInvalid, kCLOCK_RootSai1, kCLOCK_RootSai2, kCLOCK_RootSai3, \
    kCLOCK_RootSai4, kCLOCK_RootSai5, kCLOCK_RootSai6, kCLOCK_RootInvalid }
#elif ((defined SOC_IMX8MN) && (SOC_IMX8MN == 1))
#define IMX_CLOCKROOT_MAP { \
    kCLOCK_RootInvalid, kCLOCK_RootInvalid, kCLOCK_RootSai2, kCLOCK_RootSai3, \
    kCLOCK_RootInvalid, kCLOCK_RootSai5, kCLOCK_RootSai6, kCLOCK_RootSai7 }
#else
#error "Target SoC not supported"
#endif

static const clock_root_control_t s_saiClockRoot[] = IMX_CLOCKROOT_MAP;

status_t clock_audio_set_sai_mclk_config(struct device *dev,
                                    unsigned id,
                                    const char *config_name,
                                    unsigned *freq,
                                    unsigned *k)
{
    unsigned pll_freq;
    status_t ret;

    ret = clock_audio_set_pll_config(dev, config_name, &pll_freq, k);
    if (ret) {
        printlk(LK_ERR, "%s:%d: Returning error on sai %d while setting pll config %s\n",
                        __PRETTY_FUNCTION__, __LINE__, id, config_name);
        return ret;
    }

    clock_root_control_t sai_root;

    if (id >= sizeof(s_saiClockRoot) / sizeof(s_saiClockRoot[0]))
        return ERR_INVALID_ARGS;

    sai_root = s_saiClockRoot[id];

    *freq = CLOCK_GetSaiFreq(sai_root);
    if (*freq == 0) {
        printlk(LK_ERR, "%s:%d: Returning error on sai %d for clock rate %d\n",
                __PRETTY_FUNCTION__, __LINE__, id, *freq);
        return ERR_INVALID_ARGS;
    }

    printlk(LK_INFO, "Returning sai %d clock rate %d\n", id, *freq);

    return 0;
}

int32_t clock_audio_get_bus_rate(void)
{
    if (!g_clock_audio_configs) {
        printlk(LK_INFO, "--Not yet ready\n");
        return ERR_NOT_READY;
    }

    return g_clock_audio_configs->bus_rate;
}

/* static functions */
static status_t _clock_audio_get_pll_config(const char *config_name,
                                            struct device_cfg_dpll **pll)
{
    if (!g_clock_audio_configs) {
        printlk(LK_ERR, "%s:%d: --Not yet ready\n", __PRETTY_FUNCTION__, __LINE__);
        return ERR_NOT_READY;
    }

    struct device_cfg_config *config = g_clock_audio_configs->active_cfg;

    if (!config) {
        printlk(LK_ERR, "%s:%d: No active config while requesting pll %s\n",
                __PRETTY_FUNCTION__, __LINE__, config_name);
        return ERR_NOT_READY;
    }

    struct device_cfg_dplls *plls = g_clock_audio_configs->plls;
    unsigned i;
    for (i = 0; i < plls->plls_cnt; i++) {
        printlk(LK_NOTICE, "Checking %s against %s\n",
                plls->name[i], config_name);
        if (strcmp(plls->name[i], config_name) == 0) {
            *pll = &plls->cfg[i];
            ASSERT(pll);
            return 0;
        }
    }

    return ERR_NOT_FOUND;
}

static status_t _clock_audio_set_pll_config(struct device_cfg_dpll *pll,
                                            unsigned *freq)
{
    if (freq)
        *freq = 0;

    devcfg_set_dpll(pll);
    return 0;
}

static status_t _clock_audio_set_config(const char *config_name)
{
    struct device_cfg_config *config = NULL, *temp;
    unsigned i;

    printlk(LK_NOTICE, "%s: apply %s config\n", __func__, config_name);

    if (!g_clock_audio_configs) {
        printlk(LK_ERR, "%s:%d: --Not yet ready\n", __PRETTY_FUNCTION__,
                __LINE__);
        return ERR_NOT_READY;
    }

    list_for_every_entry(&g_clock_audio_configs->list, temp,
                         struct device_cfg_config, node) {
        if (strcmp(temp->name, config_name) == 0) {
            config = temp;
            break;
        }
    }

    if (!config)
        return ERR_NOT_FOUND;

    struct device_cfg_dplls *plls = g_clock_audio_configs->plls;
    for (i = 0; i < plls->plls_cnt; i++) {
        if (!(config->pll_init_mask & (1 << i)))
            continue;
        struct device_cfg_dpll *pll = &plls->cfg[i];
        ASSERT(pll);
        devcfg_set_dpll(pll);
    }

    for (i = 0; i < config->clk_count; i++) {
        struct device_cfg_clk *clk = config->clk[i];
        ASSERT(clk);
        printlk(LK_NOTICE, "Apply %s config\n", config->clk_name[i]);
#ifndef __DRY_RUN
        devcfg_set_clock(clk);
#endif
    }

    if (!g_clock_audio_configs->dev_gpr) {
        printlk(LK_NOTICE, "Skip gpr config, no device\n");
        goto skip_gpr;
    }

    printlk(LK_NOTICE, "Apply gpr config\n");
    for (i = 0; i < MAX_GPR_REG; i++) {
        uint32_t value, mask;
        mask = config->gpr[i * 3 + 1];
        value = config->gpr[i * 3 + 2];

        if (mask == 0)
            continue;

        struct gpr_ioc_cmd_reg gpr_reg = {
            .offset = config->gpr[i * 3],
        };

        device_ioctl(g_clock_audio_configs->dev_gpr, GPR_IOC_READ, &gpr_reg);
        printlk(LK_NOTICE, "read 0x%X from gpr offset %d\n", gpr_reg.value, gpr_reg.offset);

        gpr_reg.value &= ~mask;
        gpr_reg.value |= (value & mask);

        device_ioctl(g_clock_audio_configs->dev_gpr, GPR_IOC_WRITE, &gpr_reg);
        printlk(LK_NOTICE, "write 0x%X at gpr offset %d, with 0x%X mask\n",
               gpr_reg.value, gpr_reg.offset, mask);
    }

skip_gpr:
    printlk(LK_NOTICE, "Apply gpio config\n");
    if (!config->gpio.dev) {
        printlk(LK_NOTICE, "Skip gpio config, no device\n");
        goto skip_gpio;
    }
    /* Just need to setup GPIO direction, as it also appropriately sets
     * active state if requested by device tree.
     */
    gpio_desc_set_direction(&config->gpio);

skip_gpio:
    printlk(LK_NOTICE, "Set sai clock mode as %d\n", config->clk_mode);
    device_ioctl(g_clock_audio_configs->dev_sai, SAI_IOC_CLK_MODE, &config->clk_mode);

    g_clock_audio_configs->active_cfg = config;

    return 0;
}

static status_t clock_audio_init(struct device *dev)
{
    const char *config_names[MAX_CONFIG];
    struct device_cfg_config *config, *temp;
    struct device *sai_dev, *gpr_dev;

    printlk(LK_NOTICE, "Entering clock audio init\n");

    sai_dev = of_device_lookup_device(dev, "sai");
    if (sai_dev == NULL) {
        printlk(LK_ERR, "%s:%d: Invalid SAI phandle\n", __PRETTY_FUNCTION__,
                __LINE__);
        return ERR_INVALID_ARGS;
    }

    gpr_dev = of_device_lookup_device(dev, "gpr");
    if (gpr_dev == NULL) {
        printlk(LK_ERR, "%s:%d: Invalid GPR phandle\n", __PRETTY_FUNCTION__,
                __LINE__);
        return ERR_INVALID_ARGS;
    }

    struct clock_audio_config_s *clk_audio_configs =
                malloc(sizeof(struct clock_audio_config_s));

    if (!clk_audio_configs)
        return ERR_NO_MEMORY;

    dev->state = clk_audio_configs;
    g_clock_audio_configs = clk_audio_configs;

    /* Initialize audio config structure */
    memset(clk_audio_configs, 0, sizeof(struct clock_audio_config_s));
    list_initialize(&clk_audio_configs->list);

    int nb_configs = of_device_get_strings(dev, "config-names", config_names,
                                           MAX_CONFIG);

    int i;
    for (i = 0; i < nb_configs; i++) {
        config = of_device_get_config_by_name(dev, config_names[i]);

        if (config == NULL) {
            goto error;
        }

        printlk(LK_NOTICE, "%s:%d: %s config registered\n", __PRETTY_FUNCTION__,
                __LINE__, config->name);
        list_add_tail(&clk_audio_configs->list, &config->node);
    }

    clk_audio_configs->plls = of_device_get_plls(dev);
    ASSERT(clk_audio_configs->plls);

    /* Initialize other properties */
    clk_audio_configs->dev_sai = sai_dev;
    clk_audio_configs->dev_gpr = gpr_dev;

    /* Get bus clock rate */
    if (of_device_get_int32(dev, "bus-clock-rate",
                            &g_clock_audio_configs->bus_rate)) {

        printlk(LK_ERR, "Bus clock rate missing. Fallback default 200MHz\n");
        g_clock_audio_configs->bus_rate = CLOCK_AUDIO_BUS_RATE;
    }

#ifdef __DRY_RUN
    /* For test */
    _clock_audio_set_config("spdif");
    _clock_audio_set_config("hdmi");
    _clock_audio_set_config("dummy");
    _clock_audio_set_config("adc");
#endif

    return 0;

error:
    list_for_every_entry_safe(&clk_audio_configs->list, config, temp,
                         struct device_cfg_config, node) {
        list_delete(&config->node);
        free(config);
    }

    free(clk_audio_configs);

    return ERR_NOT_FOUND;
}

static struct driver_ops the_ops = {
    .device_class = NULL,
    .init = clock_audio_init,
};

DRIVER_EXPORT_WITH_LVL(imx_clock_audio, &the_ops, DRIVER_INIT_PLATFORM);
