/*
 * Copyright (c) 2017 Google Inc. All rights reserved
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

#include <imx-regs.h>
#include <debug.h>
//#include <dev/interrupt/arm_gic.h>
#include <dev/timer/arm_generic.h>
#include <dev/psci.h>
#include <kernel/vm.h>
#include <lk/init.h>
#include <string.h>
#include "fsl_clock.h"
#include "fsl_iomuxc.h"
#include "dev/driver.h"
#include "fuse_check.h"

#ifdef WITH_LIB_APPARGS
#include <lib/appargs.h>
#endif


#define ARM_GENERIC_TIMER_INT_CNTV 27
#define ARM_GENERIC_TIMER_INT_CNTPS 29
#define ARM_GENERIC_TIMER_INT_CNTP 30
#define TIMER_ARM_GENERIC_SELECTED CNTP

#define ARM_GENERIC_TIMER_INT_SELECTED(timer) ARM_GENERIC_TIMER_INT_ ## timer
#define XARM_GENERIC_TIMER_INT_SELECTED(timer) ARM_GENERIC_TIMER_INT_SELECTED(timer)
#define ARM_GENERIC_TIMER_INT XARM_GENERIC_TIMER_INT_SELECTED(TIMER_ARM_GENERIC_SELECTED)

extern void arm_gic_v3_init(void);

/* initial memory mappings. parsed by start.S */
struct mmu_initial_mapping mmu_initial_mappings[] = {
    /* Mark next entry as dynamic as it might be updated
       by platform_reset code to specify actual size and
       location of RAM to use */
    {
        .phys = MEMBASE,
        .virt = KERNEL_BASE,
        .size = MEMSIZE,
        .flags = 0,
        .name = "ram"
    },
#if IMX_USE_OCRAM
    {
        .phys = SRAM_PBASE,
        .virt = SRAM_VBASE,
        .size = SRAM_SIZE,
        .flags = MMU_INITIAL_MAPPING_FLAG_DEVICE,
        .name = "sram"
    },
#endif
    {
        .phys = SOC_REGS_PHY,
        .virt = SOC_REGS_VIRT,
        .size = SOC_REGS_SIZE,
        .flags = MMU_INITIAL_MAPPING_FLAG_DEVICE,
        .name = "soc"
    },
    /* null entry to terminate the list */
    { 0 }
};

/* Initial ram definition. May be changed from appargs lib, depending device
 * tree properties */
pmm_arena_t ram_arena = {
    .name  = "ram",
    .base  =  MEMBASE,
    .size  =  MEMSIZE,
    .flags =  PMM_ARENA_FLAG_KMAP
};

extern paddr_t ram_phys_start;

void platform_init_mmu_mappings(void)
{
}

void platform_init(void)
{
    device_init_all();

}


extern void arm_reset(void);
static void platform_boot_secondary_cpus(void)
{
    uint64_t cluster = 0;
    uint64_t cpuid = 1;

    uintptr_t sec_entry = (uintptr_t)(&arm_reset);
    sec_entry -= KERNEL_BASE;
    sec_entry += MEMBASE;

    while (cpuid < arch_max_num_cpus()) {
        int32_t ret = psci_cpu_on(cluster, cpuid, sec_entry);
        printf("Enabling cpu %lld returned %x\n", cpuid, ret);
        cpuid++;
    }
}

void platform_early_init(void)
{
    size_t memsize = MEMSIZE;
    int err;
    paddr_t mempaddr;

    SystemCoreClockUpdate();

    arm_gic_v3_init();
#define IMX8_USE_HVC 0
    arm_psci_init(IMX8_USE_HVC, "nothing", "nothing", "nothing", "nothing");

    platform_boot_secondary_cpus();

    arm_generic_timer_init(ARM_GENERIC_TIMER_INT, 0);

    /* add the main memory arena */
    err = of_get_meminfo(&memsize);
    if (!err) {
        ASSERT(memsize <= MEMSIZE);
        ram_arena.size = memsize;
    }

    printf("Initialize memory arena: memsize 0x%lX\n", ram_arena.size);
    pmm_add_arena(&ram_arena);

    err = of_get_mempaddr(&mempaddr);
    if (!err) {
        ram_phys_start = mempaddr;
    }

    /* reserve the first 64k of ram, which should be holding the fdt */
    struct list_node list = LIST_INITIAL_VALUE(list);
    pmm_alloc_range(MEMBASE, KERNEL_LOAD_OFFSET / PAGE_SIZE, &list);

    fuse_check();
}

#ifdef WITH_LIB_APPARGS

#define kCLOCK_Idx_RootNone 0xFFFFFFFF
#define kCLOCK_Divider_None 0xFFFFFFFF
#define kCLOCK_None 0xFFFFFFFF

void devcfg_set_clock(struct device_cfg_clk *cfg)
{
    uint32_t ccm_tuple = CCM_TUPLE(cfg->ccgr, cfg->ccgr_root);
    uintptr_t kclock_root = get_kCLOCK_Root_by_index(cfg->root_clk_idx);

    if (kclock_root) {
        printlk(LK_INFO, "%s:%d: Configuring root mux clock [%x@%lx ] to %x...\n",
                __PRETTY_FUNCTION__, __LINE__, cfg->root_clk_idx,
                kclock_root, cfg->root_clk_mux_idx);
        CLOCK_SetRootMux(kclock_root, cfg->root_clk_mux_idx);

        if ((cfg->pre_divider != kCLOCK_Divider_None)
                && (cfg->post_divider != kCLOCK_Divider_None)) {
            printlk(LK_INFO, "%s:%d: Configuring root clock dividers [%x@%lx ] to [%x-%x]...\n",
                    __PRETTY_FUNCTION__, __LINE__,cfg->root_clk_idx,
                    kclock_root, cfg->pre_divider, cfg->post_divider);
            CLOCK_SetRootDivider(kclock_root, cfg->pre_divider, cfg->post_divider);
        }
    }

    if (ccm_tuple != kCLOCK_None)
        CLOCK_EnableClock(ccm_tuple);

}
#define DEVCFG_PIN_TO_IOMUXC(pin) pin->muxRegister, pin->muxMode, pin->inputRegister, pin->inputDaisy, pin->configRegister
void devcfg_set_pin(struct device_cfg_pin *pin)
{
    printlk(LK_INFO, "%s:%d: Configuring pin: %x %x %x %x %x %x %x\n",
            __PRETTY_FUNCTION__, __LINE__, pin->muxRegister, pin->muxMode,
            pin->inputRegister, pin->inputDaisy, pin->configRegister,
            pin->inputOnfield, pin->configValue);
    IOMUXC_SetPinMux(DEVCFG_PIN_TO_IOMUXC(pin), pin->inputOnfield);
    IOMUXC_SetPinConfig(DEVCFG_PIN_TO_IOMUXC(pin), pin->configValue);
}

void devcfg_set_dpll(struct device_cfg_dpll *dpll)
{
    ccm_analog_frac_pll_config_t audioPllConfig;
    audioPllConfig.refSel = dpll->refSel;
    audioPllConfig.mainDiv = dpll->mdiv;
    audioPllConfig.dsm = dpll->kdiv;
    audioPllConfig.preDiv = dpll->pdiv;
    audioPllConfig.postDiv = dpll->sdiv;
    unsigned pll_id = dpll->id;

    printlk(LK_INFO, "Setup Audio pll(%x): %u %d %d %d %d\n",
            pll_id, dpll->rate, dpll->mdiv, dpll->pdiv, dpll->sdiv, dpll->kdiv);

    switch(pll_id) {
    case kCLOCK_AudioPll1Ctrl:
        CLOCK_InitAudioPll1(&audioPllConfig);
        break;
    case kCLOCK_AudioPll2Ctrl:
        CLOCK_InitAudioPll2(&audioPllConfig);
        break;
    case kCLOCK_VideoPll1Ctrl:
        CLOCK_InitVideoPll1(&audioPllConfig);
        break;
    default:
        panic("No support of audio %x!\n", pll_id);
    }

}
#endif
