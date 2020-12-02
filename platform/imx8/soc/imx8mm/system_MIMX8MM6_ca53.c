/*
** ###################################################################
**     Processor:           MIMX8MM6DVTLZ
**     Compilers:           Keil ARM C/C++ Compiler
**                          GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manual:    MX8MMRM, Rev. A, 07/2018
**     Version:             rev. 2.0, 2018-07-20
**     Build:               b180801
**
**     Abstract:
**         Provides a system configuration function and a global variable that
**         contains the system frequency. It configures the device and initializes
**         the oscillator (PLL) that is part of the microcontroller device.
**
**     Copyright 2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2018 NXP
**     All rights reserved.
**
**     SPDX-License-Identifier: BSD-3-Clause
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
**     Revisions:
**     - rev. 1.0 (2018-03-26)
**         Initial version.
**     - rev. 2.0 (2018-07-20)
**         Rev.A Header EAR
**
** ###################################################################
*/

/*!
 * @file MIMX8MM6_cm4
 * @version 2.0
 * @date 2018-07-20
 * @brief Device specific configuration file for MIMX8MM6_cm4 (implementation
 *        file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 */

#include <stdint.h>
#include "fsl_device_registers.h"
#include "fsl_clock.h"

/* Local irq table and nesting level value */
static sys_irq_handle_t irqTable[NUMBER_OF_INT_VECTORS];

/* Local IRQ functions */
static void defaultIrqHandler (uint32_t giccIar, void *userParam) {
  while(1) {
  }
}

/*!
 * @brief CCM reg macros to extract corresponding registers bit field.
 */
#define CCM_BIT_FIELD_VAL(val, mask, shift)  (((val) & mask) >> shift)

/*!
 * @brief CCM reg macros to get corresponding registers values.
 */
#define CCM_ANALOG_REG_VAL(base, off) (*((volatile uint32_t *)((uintptr_t)base + off)))

uint32_t GetFracPllFreq(const volatile uint32_t *base)
{

    uint32_t fracCfg0 = CCM_ANALOG_REG_VAL(base, 0U);
    uint32_t fracCfg1 = CCM_ANALOG_REG_VAL(base, 4U);
    uint32_t fracCfg2 = CCM_ANALOG_REG_VAL(base, 8U);
    uint32_t refClkFreq = 0U;
    uint64_t fracClk = 0U;

    uint8_t refSel = CCM_BIT_FIELD_VAL(fracCfg0, CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_REF_CLK_SEL_MASK, CCM_ANALOG_AUDIO_PLL1_GEN_CTRL_PLL_REF_CLK_SEL_SHIFT);
    uint32_t mainDiv = CCM_BIT_FIELD_VAL(fracCfg1, CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_MAIN_DIV_MASK, CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_MAIN_DIV_SHIFT);
    uint8_t preDiv = CCM_BIT_FIELD_VAL(fracCfg1, CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_PRE_DIV_MASK, CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_PRE_DIV_SHIFT);
    uint8_t postDiv = CCM_BIT_FIELD_VAL(fracCfg1, CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_POST_DIV_MASK, CCM_ANALOG_AUDIO_PLL1_FDIV_CTL0_PLL_POST_DIV_SHIFT);
    uint32_t dsm = CCM_BIT_FIELD_VAL(fracCfg2, CCM_ANALOG_AUDIO_PLL1_FDIV_CTL1_PLL_DSM_MASK, CCM_ANALOG_AUDIO_PLL1_FDIV_CTL1_PLL_DSM_SHIFT);

    if(refSel == 0) /* OSC 24M Clock */
    {
       refClkFreq = CPU_XTAL_SOSC_CLK_24MHZ;
    }
    else
    {
       refClkFreq = CLK_PAD_CLK;  /* CLK_PAD_CLK Clock, please note that the value is 0hz by default, it could be set at system_MIMX8MMx_cm4.h :96 */
    }
    fracClk = (uint64_t)refClkFreq * (mainDiv * 65536 + dsm) / (65536 * preDiv * (1 << postDiv));

    return fracClk;
}

uint32_t GetIntegerPllFreq(const volatile uint32_t * base)
{
    uint32_t integerCfg0 = CCM_ANALOG_REG_VAL(base, 0U);
    uint32_t integerCfg1 = CCM_ANALOG_REG_VAL(base, 4U);
    uint32_t refClkFreq = 0U;
    uint64_t pllOutClock = 0U;

    uint8_t pllBypass = CCM_BIT_FIELD_VAL(integerCfg0, CCM_ANALOG_SYS_PLL1_GEN_CTRL_PLL_BYPASS_MASK, CCM_ANALOG_SYS_PLL1_GEN_CTRL_PLL_BYPASS_SHIFT);
    uint8_t refSel = CCM_BIT_FIELD_VAL(integerCfg0, CCM_ANALOG_SYS_PLL1_GEN_CTRL_PLL_REF_CLK_SEL_MASK, CCM_ANALOG_SYS_PLL1_GEN_CTRL_PLL_REF_CLK_SEL_SHIFT);
    uint32_t mainDiv = CCM_BIT_FIELD_VAL(integerCfg1, CCM_ANALOG_SYS_PLL1_FDIV_CTL0_PLL_MAIN_DIV_MASK, CCM_ANALOG_SYS_PLL1_FDIV_CTL0_PLL_MAIN_DIV_SHIFT);
    uint8_t preDiv = CCM_BIT_FIELD_VAL(integerCfg1, CCM_ANALOG_SYS_PLL1_FDIV_CTL0_PLL_PRE_DIV_MASK, CCM_ANALOG_SYS_PLL1_FDIV_CTL0_PLL_PRE_DIV_SHIFT);
    uint8_t postDiv = CCM_BIT_FIELD_VAL(integerCfg1, CCM_ANALOG_SYS_PLL1_FDIV_CTL0_PLL_POST_DIV_MASK, CCM_ANALOG_SYS_PLL1_FDIV_CTL0_PLL_POST_DIV_SHIFT);

    if(refSel == 0) /* OSC 24M Clock */
    {
       refClkFreq = CPU_XTAL_SOSC_CLK_24MHZ;
    }
    else
    {
       refClkFreq = CLK_PAD_CLK;  /* CLK_PAD_CLK Clock, please note that the value is 0hz by default, it could be set at system_MIMX8MMx_cm4.h :96 */
    }

    if(pllBypass)
    {
        pllOutClock = refClkFreq;
    }

    else
    {
        pllOutClock = (uint64_t)refClkFreq * mainDiv / (((uint64_t)(1U) << postDiv) * preDiv);
    }

    return pllOutClock;
}


/* ----------------------------------------------------------------------------
   -- Core clock
   ---------------------------------------------------------------------------- */

uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;


/* ----------------------------------------------------------------------------
   -- SystemInit()
   ---------------------------------------------------------------------------- */

void SystemInit (int doGlobalInit)
{
    /* GIC distributor setup should be skipped if we are not the main core */
    if(doGlobalInit)
        GIC_DistInit();

    GIC_RedistInit();
    GIC_CPUInterfaceInit();

    SystemInitHook();
}

/* ----------------------------------------------------------------------------
   -- SystemCoreClockUpdate()
   ---------------------------------------------------------------------------- */

void SystemCoreClockUpdate (void) {


  volatile uint32_t * M4_ClockRoot = (uint32_t *)(&(CCM)->ROOT[1].TARGET_ROOT);
  uint32_t pre  = ((*M4_ClockRoot & CCM_TARGET_ROOT_PRE_PODF_MASK) >> CCM_TARGET_ROOT_PRE_PODF_SHIFT) + 1U;
  uint32_t post = ((*M4_ClockRoot & CCM_TARGET_ROOT_POST_PODF_MASK) >> CCM_TARGET_ROOT_POST_PODF_SHIFT) + 1U;

  uint32_t freq = 0U;

  CLOCK_EnableClock(kCLOCK_Gpio1);

  switch((*M4_ClockRoot & CCM_TARGET_ROOT_MUX_MASK) >> CCM_TARGET_ROOT_MUX_SHIFT)
    {
        case 0U: /* OSC 24M Clock */
            freq = CPU_XTAL_SOSC_CLK_24MHZ;
            break;
        case 1U: /* System PLL2 DIV5 */
            freq = GetIntegerPllFreq(&(CCM_ANALOG->SYS_PLL2_GEN_CTRL)) / 5; /* Get System PLL2 DIV5 freq */
            break;
        case 2U: /* System PLL2 DIV4 */
            freq = GetIntegerPllFreq(&(CCM_ANALOG->SYS_PLL2_GEN_CTRL)) / 4; /* Get System PLL2 DIV4 freq */
            break;
        case 3U: /* System PLL1 DIV3 */
            freq = GetIntegerPllFreq(&(CCM_ANALOG->SYS_PLL1_GEN_CTRL)) / 3;  /* Get System PLL1 DIV3 freq */
            break;
        case 4U: /* System PLL1 */
            freq = GetIntegerPllFreq(&(CCM_ANALOG->SYS_PLL1_GEN_CTRL));      /* Get System PLL1 freq */
            break;
        case 5U: /* AUDIO PLL1 */
             freq = GetFracPllFreq(&(CCM_ANALOG->AUDIO_PLL1_GEN_CTRL));  /* Get AUDIO PLL1 freq */
            break;
        case 6U: /* VIDEO PLL1 */
              freq = GetFracPllFreq(&(CCM_ANALOG->VIDEO_PLL1_GEN_CTRL));  /* Get VIDEO PLL1 freq */
            break;
        case 7U: /* System PLL3 */
             freq = GetIntegerPllFreq(&(CCM_ANALOG->SYS_PLL3_GEN_CTRL));    /* Get System PLL3 freq */
            break;
        default:
            break;
    }

  SystemCoreClock = freq / pre / post;
}

/* ----------------------------------------------------------------------------
   -- SystemInitHook()
   ---------------------------------------------------------------------------- */

__attribute__ ((weak)) void SystemInitHook (void)
{
  /* Void implementation of the weak function. */
}

/* ----------------------------------------------------------------------------
   -- SystemInitIrqTable()
   ---------------------------------------------------------------------------- */

void SystemInitIrqTable (void) {
  uint32_t i;

  /* First set all handler to default */
  for (i = 0; i < NUMBER_OF_INT_VECTORS; i++) {
    SystemInstallIrqHandler((IRQn_Type)i, defaultIrqHandler, NULL);
  }

}

/* ----------------------------------------------------------------------------
   -- SystemInstallIrqHandler()
   ---------------------------------------------------------------------------- */

void SystemInstallIrqHandler(IRQn_Type irq, system_irq_handler_t handler, void *userParam) {
  irqTable[irq].irqHandler = handler;
  irqTable[irq].userParam = userParam;
}

/* ----------------------------------------------------------------------------
   -- SystemIrqHandler()
   ---------------------------------------------------------------------------- */

#if defined(__IAR_SYSTEMS_ICC__)
#pragma weak SystemIrqHandler
void SystemIrqHandler(uint32_t giccIar) {
#elif defined(__GNUC__)
__attribute__((weak)) void SystemIrqHandler(uint32_t giccIar) {
#else
  #error Not supported compiler type
#endif
  uint32_t intNum = giccIar & 0x3FFUL;

  /* Spurious interrupt ID or Wrong interrupt number */
  if ((intNum == 1023) || (intNum >= NUMBER_OF_INT_VECTORS))
  {
    return;
  }

  /* Now call the real irq handler for intNum */
  if(irqTable[intNum].irqHandler)
    irqTable[intNum].irqHandler(giccIar, irqTable[intNum].userParam);
}
