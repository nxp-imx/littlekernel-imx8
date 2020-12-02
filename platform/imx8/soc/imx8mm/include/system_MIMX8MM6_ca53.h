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
 * @brief Device specific configuration file for MIMX8MM6_cm4 (header file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 */

#ifndef _SYSTEM_MIMX8MM6_cm4_H_
#define _SYSTEM_MIMX8MM6_cm4_H_                  /**< Symbol preventing repeated inclusion */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


/* i.MX8MM Definitions */
#ifndef DISABLE_WDOG
  #define DISABLE_WDOG      1
#endif
/* Define clock source values */
#define CPU_XTAL_SOSC_CLK_24MHZ           24000000u           /* Value of the external System Oscillator Clock(SOSC) frequency in Hz */
#define CLK_PAD_CLK                       0u                  /* The value could be changed according to the  actual usage */
#define DEFAULT_SYSTEM_CLOCK              400000000u          /* Default System clock value */

typedef void (*system_irq_handler_t) (uint32_t giccIar, void *param);
/**
 * @brief IRQ handle for specific IRQ
 */
typedef struct _sys_irq_handle
{
    system_irq_handler_t irqHandler; /**< IRQ handler for specific IRQ */
    void *userParam;                 /**< User param for handler callback */
} sys_irq_handle_t;

/**
 * @brief System clock frequency (core clock)
 *
 * The system clock frequency supplied to the SysTick timer and the processor
 * core clock. This variable can be used by the user application to setup the
 * SysTick timer or configure other parameters. It may also be used by debugger to
 * query the frequency of the debug timer or configure the trace clock speed
 * SystemCoreClock is initialized with a correct predefined value.
 */
extern uint32_t SystemCoreClock;

/**
 * @brief Setup the microcontroller system.
 *
 * Typically this function configures the oscillator (PLL) that is part of the
 * microcontroller device. For systems with variable clock speed it also updates
 * the variable SystemCoreClock. SystemInit is called from startup_device file.
 */
void SystemInit (int doGlobalInit);

/**
 * @brief Updates the SystemCoreClock variable.
 *
 * It must be called whenever the core clock is changed during program
 * execution. SystemCoreClockUpdate() evaluates the clock register settings and calculates
 * the current core clock.
 */
void SystemCoreClockUpdate (void);

/**
 * @brief SystemInit function hook.
 *
 * This weak function allows to call specific initialization code during the
 * SystemInit() execution.This can be used when an application specific code needs
 * to be called as close to the reset entry as possible (for example the Multicore
 * Manager MCMGR_EarlyInit() function call).
 * NOTE: No global r/w variables can be used in this hook function because the
 * initialization of these variables happens after this function.
 */
void SystemInitHook (void);

/**
 * @brief Initialize IRQ table, set default handlers
 */
void SystemInitIrqTable (void);

/**
 * @brief Install IRQ handler for specific IRQ
 *
 * It can't be called at interrupt context to avoid IRQ table corrupt during interrupt preemption
 *
 * @param irq IRQ number corresponds to the installed handler
 * @param handler IRQ handler for the IRQ number
 * @param userParam User specified parameter for IRQ handler callback
 */
void SystemInstallIrqHandler (IRQn_Type irq, system_irq_handler_t handler, void *userParam);

/**
 * @brief System IRQ handler which dispatches specific IRQ to corresponding registered handler.
 *
 * It is called from IRQ exception context and dispatches to registered handler according to
 * GICC_IAR interrupt number.
 * The default implementation is weak and user can override this function with his own SystemIrqHandler.
 *
 * @param giccIar IRQ acknowledge value read from GICC_IAR
 */
void SystemIrqHandler (uint32_t giccIar);

#ifdef __cplusplus
}
#endif

#endif  /* _SYSTEM_MIMX8MM6_cm4_H_ */
