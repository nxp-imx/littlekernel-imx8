/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
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

#ifndef __IMX_GPIO_HW
#define __IMX_GPIO_HW
#include "sys/types.h"
#include "compiler.h"
#include <assert.h>

/*!
 * @addtogroup gpio_driver
 * @{
 */
#define FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL true

/* ----------------------------------------------------------------------------
   -- GPIO Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Peripheral_Access_Layer GPIO Peripheral Access Layer
 * @{
 */

/** GPIO - Register Layout Typedef */
typedef struct {
  __IO uint32_t DR;                                /**< GPIO data register, offset: 0x0 */
  __IO uint32_t GDIR;                              /**< GPIO direction register, offset: 0x4 */
  __I  uint32_t PSR;                               /**< GPIO pad status register, offset: 0x8 */
  __IO uint32_t ICR1;                              /**< GPIO interrupt configuration register1, offset: 0xC */
  __IO uint32_t ICR2;                              /**< GPIO interrupt configuration register2, offset: 0x10 */
  __IO uint32_t IMR;                               /**< GPIO interrupt mask register, offset: 0x14 */
  __IO uint32_t ISR;                               /**< GPIO interrupt status register, offset: 0x18 */
  __IO uint32_t EDGE_SEL;                          /**< GPIO edge select register, offset: 0x1C */
} GPIO_Type;

/* ----------------------------------------------------------------------------
   -- GPIO Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Register_Masks GPIO Register Masks
 * @{
 */

/*! @name DR - GPIO data register */
/*! @{ */
#define GPIO_DR_DR_MASK                          (0xFFFFFFFFU)
#define GPIO_DR_DR_SHIFT                         (0U)
#define GPIO_DR_DR(x)                            (((uint32_t)(((uint32_t)(x)) << GPIO_DR_DR_SHIFT)) & GPIO_DR_DR_MASK)
/*! @} */

/*! @name GDIR - GPIO direction register */
/*! @{ */
#define GPIO_GDIR_GDIR_MASK                      (0xFFFFFFFFU)
#define GPIO_GDIR_GDIR_SHIFT                     (0U)
/*! GDIR
 *  0b00000000000000000000000000000000..GPIO is configured as input.
 *  0b00000000000000000000000000000001..GPIO is configured as output.
 */
#define GPIO_GDIR_GDIR(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_GDIR_GDIR_SHIFT)) & GPIO_GDIR_GDIR_MASK)
/*! @} */

/*! @name PSR - GPIO pad status register */
/*! @{ */
#define GPIO_PSR_PSR_MASK                        (0xFFFFFFFFU)
#define GPIO_PSR_PSR_SHIFT                       (0U)
#define GPIO_PSR_PSR(x)                          (((uint32_t)(((uint32_t)(x)) << GPIO_PSR_PSR_SHIFT)) & GPIO_PSR_PSR_MASK)
/*! @} */

/*! @name ICR1 - GPIO interrupt configuration register1 */
/*! @{ */
#define GPIO_ICR1_ICR0_MASK                      (0x3U)
#define GPIO_ICR1_ICR0_SHIFT                     (0U)
/*! ICR0
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR1_ICR0(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_ICR1_ICR0_SHIFT)) & GPIO_ICR1_ICR0_MASK)
#define GPIO_ICR1_ICR1_MASK                      (0xCU)
#define GPIO_ICR1_ICR1_SHIFT                     (2U)
/*! ICR1
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR1_ICR1(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_ICR1_ICR1_SHIFT)) & GPIO_ICR1_ICR1_MASK)
#define GPIO_ICR1_ICR2_MASK                      (0x30U)
#define GPIO_ICR1_ICR2_SHIFT                     (4U)
/*! ICR2
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR1_ICR2(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_ICR1_ICR2_SHIFT)) & GPIO_ICR1_ICR2_MASK)
#define GPIO_ICR1_ICR3_MASK                      (0xC0U)
#define GPIO_ICR1_ICR3_SHIFT                     (6U)
/*! ICR3
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR1_ICR3(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_ICR1_ICR3_SHIFT)) & GPIO_ICR1_ICR3_MASK)
#define GPIO_ICR1_ICR4_MASK                      (0x300U)
#define GPIO_ICR1_ICR4_SHIFT                     (8U)
/*! ICR4
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR1_ICR4(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_ICR1_ICR4_SHIFT)) & GPIO_ICR1_ICR4_MASK)
#define GPIO_ICR1_ICR5_MASK                      (0xC00U)
#define GPIO_ICR1_ICR5_SHIFT                     (10U)
/*! ICR5
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR1_ICR5(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_ICR1_ICR5_SHIFT)) & GPIO_ICR1_ICR5_MASK)
#define GPIO_ICR1_ICR6_MASK                      (0x3000U)
#define GPIO_ICR1_ICR6_SHIFT                     (12U)
/*! ICR6
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR1_ICR6(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_ICR1_ICR6_SHIFT)) & GPIO_ICR1_ICR6_MASK)
#define GPIO_ICR1_ICR7_MASK                      (0xC000U)
#define GPIO_ICR1_ICR7_SHIFT                     (14U)
/*! ICR7
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR1_ICR7(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_ICR1_ICR7_SHIFT)) & GPIO_ICR1_ICR7_MASK)
#define GPIO_ICR1_ICR8_MASK                      (0x30000U)
#define GPIO_ICR1_ICR8_SHIFT                     (16U)
/*! ICR8
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR1_ICR8(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_ICR1_ICR8_SHIFT)) & GPIO_ICR1_ICR8_MASK)
#define GPIO_ICR1_ICR9_MASK                      (0xC0000U)
#define GPIO_ICR1_ICR9_SHIFT                     (18U)
/*! ICR9
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR1_ICR9(x)                        (((uint32_t)(((uint32_t)(x)) << GPIO_ICR1_ICR9_SHIFT)) & GPIO_ICR1_ICR9_MASK)
#define GPIO_ICR1_ICR10_MASK                     (0x300000U)
#define GPIO_ICR1_ICR10_SHIFT                    (20U)
/*! ICR10
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR1_ICR10(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ICR1_ICR10_SHIFT)) & GPIO_ICR1_ICR10_MASK)
#define GPIO_ICR1_ICR11_MASK                     (0xC00000U)
#define GPIO_ICR1_ICR11_SHIFT                    (22U)
/*! ICR11
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR1_ICR11(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ICR1_ICR11_SHIFT)) & GPIO_ICR1_ICR11_MASK)
#define GPIO_ICR1_ICR12_MASK                     (0x3000000U)
#define GPIO_ICR1_ICR12_SHIFT                    (24U)
/*! ICR12
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR1_ICR12(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ICR1_ICR12_SHIFT)) & GPIO_ICR1_ICR12_MASK)
#define GPIO_ICR1_ICR13_MASK                     (0xC000000U)
#define GPIO_ICR1_ICR13_SHIFT                    (26U)
/*! ICR13
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR1_ICR13(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ICR1_ICR13_SHIFT)) & GPIO_ICR1_ICR13_MASK)
#define GPIO_ICR1_ICR14_MASK                     (0x30000000U)
#define GPIO_ICR1_ICR14_SHIFT                    (28U)
/*! ICR14
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR1_ICR14(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ICR1_ICR14_SHIFT)) & GPIO_ICR1_ICR14_MASK)
#define GPIO_ICR1_ICR15_MASK                     (0xC0000000U)
#define GPIO_ICR1_ICR15_SHIFT                    (30U)
/*! ICR15
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR1_ICR15(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ICR1_ICR15_SHIFT)) & GPIO_ICR1_ICR15_MASK)
/*! @} */

/*! @name ICR2 - GPIO interrupt configuration register2 */
/*! @{ */
#define GPIO_ICR2_ICR16_MASK                     (0x3U)
#define GPIO_ICR2_ICR16_SHIFT                    (0U)
/*! ICR16
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR2_ICR16(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ICR2_ICR16_SHIFT)) & GPIO_ICR2_ICR16_MASK)
#define GPIO_ICR2_ICR17_MASK                     (0xCU)
#define GPIO_ICR2_ICR17_SHIFT                    (2U)
/*! ICR17
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR2_ICR17(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ICR2_ICR17_SHIFT)) & GPIO_ICR2_ICR17_MASK)
#define GPIO_ICR2_ICR18_MASK                     (0x30U)
#define GPIO_ICR2_ICR18_SHIFT                    (4U)
/*! ICR18
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR2_ICR18(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ICR2_ICR18_SHIFT)) & GPIO_ICR2_ICR18_MASK)
#define GPIO_ICR2_ICR19_MASK                     (0xC0U)
#define GPIO_ICR2_ICR19_SHIFT                    (6U)
/*! ICR19
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR2_ICR19(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ICR2_ICR19_SHIFT)) & GPIO_ICR2_ICR19_MASK)
#define GPIO_ICR2_ICR20_MASK                     (0x300U)
#define GPIO_ICR2_ICR20_SHIFT                    (8U)
/*! ICR20
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR2_ICR20(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ICR2_ICR20_SHIFT)) & GPIO_ICR2_ICR20_MASK)
#define GPIO_ICR2_ICR21_MASK                     (0xC00U)
#define GPIO_ICR2_ICR21_SHIFT                    (10U)
/*! ICR21
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR2_ICR21(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ICR2_ICR21_SHIFT)) & GPIO_ICR2_ICR21_MASK)
#define GPIO_ICR2_ICR22_MASK                     (0x3000U)
#define GPIO_ICR2_ICR22_SHIFT                    (12U)
/*! ICR22
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR2_ICR22(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ICR2_ICR22_SHIFT)) & GPIO_ICR2_ICR22_MASK)
#define GPIO_ICR2_ICR23_MASK                     (0xC000U)
#define GPIO_ICR2_ICR23_SHIFT                    (14U)
/*! ICR23
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR2_ICR23(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ICR2_ICR23_SHIFT)) & GPIO_ICR2_ICR23_MASK)
#define GPIO_ICR2_ICR24_MASK                     (0x30000U)
#define GPIO_ICR2_ICR24_SHIFT                    (16U)
/*! ICR24
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR2_ICR24(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ICR2_ICR24_SHIFT)) & GPIO_ICR2_ICR24_MASK)
#define GPIO_ICR2_ICR25_MASK                     (0xC0000U)
#define GPIO_ICR2_ICR25_SHIFT                    (18U)
/*! ICR25
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR2_ICR25(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ICR2_ICR25_SHIFT)) & GPIO_ICR2_ICR25_MASK)
#define GPIO_ICR2_ICR26_MASK                     (0x300000U)
#define GPIO_ICR2_ICR26_SHIFT                    (20U)
/*! ICR26
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR2_ICR26(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ICR2_ICR26_SHIFT)) & GPIO_ICR2_ICR26_MASK)
#define GPIO_ICR2_ICR27_MASK                     (0xC00000U)
#define GPIO_ICR2_ICR27_SHIFT                    (22U)
/*! ICR27
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR2_ICR27(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ICR2_ICR27_SHIFT)) & GPIO_ICR2_ICR27_MASK)
#define GPIO_ICR2_ICR28_MASK                     (0x3000000U)
#define GPIO_ICR2_ICR28_SHIFT                    (24U)
/*! ICR28
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR2_ICR28(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ICR2_ICR28_SHIFT)) & GPIO_ICR2_ICR28_MASK)
#define GPIO_ICR2_ICR29_MASK                     (0xC000000U)
#define GPIO_ICR2_ICR29_SHIFT                    (26U)
/*! ICR29
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR2_ICR29(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ICR2_ICR29_SHIFT)) & GPIO_ICR2_ICR29_MASK)
#define GPIO_ICR2_ICR30_MASK                     (0x30000000U)
#define GPIO_ICR2_ICR30_SHIFT                    (28U)
/*! ICR30
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR2_ICR30(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ICR2_ICR30_SHIFT)) & GPIO_ICR2_ICR30_MASK)
#define GPIO_ICR2_ICR31_MASK                     (0xC0000000U)
#define GPIO_ICR2_ICR31_SHIFT                    (30U)
/*! ICR31
 *  0b00..Interrupt n is low-level sensitive.
 *  0b01..Interrupt n is high-level sensitive.
 *  0b10..Interrupt n is rising-edge sensitive.
 *  0b11..Interrupt n is falling-edge sensitive.
 */
#define GPIO_ICR2_ICR31(x)                       (((uint32_t)(((uint32_t)(x)) << GPIO_ICR2_ICR31_SHIFT)) & GPIO_ICR2_ICR31_MASK)
/*! @} */

/*! @name IMR - GPIO interrupt mask register */
/*! @{ */
#define GPIO_IMR_IMR_MASK                        (0xFFFFFFFFU)
#define GPIO_IMR_IMR_SHIFT                       (0U)
/*! IMR
 *  0b00000000000000000000000000000000..Interrupt n is disabled.
 *  0b00000000000000000000000000000001..Interrupt n is enabled.
 */
#define GPIO_IMR_IMR(x)                          (((uint32_t)(((uint32_t)(x)) << GPIO_IMR_IMR_SHIFT)) & GPIO_IMR_IMR_MASK)
/*! @} */

/*! @name ISR - GPIO interrupt status register */
/*! @{ */
#define GPIO_ISR_ISR_MASK                        (0xFFFFFFFFU)
#define GPIO_ISR_ISR_SHIFT                       (0U)
#define GPIO_ISR_ISR(x)                          (((uint32_t)(((uint32_t)(x)) << GPIO_ISR_ISR_SHIFT)) & GPIO_ISR_ISR_MASK)
/*! @} */

/*! @name EDGE_SEL - GPIO edge select register */
/*! @{ */
#define GPIO_EDGE_SEL_GPIO_EDGE_SEL_MASK         (0xFFFFFFFFU)
#define GPIO_EDGE_SEL_GPIO_EDGE_SEL_SHIFT        (0U)
#define GPIO_EDGE_SEL_GPIO_EDGE_SEL(x)           (((uint32_t)(((uint32_t)(x)) << GPIO_EDGE_SEL_GPIO_EDGE_SEL_SHIFT)) & GPIO_EDGE_SEL_GPIO_EDGE_SEL_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group GPIO_Register_Masks */


/* GPIO - Peripheral instance base addresses */
/** Peripheral GPIO1 base address */
#define GPIO1_BASE                               SOC_REGS(0x30200000u)
/** Peripheral GPIO1 base pointer */
#define GPIO1                                    ((GPIO_Type *)GPIO1_BASE)
/** Peripheral GPIO2 base address */
#define GPIO2_BASE                               SOC_REGS(0x30210000u)
/** Peripheral GPIO2 base pointer */
#define GPIO2                                    ((GPIO_Type *)GPIO2_BASE)
/** Peripheral GPIO3 base address */
#define GPIO3_BASE                               SOC_REGS(0x30220000u)
/** Peripheral GPIO3 base pointer */
#define GPIO3                                    ((GPIO_Type *)GPIO3_BASE)
/** Peripheral GPIO4 base address */
#define GPIO4_BASE                               SOC_REGS(0x30230000u)
/** Peripheral GPIO4 base pointer */
#define GPIO4                                    ((GPIO_Type *)GPIO4_BASE)
/** Peripheral GPIO5 base address */
#define GPIO5_BASE                               SOC_REGS(0x30240000u)
/** Peripheral GPIO5 base pointer */
#define GPIO5                                    ((GPIO_Type *)GPIO5_BASE)
/** Array initializer of GPIO peripheral base addresses */
#define GPIO_BASE_ADDRS                          { 0u, GPIO1_BASE, GPIO2_BASE, GPIO3_BASE, GPIO4_BASE, GPIO5_BASE }
/** Array initializer of GPIO peripheral base pointers */
#define GPIO_BASE_PTRS                           { (GPIO_Type *)0u, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5 }
/** Interrupt vectors for the GPIO peripheral type */
#define GPIO_IRQS                                { NotAvail_IRQn, GPIO1_INT0_IRQn, GPIO1_INT1_IRQn, GPIO1_INT2_IRQn, GPIO1_INT3_IRQn, GPIO1_INT4_IRQn, GPIO1_INT5_IRQn, GPIO1_INT6_IRQn, GPIO1_INT7_IRQn, NotAvail_IRQn, NotAvail_IRQn, NotAvail_IRQn, NotAvail_IRQn }
#define GPIO_COMBINED_LOW_IRQS                   { NotAvail_IRQn, GPIO1_Combined_0_15_IRQn, GPIO2_Combined_0_15_IRQn, GPIO3_Combined_0_15_IRQn, GPIO4_Combined_0_15_IRQn, GPIO5_Combined_0_15_IRQn }
#define GPIO_COMBINED_HIGH_IRQS                  { NotAvail_IRQn, GPIO1_Combined_16_31_IRQn, GPIO2_Combined_16_31_IRQn, GPIO3_Combined_16_31_IRQn, GPIO4_Combined_16_31_IRQn, GPIO5_Combined_16_31_IRQn }

/*!
 * @}
 */ /* end of group GPIO_Peripheral_Access_Layer */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief GPIO driver version 2.0.1. */
//#define FSL_GPIO_DRIVER_VERSION (MAKE_VERSION(2, 0, 1))
/*@}*/

/*! @brief GPIO direction definition. */
typedef enum _gpio_pin_direction
{
    kGPIO_DigitalInput  = 0U, /*!< Set current pin as digital input.*/
    kGPIO_DigitalOutput = 1U, /*!< Set current pin as digital output.*/
} gpio_pin_direction_t;

/*! @brief GPIO interrupt mode definition. */
typedef enum _gpio_interrupt_mode
{
    kGPIO_NoIntmode = 0U, /*!< Set current pin general IO functionality.*/
    kGPIO_IntLowLevel = 1U, /*!< Set current pin interrupt is low-level sensitive.*/
    kGPIO_IntHighLevel = 2U, /*!< Set current pin interrupt is high-level sensitive.*/
    kGPIO_IntRisingEdge = 3U, /*!< Set current pin interrupt is rising-edge sensitive.*/
    kGPIO_IntFallingEdge = 4U, /*!< Set current pin interrupt is falling-edge sensitive.*/
    kGPIO_IntRisingOrFallingEdge = 5U, /*!< Enable the edge select bit to override the ICR register's configuration.*/
} gpio_interrupt_mode_t;

/*! @brief GPIO Init structure definition. */
typedef struct _gpio_pin_config
{
    gpio_pin_direction_t  direction;     /*!< Specifies the pin direction. */
    uint8_t outputLogic; /*!< Set a default output logic, which has no use in input */
    gpio_interrupt_mode_t interruptMode; /*!< Specifies the pin interrupt mode, a value of @ref gpio_interrupt_mode_t. */
} gpio_pin_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name GPIO Initialization and Configuration functions
 * @{
 */

/*!
 * @brief Initializes the GPIO peripheral according to the specified
 *        parameters in the initConfig.
 *
 * @param base GPIO base pointer.
 * @param pin Specifies the pin number
 * @param initConfig pointer to a @ref gpio_pin_config_t structure that
 *        contains the configuration information.
 */
void GPIO_PinInit(GPIO_Type* base, uint32_t pin, const gpio_pin_config_t* Config);
/*@}*/

/*!
 * @name GPIO Reads and Write Functions
 * @{
 */

/*!
 * @brief Sets the output level of the individual GPIO pin to logic 1 or 0.
 *
 * @param base GPIO base pointer.
 * @param pin GPIO port pin number.
 * @param output GPIOpin output logic level.
 *        - 0: corresponding pin output low-logic level.
 *        - 1: corresponding pin output high-logic level.
 */
void GPIO_PinWrite(GPIO_Type* base, uint32_t pin, uint8_t output);

/*!
 * @brief Sets the output level of the individual GPIO pin to logic 1 or 0.
 * @deprecated Do not use this function.  It has been superceded by @ref GPIO_PinWrite.
 */
static inline void GPIO_WritePinOutput(GPIO_Type* base, uint32_t pin, uint8_t output)
{
    GPIO_PinWrite(base, pin, output);
}

/*!
 * @brief Sets the output level of the multiple GPIO pins to the logic 1.
 *
 * @param base GPIO peripheral base pointer (GPIO1, GPIO2, GPIO3, and so on.)
 * @param mask GPIO pin number macro
 */
static inline void GPIO_PortSet(GPIO_Type* base, uint32_t mask)
{
    base->DR |= mask;
}

/*!
 * @brief Sets the output level of the multiple GPIO pins to the logic 1.
 * @deprecated Do not use this function.  It has been superceded by @ref GPIO_PortSet.
 */
static inline void GPIO_SetPinsOutput(GPIO_Type* base, uint32_t mask)
{
    GPIO_PortSet(base, mask);
}

/*!
 * @brief Sets the output level of the multiple GPIO pins to the logic 0.
 *
 * @param base GPIO peripheral base pointer (GPIO1, GPIO2, GPIO3, and so on.)
 * @param mask GPIO pin number macro
 */
static inline void GPIO_PortClear(GPIO_Type* base, uint32_t mask)
{
    base->DR &= ~mask;
}

/*!
 * @brief Sets the output level of the multiple GPIO pins to the logic 0.
 * @deprecated Do not use this function.  It has been superceded by @ref GPIO_PortClear.
 */
static inline void GPIO_ClearPinsOutput(GPIO_Type* base, uint32_t mask)
{
    GPIO_PortClear(base, mask);
}

/*!
 * @brief Reads the current input value of the GPIO port.
 *
 * @param base GPIO base pointer.
 * @param pin GPIO port pin number.
 * @retval GPIO port input value.
 */
static inline uint32_t GPIO_PinRead(GPIO_Type* base, uint32_t pin)
{
    assert(pin < 32);

    return (((base->DR) >> pin) & 0x1U);
}

/*!
 * @brief Reads the current input value of the GPIO port.
 * @deprecated Do not use this function.  It has been superceded by @ref GPIO_PinRead.
 */
static inline uint32_t GPIO_ReadPinInput(GPIO_Type* base, uint32_t pin)
{
    return GPIO_PinRead(base, pin);
}
/*@}*/

/*!
 * @name GPIO Reads Pad Status Functions
 * @{
 */

 /*!
 * @brief Reads the current GPIO pin pad status.
 *
 * @param base GPIO base pointer.
 * @param pin GPIO port pin number.
 * @retval GPIO pin pad status value.
 */
static inline uint8_t GPIO_PinReadPadStatus(GPIO_Type* base, uint32_t pin)
{
    assert(pin < 32);

    return (uint8_t)(((base->PSR) >> pin) & 0x1U);
}

 /*!
 * @brief Reads the current GPIO pin pad status.
 * @deprecated Do not use this function.  It has been superceded by @ref GPIO_PinReadPadStatus.
 */
static inline uint8_t GPIO_ReadPadStatus(GPIO_Type* base, uint32_t pin)
{
    return GPIO_PinReadPadStatus(base, pin);
}
/*@}*/

/*!
 * @name Interrupts and flags management functions
 * @{
 */

/*!
 * @brief Sets the current pin interrupt mode.
 *
 * @param base GPIO base pointer.
 * @param pin GPIO port pin number.
 * @param pininterruptMode pointer to a @ref gpio_interrupt_mode_t structure
 *        that contains the interrupt mode information.
 */
void GPIO_PinSetInterruptConfig(GPIO_Type* base, uint32_t pin, gpio_interrupt_mode_t pinInterruptMode);

/*!
 * @brief Sets the current pin interrupt mode.
 * @deprecated Do not use this function.  It has been superceded by @ref GPIO_PinSetInterruptConfig.
 */
static inline void GPIO_SetPinInterruptConfig(GPIO_Type* base, uint32_t pin, gpio_interrupt_mode_t pinInterruptMode)
{
    GPIO_PinSetInterruptConfig(base, pin, pinInterruptMode);
}

/*!
 * @brief Enables the specific pin interrupt.
 *
 * @param base GPIO base pointer.
 * @param mask GPIO pin number macro.
 */
static inline void GPIO_PortEnableInterrupts(GPIO_Type* base, uint32_t mask)
{
    base->IMR |= mask;
}

/*!
 * @brief Enables the specific pin interrupt.
 *
 * @param base GPIO base pointer.
 * @param mask GPIO pin number macro.
 */
static inline void GPIO_EnableInterrupts(GPIO_Type* base, uint32_t mask)
{
    GPIO_PortEnableInterrupts(base, mask);
}

/*!
 * @brief Disables the specific pin interrupt.
 *
 * @param base GPIO base pointer.
 * @param mask GPIO pin number macro.
 */
static inline void GPIO_PortDisableInterrupts(GPIO_Type* base, uint32_t mask)
{
    base->IMR &= ~mask;
}

/*!
 * @brief Disables the specific pin interrupt.
 * @deprecated Do not use this function.  It has been superceded by @ref GPIO_PortDisableInterrupts.
 */
static inline void GPIO_DisableInterrupts(GPIO_Type* base, uint32_t mask)
{
    GPIO_PortDisableInterrupts(base, mask);
}

/*!
 * @brief Reads individual pin interrupt status.
 *
 * @param base GPIO base pointer.
 * @retval current pin interrupt status flag.
 */
static inline uint32_t GPIO_PortGetInterruptFlags(GPIO_Type* base)
{
    return base->ISR;
}

/*!
 * @brief Reads individual pin interrupt status.
 *
 * @param base GPIO base pointer.
 * @retval current pin interrupt status flag.
 */
static inline uint32_t GPIO_GetPinsInterruptFlags(GPIO_Type* base)
{
    return GPIO_PortGetInterruptFlags(base);
}

/*!
 * @brief Clears pin interrupt flag. Status flags are cleared by
 *        writing a 1 to the corresponding bit position.
 *
 * @param base GPIO base pointer.
 * @param mask GPIO pin number macro.
 */
static inline void GPIO_PortClearInterruptFlags(GPIO_Type* base, uint32_t mask)
{
    base->ISR = mask;
}

/*!
 * @brief Clears pin interrupt flag. Status flags are cleared by
 *        writing a 1 to the corresponding bit position.
 *
 * @param base GPIO base pointer.
 * @param mask GPIO pin number macro.
 */
static inline void GPIO_ClearPinsInterruptFlags(GPIO_Type* base, uint32_t mask)
{
    GPIO_PortClearInterruptFlags(base, mask);
}
/*@}*/

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */



#endif /* __IMX_GPIO_HW */
