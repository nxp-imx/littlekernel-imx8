/*
 * Copyright (c) 2018-2020, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __PDM_H_
#define __PDM_H_

#include "fsl_common.h"
#include "fsl_clock.h"
/*!
 * @addtogroup pdm_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
#define FSL_PDM_DRIVER_VERSION (MAKE_VERSION(2, 0, 1)) /*!< Version 2.0.1 */
/*@}*/

/*! @brief PDM XFER QUEUE SIZE */
#define PDM_XFER_QUEUE_SIZE (4)

/* ----------------------------------------------------------------------------
   -- PDM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PDM_Peripheral_Access_Layer PDM Peripheral Access Layer
 * @{
 */

/* ----------------------------------------------------------------------------
   -- PDM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PDM_Register_Masks PDM Register Masks
 * @{
 */

/*! @name CTRL_1 - CTRL_1 */
/*! @{ */
#define PDM_CTRL_1_CH0EN_MASK                    (0x1U)
#define PDM_CTRL_1_CH0EN_SHIFT                   (0U)
#define PDM_CTRL_1_CH0EN(x)                      (((uint32_t)(((uint32_t)(x)) << PDM_CTRL_1_CH0EN_SHIFT)) & PDM_CTRL_1_CH0EN_MASK)
#define PDM_CTRL_1_CH1EN_MASK                    (0x2U)
#define PDM_CTRL_1_CH1EN_SHIFT                   (1U)
#define PDM_CTRL_1_CH1EN(x)                      (((uint32_t)(((uint32_t)(x)) << PDM_CTRL_1_CH1EN_SHIFT)) & PDM_CTRL_1_CH1EN_MASK)
#define PDM_CTRL_1_CH2EN_MASK                    (0x4U)
#define PDM_CTRL_1_CH2EN_SHIFT                   (2U)
#define PDM_CTRL_1_CH2EN(x)                      (((uint32_t)(((uint32_t)(x)) << PDM_CTRL_1_CH2EN_SHIFT)) & PDM_CTRL_1_CH2EN_MASK)
#define PDM_CTRL_1_CH3EN_MASK                    (0x8U)
#define PDM_CTRL_1_CH3EN_SHIFT                   (3U)
#define PDM_CTRL_1_CH3EN(x)                      (((uint32_t)(((uint32_t)(x)) << PDM_CTRL_1_CH3EN_SHIFT)) & PDM_CTRL_1_CH3EN_MASK)
#define PDM_CTRL_1_CH4EN_MASK                    (0x10U)
#define PDM_CTRL_1_CH4EN_SHIFT                   (4U)
#define PDM_CTRL_1_CH4EN(x)                      (((uint32_t)(((uint32_t)(x)) << PDM_CTRL_1_CH4EN_SHIFT)) & PDM_CTRL_1_CH4EN_MASK)
#define PDM_CTRL_1_CH5EN_MASK                    (0x20U)
#define PDM_CTRL_1_CH5EN_SHIFT                   (5U)
#define PDM_CTRL_1_CH5EN(x)                      (((uint32_t)(((uint32_t)(x)) << PDM_CTRL_1_CH5EN_SHIFT)) & PDM_CTRL_1_CH5EN_MASK)
#define PDM_CTRL_1_CH6EN_MASK                    (0x40U)
#define PDM_CTRL_1_CH6EN_SHIFT                   (6U)
#define PDM_CTRL_1_CH6EN(x)                      (((uint32_t)(((uint32_t)(x)) << PDM_CTRL_1_CH6EN_SHIFT)) & PDM_CTRL_1_CH6EN_MASK)
#define PDM_CTRL_1_CH7EN_MASK                    (0x80U)
#define PDM_CTRL_1_CH7EN_SHIFT                   (7U)
#define PDM_CTRL_1_CH7EN(x)                      (((uint32_t)(((uint32_t)(x)) << PDM_CTRL_1_CH7EN_SHIFT)) & PDM_CTRL_1_CH7EN_MASK)
#define PDM_CTRL_1_ERREN_MASK                    (0x800000U)
#define PDM_CTRL_1_ERREN_SHIFT                   (23U)
#define PDM_CTRL_1_ERREN(x)                      (((uint32_t)(((uint32_t)(x)) << PDM_CTRL_1_ERREN_SHIFT)) & PDM_CTRL_1_ERREN_MASK)
#define PDM_CTRL_1_DISEL_MASK                    (0x3000000U)
#define PDM_CTRL_1_DISEL_SHIFT                   (24U)
#define PDM_CTRL_1_DISEL(x)                      (((uint32_t)(((uint32_t)(x)) << PDM_CTRL_1_DISEL_SHIFT)) & PDM_CTRL_1_DISEL_MASK)
#define PDM_CTRL_1_DBGE_MASK                     (0x4000000U)
#define PDM_CTRL_1_DBGE_SHIFT                    (26U)
#define PDM_CTRL_1_DBGE(x)                       (((uint32_t)(((uint32_t)(x)) << PDM_CTRL_1_DBGE_SHIFT)) & PDM_CTRL_1_DBGE_MASK)
#define PDM_CTRL_1_SRES_MASK                     (0x8000000U)
#define PDM_CTRL_1_SRES_SHIFT                    (27U)
#define PDM_CTRL_1_SRES(x)                       (((uint32_t)(((uint32_t)(x)) << PDM_CTRL_1_SRES_SHIFT)) & PDM_CTRL_1_SRES_MASK)
#define PDM_CTRL_1_DBG_MASK                      (0x10000000U)
#define PDM_CTRL_1_DBG_SHIFT                     (28U)
#define PDM_CTRL_1_DBG(x)                        (((uint32_t)(((uint32_t)(x)) << PDM_CTRL_1_DBG_SHIFT)) & PDM_CTRL_1_DBG_MASK)
#define PDM_CTRL_1_PDMIEN_MASK                   (0x20000000U)
#define PDM_CTRL_1_PDMIEN_SHIFT                  (29U)
#define PDM_CTRL_1_PDMIEN(x)                     (((uint32_t)(((uint32_t)(x)) << PDM_CTRL_1_PDMIEN_SHIFT)) & PDM_CTRL_1_PDMIEN_MASK)
#define PDM_CTRL_1_DOZEN_MASK                    (0x40000000U)
#define PDM_CTRL_1_DOZEN_SHIFT                   (30U)
#define PDM_CTRL_1_DOZEN(x)                      (((uint32_t)(((uint32_t)(x)) << PDM_CTRL_1_DOZEN_SHIFT)) & PDM_CTRL_1_DOZEN_MASK)
#define PDM_CTRL_1_MDIS_MASK                     (0x80000000U)
#define PDM_CTRL_1_MDIS_SHIFT                    (31U)
#define PDM_CTRL_1_MDIS(x)                       (((uint32_t)(((uint32_t)(x)) << PDM_CTRL_1_MDIS_SHIFT)) & PDM_CTRL_1_MDIS_MASK)
/*! @} */

/*! @name CTRL_2 - CTRL_2 */
/*! @{ */
#define PDM_CTRL_2_CLKDIV_MASK                   (0xFFU)
#define PDM_CTRL_2_CLKDIV_SHIFT                  (0U)
#define PDM_CTRL_2_CLKDIV(x)                     (((uint32_t)(((uint32_t)(x)) << PDM_CTRL_2_CLKDIV_SHIFT)) & PDM_CTRL_2_CLKDIV_MASK)
#define PDM_CTRL_2_CICOSR_MASK                   (0xF0000U)
#define PDM_CTRL_2_CICOSR_SHIFT                  (16U)
#define PDM_CTRL_2_CICOSR(x)                     (((uint32_t)(((uint32_t)(x)) << PDM_CTRL_2_CICOSR_SHIFT)) & PDM_CTRL_2_CICOSR_MASK)
#define PDM_CTRL_2_QSEL_MASK                     (0xE000000U)
#define PDM_CTRL_2_QSEL_SHIFT                    (25U)
#define PDM_CTRL_2_QSEL(x)                       (((uint32_t)(((uint32_t)(x)) << PDM_CTRL_2_QSEL_SHIFT)) & PDM_CTRL_2_QSEL_MASK)
/*! @} */

/*! @name STAT - STAT */
/*! @{ */
#define PDM_STAT_CH0F_MASK                       (0x1U)
#define PDM_STAT_CH0F_SHIFT                      (0U)
#define PDM_STAT_CH0F(x)                         (((uint32_t)(((uint32_t)(x)) << PDM_STAT_CH0F_SHIFT)) & PDM_STAT_CH0F_MASK)
#define PDM_STAT_CH1F_MASK                       (0x2U)
#define PDM_STAT_CH1F_SHIFT                      (1U)
#define PDM_STAT_CH1F(x)                         (((uint32_t)(((uint32_t)(x)) << PDM_STAT_CH1F_SHIFT)) & PDM_STAT_CH1F_MASK)
#define PDM_STAT_CH2F_MASK                       (0x4U)
#define PDM_STAT_CH2F_SHIFT                      (2U)
#define PDM_STAT_CH2F(x)                         (((uint32_t)(((uint32_t)(x)) << PDM_STAT_CH2F_SHIFT)) & PDM_STAT_CH2F_MASK)
#define PDM_STAT_CH3F_MASK                       (0x8U)
#define PDM_STAT_CH3F_SHIFT                      (3U)
#define PDM_STAT_CH3F(x)                         (((uint32_t)(((uint32_t)(x)) << PDM_STAT_CH3F_SHIFT)) & PDM_STAT_CH3F_MASK)
#define PDM_STAT_CH4F_MASK                       (0x10U)
#define PDM_STAT_CH4F_SHIFT                      (4U)
#define PDM_STAT_CH4F(x)                         (((uint32_t)(((uint32_t)(x)) << PDM_STAT_CH4F_SHIFT)) & PDM_STAT_CH4F_MASK)
#define PDM_STAT_CH5F_MASK                       (0x20U)
#define PDM_STAT_CH5F_SHIFT                      (5U)
#define PDM_STAT_CH5F(x)                         (((uint32_t)(((uint32_t)(x)) << PDM_STAT_CH5F_SHIFT)) & PDM_STAT_CH5F_MASK)
#define PDM_STAT_CH6F_MASK                       (0x40U)
#define PDM_STAT_CH6F_SHIFT                      (6U)
#define PDM_STAT_CH6F(x)                         (((uint32_t)(((uint32_t)(x)) << PDM_STAT_CH6F_SHIFT)) & PDM_STAT_CH6F_MASK)
#define PDM_STAT_CH7F_MASK                       (0x80U)
#define PDM_STAT_CH7F_SHIFT                      (7U)
#define PDM_STAT_CH7F(x)                         (((uint32_t)(((uint32_t)(x)) << PDM_STAT_CH7F_SHIFT)) & PDM_STAT_CH7F_MASK)
#define PDM_STAT_LOWFREQF_MASK                   (0x20000000U)
#define PDM_STAT_LOWFREQF_SHIFT                  (29U)
#define PDM_STAT_LOWFREQF(x)                     (((uint32_t)(((uint32_t)(x)) << PDM_STAT_LOWFREQF_SHIFT)) & PDM_STAT_LOWFREQF_MASK)
#define PDM_STAT_FIR_RDY_MASK                    (0x40000000U)
#define PDM_STAT_FIR_RDY_SHIFT                   (30U)
#define PDM_STAT_FIR_RDY(x)                      (((uint32_t)(((uint32_t)(x)) << PDM_STAT_FIR_RDY_SHIFT)) & PDM_STAT_FIR_RDY_MASK)
#define PDM_STAT_BSY_FIL_MASK                    (0x80000000U)
#define PDM_STAT_BSY_FIL_SHIFT                   (31U)
#define PDM_STAT_BSY_FIL(x)                      (((uint32_t)(((uint32_t)(x)) << PDM_STAT_BSY_FIL_SHIFT)) & PDM_STAT_BSY_FIL_MASK)
/*! @} */

/*! @name FIFO_CTRL - FIFO_CTRL */
/*! @{ */
#define PDM_FIFO_CTRL_FIFOWMK_MASK               (0x7U)
#define PDM_FIFO_CTRL_FIFOWMK_SHIFT              (0U)
#define PDM_FIFO_CTRL_FIFOWMK(x)                 (((uint32_t)(((uint32_t)(x)) << PDM_FIFO_CTRL_FIFOWMK_SHIFT)) & PDM_FIFO_CTRL_FIFOWMK_MASK)
/*! @} */

/*! @name FIFO_STAT - FIFO_STAT */
/*! @{ */
#define PDM_FIFO_STAT_FIFOOVF0_MASK              (0x1U)
#define PDM_FIFO_STAT_FIFOOVF0_SHIFT             (0U)
#define PDM_FIFO_STAT_FIFOOVF0(x)                (((uint32_t)(((uint32_t)(x)) << PDM_FIFO_STAT_FIFOOVF0_SHIFT)) & PDM_FIFO_STAT_FIFOOVF0_MASK)
#define PDM_FIFO_STAT_FIFOOVF1_MASK              (0x2U)
#define PDM_FIFO_STAT_FIFOOVF1_SHIFT             (1U)
#define PDM_FIFO_STAT_FIFOOVF1(x)                (((uint32_t)(((uint32_t)(x)) << PDM_FIFO_STAT_FIFOOVF1_SHIFT)) & PDM_FIFO_STAT_FIFOOVF1_MASK)
#define PDM_FIFO_STAT_FIFOOVF2_MASK              (0x4U)
#define PDM_FIFO_STAT_FIFOOVF2_SHIFT             (2U)
#define PDM_FIFO_STAT_FIFOOVF2(x)                (((uint32_t)(((uint32_t)(x)) << PDM_FIFO_STAT_FIFOOVF2_SHIFT)) & PDM_FIFO_STAT_FIFOOVF2_MASK)
#define PDM_FIFO_STAT_FIFOOVF3_MASK              (0x8U)
#define PDM_FIFO_STAT_FIFOOVF3_SHIFT             (3U)
#define PDM_FIFO_STAT_FIFOOVF3(x)                (((uint32_t)(((uint32_t)(x)) << PDM_FIFO_STAT_FIFOOVF3_SHIFT)) & PDM_FIFO_STAT_FIFOOVF3_MASK)
#define PDM_FIFO_STAT_FIFOOVF4_MASK              (0x10U)
#define PDM_FIFO_STAT_FIFOOVF4_SHIFT             (4U)
#define PDM_FIFO_STAT_FIFOOVF4(x)                (((uint32_t)(((uint32_t)(x)) << PDM_FIFO_STAT_FIFOOVF4_SHIFT)) & PDM_FIFO_STAT_FIFOOVF4_MASK)
#define PDM_FIFO_STAT_FIFOOVF5_MASK              (0x20U)
#define PDM_FIFO_STAT_FIFOOVF5_SHIFT             (5U)
#define PDM_FIFO_STAT_FIFOOVF5(x)                (((uint32_t)(((uint32_t)(x)) << PDM_FIFO_STAT_FIFOOVF5_SHIFT)) & PDM_FIFO_STAT_FIFOOVF5_MASK)
#define PDM_FIFO_STAT_FIFOOVF6_MASK              (0x40U)
#define PDM_FIFO_STAT_FIFOOVF6_SHIFT             (6U)
#define PDM_FIFO_STAT_FIFOOVF6(x)                (((uint32_t)(((uint32_t)(x)) << PDM_FIFO_STAT_FIFOOVF6_SHIFT)) & PDM_FIFO_STAT_FIFOOVF6_MASK)
#define PDM_FIFO_STAT_FIFOOVF7_MASK              (0x80U)
#define PDM_FIFO_STAT_FIFOOVF7_SHIFT             (7U)
#define PDM_FIFO_STAT_FIFOOVF7(x)                (((uint32_t)(((uint32_t)(x)) << PDM_FIFO_STAT_FIFOOVF7_SHIFT)) & PDM_FIFO_STAT_FIFOOVF7_MASK)
#define PDM_FIFO_STAT_FIFOUND0_MASK              (0x100U)
#define PDM_FIFO_STAT_FIFOUND0_SHIFT             (8U)
#define PDM_FIFO_STAT_FIFOUND0(x)                (((uint32_t)(((uint32_t)(x)) << PDM_FIFO_STAT_FIFOUND0_SHIFT)) & PDM_FIFO_STAT_FIFOUND0_MASK)
#define PDM_FIFO_STAT_FIFOUND1_MASK              (0x200U)
#define PDM_FIFO_STAT_FIFOUND1_SHIFT             (9U)
#define PDM_FIFO_STAT_FIFOUND1(x)                (((uint32_t)(((uint32_t)(x)) << PDM_FIFO_STAT_FIFOUND1_SHIFT)) & PDM_FIFO_STAT_FIFOUND1_MASK)
#define PDM_FIFO_STAT_FIFOUND2_MASK              (0x400U)
#define PDM_FIFO_STAT_FIFOUND2_SHIFT             (10U)
#define PDM_FIFO_STAT_FIFOUND2(x)                (((uint32_t)(((uint32_t)(x)) << PDM_FIFO_STAT_FIFOUND2_SHIFT)) & PDM_FIFO_STAT_FIFOUND2_MASK)
#define PDM_FIFO_STAT_FIFOUND3_MASK              (0x800U)
#define PDM_FIFO_STAT_FIFOUND3_SHIFT             (11U)
#define PDM_FIFO_STAT_FIFOUND3(x)                (((uint32_t)(((uint32_t)(x)) << PDM_FIFO_STAT_FIFOUND3_SHIFT)) & PDM_FIFO_STAT_FIFOUND3_MASK)
#define PDM_FIFO_STAT_FIFOUND4_MASK              (0x1000U)
#define PDM_FIFO_STAT_FIFOUND4_SHIFT             (12U)
#define PDM_FIFO_STAT_FIFOUND4(x)                (((uint32_t)(((uint32_t)(x)) << PDM_FIFO_STAT_FIFOUND4_SHIFT)) & PDM_FIFO_STAT_FIFOUND4_MASK)
#define PDM_FIFO_STAT_FIFOUND5_MASK              (0x2000U)
#define PDM_FIFO_STAT_FIFOUND5_SHIFT             (13U)
#define PDM_FIFO_STAT_FIFOUND5(x)                (((uint32_t)(((uint32_t)(x)) << PDM_FIFO_STAT_FIFOUND5_SHIFT)) & PDM_FIFO_STAT_FIFOUND5_MASK)
#define PDM_FIFO_STAT_FIFOUND6_MASK              (0x4000U)
#define PDM_FIFO_STAT_FIFOUND6_SHIFT             (14U)
#define PDM_FIFO_STAT_FIFOUND6(x)                (((uint32_t)(((uint32_t)(x)) << PDM_FIFO_STAT_FIFOUND6_SHIFT)) & PDM_FIFO_STAT_FIFOUND6_MASK)
#define PDM_FIFO_STAT_FIFOUND7_MASK              (0x8000U)
#define PDM_FIFO_STAT_FIFOUND7_SHIFT             (15U)
#define PDM_FIFO_STAT_FIFOUND7(x)                (((uint32_t)(((uint32_t)(x)) << PDM_FIFO_STAT_FIFOUND7_SHIFT)) & PDM_FIFO_STAT_FIFOUND7_MASK)
/*! @} */

/*! @name DATACH - DATACH0..DATACH7 */
/*! @{ */
#define PDM_DATACH_DATA_MASK                     (0xFFFFU)
#define PDM_DATACH_DATA_SHIFT                    (0U)
#define PDM_DATACH_DATA(x)                       (((uint32_t)(((uint32_t)(x)) << PDM_DATACH_DATA_SHIFT)) & PDM_DATACH_DATA_MASK)
/*! @} */

/* The count of PDM_DATACH */
#define PDM_DATACH_COUNT                         (8U)

/*! @name DC_CTRL - DC_CTRL */
/*! @{ */
#define PDM_DC_CTRL_DCCONFIG0_MASK               (0x3U)
#define PDM_DC_CTRL_DCCONFIG0_SHIFT              (0U)
#define PDM_DC_CTRL_DCCONFIG0(x)                 (((uint32_t)(((uint32_t)(x)) << PDM_DC_CTRL_DCCONFIG0_SHIFT)) & PDM_DC_CTRL_DCCONFIG0_MASK)
#define PDM_DC_CTRL_DCCONFIG1_MASK               (0xCU)
#define PDM_DC_CTRL_DCCONFIG1_SHIFT              (2U)
#define PDM_DC_CTRL_DCCONFIG1(x)                 (((uint32_t)(((uint32_t)(x)) << PDM_DC_CTRL_DCCONFIG1_SHIFT)) & PDM_DC_CTRL_DCCONFIG1_MASK)
#define PDM_DC_CTRL_DCCONFIG2_MASK               (0x30U)
#define PDM_DC_CTRL_DCCONFIG2_SHIFT              (4U)
#define PDM_DC_CTRL_DCCONFIG2(x)                 (((uint32_t)(((uint32_t)(x)) << PDM_DC_CTRL_DCCONFIG2_SHIFT)) & PDM_DC_CTRL_DCCONFIG2_MASK)
#define PDM_DC_CTRL_DCCONFIG3_MASK               (0xC0U)
#define PDM_DC_CTRL_DCCONFIG3_SHIFT              (6U)
#define PDM_DC_CTRL_DCCONFIG3(x)                 (((uint32_t)(((uint32_t)(x)) << PDM_DC_CTRL_DCCONFIG3_SHIFT)) & PDM_DC_CTRL_DCCONFIG3_MASK)
#define PDM_DC_CTRL_DCCONFIG4_MASK               (0x300U)
#define PDM_DC_CTRL_DCCONFIG4_SHIFT              (8U)
#define PDM_DC_CTRL_DCCONFIG4(x)                 (((uint32_t)(((uint32_t)(x)) << PDM_DC_CTRL_DCCONFIG4_SHIFT)) & PDM_DC_CTRL_DCCONFIG4_MASK)
#define PDM_DC_CTRL_DCCONFIG5_MASK               (0xC00U)
#define PDM_DC_CTRL_DCCONFIG5_SHIFT              (10U)
#define PDM_DC_CTRL_DCCONFIG5(x)                 (((uint32_t)(((uint32_t)(x)) << PDM_DC_CTRL_DCCONFIG5_SHIFT)) & PDM_DC_CTRL_DCCONFIG5_MASK)
#define PDM_DC_CTRL_DCCONFIG6_MASK               (0x3000U)
#define PDM_DC_CTRL_DCCONFIG6_SHIFT              (12U)
#define PDM_DC_CTRL_DCCONFIG6(x)                 (((uint32_t)(((uint32_t)(x)) << PDM_DC_CTRL_DCCONFIG6_SHIFT)) & PDM_DC_CTRL_DCCONFIG6_MASK)
#define PDM_DC_CTRL_DCCONFIG7_MASK               (0xC000U)
#define PDM_DC_CTRL_DCCONFIG7_SHIFT              (14U)
#define PDM_DC_CTRL_DCCONFIG7(x)                 (((uint32_t)(((uint32_t)(x)) << PDM_DC_CTRL_DCCONFIG7_SHIFT)) & PDM_DC_CTRL_DCCONFIG7_MASK)
/*! @} */

/*! @name OUT_CTRL - OUT_CTRL */
/*! @{ */
#define PDM_OUT_CTRL_OUTGAIN0_MASK               (0xFU)
#define PDM_OUT_CTRL_OUTGAIN0_SHIFT              (0U)
#define PDM_OUT_CTRL_OUTGAIN0(x)                 (((uint32_t)(((uint32_t)(x)) << PDM_OUT_CTRL_OUTGAIN0_SHIFT)) & PDM_OUT_CTRL_OUTGAIN0_MASK)
#define PDM_OUT_CTRL_OUTGAIN1_MASK               (0xF0U)
#define PDM_OUT_CTRL_OUTGAIN1_SHIFT              (4U)
#define PDM_OUT_CTRL_OUTGAIN1(x)                 (((uint32_t)(((uint32_t)(x)) << PDM_OUT_CTRL_OUTGAIN1_SHIFT)) & PDM_OUT_CTRL_OUTGAIN1_MASK)
#define PDM_OUT_CTRL_OUTGAIN2_MASK               (0xF00U)
#define PDM_OUT_CTRL_OUTGAIN2_SHIFT              (8U)
#define PDM_OUT_CTRL_OUTGAIN2(x)                 (((uint32_t)(((uint32_t)(x)) << PDM_OUT_CTRL_OUTGAIN2_SHIFT)) & PDM_OUT_CTRL_OUTGAIN2_MASK)
#define PDM_OUT_CTRL_OUTGAIN3_MASK               (0xF000U)
#define PDM_OUT_CTRL_OUTGAIN3_SHIFT              (12U)
#define PDM_OUT_CTRL_OUTGAIN3(x)                 (((uint32_t)(((uint32_t)(x)) << PDM_OUT_CTRL_OUTGAIN3_SHIFT)) & PDM_OUT_CTRL_OUTGAIN3_MASK)
#define PDM_OUT_CTRL_OUTGAIN4_MASK               (0xF0000U)
#define PDM_OUT_CTRL_OUTGAIN4_SHIFT              (16U)
#define PDM_OUT_CTRL_OUTGAIN4(x)                 (((uint32_t)(((uint32_t)(x)) << PDM_OUT_CTRL_OUTGAIN4_SHIFT)) & PDM_OUT_CTRL_OUTGAIN4_MASK)
#define PDM_OUT_CTRL_OUTGAIN5_MASK               (0xF00000U)
#define PDM_OUT_CTRL_OUTGAIN5_SHIFT              (20U)
#define PDM_OUT_CTRL_OUTGAIN5(x)                 (((uint32_t)(((uint32_t)(x)) << PDM_OUT_CTRL_OUTGAIN5_SHIFT)) & PDM_OUT_CTRL_OUTGAIN5_MASK)
#define PDM_OUT_CTRL_OUTGAIN6_MASK               (0xF000000U)
#define PDM_OUT_CTRL_OUTGAIN6_SHIFT              (24U)
#define PDM_OUT_CTRL_OUTGAIN6(x)                 (((uint32_t)(((uint32_t)(x)) << PDM_OUT_CTRL_OUTGAIN6_SHIFT)) & PDM_OUT_CTRL_OUTGAIN6_MASK)
#define PDM_OUT_CTRL_OUTGAIN7_MASK               (0xF0000000U)
#define PDM_OUT_CTRL_OUTGAIN7_SHIFT              (28U)
#define PDM_OUT_CTRL_OUTGAIN7(x)                 (((uint32_t)(((uint32_t)(x)) << PDM_OUT_CTRL_OUTGAIN7_SHIFT)) & PDM_OUT_CTRL_OUTGAIN7_MASK)
/*! @} */

/*! @name OUT_STAT - OUT_STAT */
/*! @{ */
#define PDM_OUT_STAT_OUTOVF0_MASK                (0x1U)
#define PDM_OUT_STAT_OUTOVF0_SHIFT               (0U)
#define PDM_OUT_STAT_OUTOVF0(x)                  (((uint32_t)(((uint32_t)(x)) << PDM_OUT_STAT_OUTOVF0_SHIFT)) & PDM_OUT_STAT_OUTOVF0_MASK)
#define PDM_OUT_STAT_OUTOVF1_MASK                (0x2U)
#define PDM_OUT_STAT_OUTOVF1_SHIFT               (1U)
#define PDM_OUT_STAT_OUTOVF1(x)                  (((uint32_t)(((uint32_t)(x)) << PDM_OUT_STAT_OUTOVF1_SHIFT)) & PDM_OUT_STAT_OUTOVF1_MASK)
#define PDM_OUT_STAT_OUTOVF2_MASK                (0x4U)
#define PDM_OUT_STAT_OUTOVF2_SHIFT               (2U)
#define PDM_OUT_STAT_OUTOVF2(x)                  (((uint32_t)(((uint32_t)(x)) << PDM_OUT_STAT_OUTOVF2_SHIFT)) & PDM_OUT_STAT_OUTOVF2_MASK)
#define PDM_OUT_STAT_OUTOVF3_MASK                (0x8U)
#define PDM_OUT_STAT_OUTOVF3_SHIFT               (3U)
#define PDM_OUT_STAT_OUTOVF3(x)                  (((uint32_t)(((uint32_t)(x)) << PDM_OUT_STAT_OUTOVF3_SHIFT)) & PDM_OUT_STAT_OUTOVF3_MASK)
#define PDM_OUT_STAT_OUTOVF4_MASK                (0x10U)
#define PDM_OUT_STAT_OUTOVF4_SHIFT               (4U)
#define PDM_OUT_STAT_OUTOVF4(x)                  (((uint32_t)(((uint32_t)(x)) << PDM_OUT_STAT_OUTOVF4_SHIFT)) & PDM_OUT_STAT_OUTOVF4_MASK)
#define PDM_OUT_STAT_OUTOVF5_MASK                (0x20U)
#define PDM_OUT_STAT_OUTOVF5_SHIFT               (5U)
#define PDM_OUT_STAT_OUTOVF5(x)                  (((uint32_t)(((uint32_t)(x)) << PDM_OUT_STAT_OUTOVF5_SHIFT)) & PDM_OUT_STAT_OUTOVF5_MASK)
#define PDM_OUT_STAT_OUTOVF6_MASK                (0x40U)
#define PDM_OUT_STAT_OUTOVF6_SHIFT               (6U)
#define PDM_OUT_STAT_OUTOVF6(x)                  (((uint32_t)(((uint32_t)(x)) << PDM_OUT_STAT_OUTOVF6_SHIFT)) & PDM_OUT_STAT_OUTOVF6_MASK)
#define PDM_OUT_STAT_OUTOVF7_MASK                (0x80U)
#define PDM_OUT_STAT_OUTOVF7_SHIFT               (7U)
#define PDM_OUT_STAT_OUTOVF7(x)                  (((uint32_t)(((uint32_t)(x)) << PDM_OUT_STAT_OUTOVF7_SHIFT)) & PDM_OUT_STAT_OUTOVF7_MASK)
#define PDM_OUT_STAT_OUTUNF0_MASK                (0x10000U)
#define PDM_OUT_STAT_OUTUNF0_SHIFT               (16U)
#define PDM_OUT_STAT_OUTUNF0(x)                  (((uint32_t)(((uint32_t)(x)) << PDM_OUT_STAT_OUTUNF0_SHIFT)) & PDM_OUT_STAT_OUTUNF0_MASK)
#define PDM_OUT_STAT_OUTUNF1_MASK                (0x20000U)
#define PDM_OUT_STAT_OUTUNF1_SHIFT               (17U)
#define PDM_OUT_STAT_OUTUNF1(x)                  (((uint32_t)(((uint32_t)(x)) << PDM_OUT_STAT_OUTUNF1_SHIFT)) & PDM_OUT_STAT_OUTUNF1_MASK)
#define PDM_OUT_STAT_OUTUNF2_MASK                (0x40000U)
#define PDM_OUT_STAT_OUTUNF2_SHIFT               (18U)
#define PDM_OUT_STAT_OUTUNF2(x)                  (((uint32_t)(((uint32_t)(x)) << PDM_OUT_STAT_OUTUNF2_SHIFT)) & PDM_OUT_STAT_OUTUNF2_MASK)
#define PDM_OUT_STAT_OUTUNF3_MASK                (0x80000U)
#define PDM_OUT_STAT_OUTUNF3_SHIFT               (19U)
#define PDM_OUT_STAT_OUTUNF3(x)                  (((uint32_t)(((uint32_t)(x)) << PDM_OUT_STAT_OUTUNF3_SHIFT)) & PDM_OUT_STAT_OUTUNF3_MASK)
#define PDM_OUT_STAT_OUTUNF4_MASK                (0x100000U)
#define PDM_OUT_STAT_OUTUNF4_SHIFT               (20U)
#define PDM_OUT_STAT_OUTUNF4(x)                  (((uint32_t)(((uint32_t)(x)) << PDM_OUT_STAT_OUTUNF4_SHIFT)) & PDM_OUT_STAT_OUTUNF4_MASK)
#define PDM_OUT_STAT_OUTUNF5_MASK                (0x200000U)
#define PDM_OUT_STAT_OUTUNF5_SHIFT               (21U)
#define PDM_OUT_STAT_OUTUNF5(x)                  (((uint32_t)(((uint32_t)(x)) << PDM_OUT_STAT_OUTUNF5_SHIFT)) & PDM_OUT_STAT_OUTUNF5_MASK)
#define PDM_OUT_STAT_OUTUNF6_MASK                (0x400000U)
#define PDM_OUT_STAT_OUTUNF6_SHIFT               (22U)
#define PDM_OUT_STAT_OUTUNF6(x)                  (((uint32_t)(((uint32_t)(x)) << PDM_OUT_STAT_OUTUNF6_SHIFT)) & PDM_OUT_STAT_OUTUNF6_MASK)
#define PDM_OUT_STAT_OUTUNF7_MASK                (0x800000U)
#define PDM_OUT_STAT_OUTUNF7_SHIFT               (23U)
#define PDM_OUT_STAT_OUTUNF7(x)                  (((uint32_t)(((uint32_t)(x)) << PDM_OUT_STAT_OUTUNF7_SHIFT)) & PDM_OUT_STAT_OUTUNF7_MASK)
/*! @} */

/*! @name VAD0_CTRL_1 - VAD0_CTRL_1 */
/*! @{ */
#define PDM_VAD0_CTRL_1_VADEN_MASK               (0x1U)
#define PDM_VAD0_CTRL_1_VADEN_SHIFT              (0U)
#define PDM_VAD0_CTRL_1_VADEN(x)                 (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_CTRL_1_VADEN_SHIFT)) & PDM_VAD0_CTRL_1_VADEN_MASK)
#define PDM_VAD0_CTRL_1_VADRST_MASK              (0x2U)
#define PDM_VAD0_CTRL_1_VADRST_SHIFT             (1U)
#define PDM_VAD0_CTRL_1_VADRST(x)                (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_CTRL_1_VADRST_SHIFT)) & PDM_VAD0_CTRL_1_VADRST_MASK)
#define PDM_VAD0_CTRL_1_VADIE_MASK               (0x4U)
#define PDM_VAD0_CTRL_1_VADIE_SHIFT              (2U)
#define PDM_VAD0_CTRL_1_VADIE(x)                 (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_CTRL_1_VADIE_SHIFT)) & PDM_VAD0_CTRL_1_VADIE_MASK)
#define PDM_VAD0_CTRL_1_VADERIE_MASK             (0x8U)
#define PDM_VAD0_CTRL_1_VADERIE_SHIFT            (3U)
#define PDM_VAD0_CTRL_1_VADERIE(x)               (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_CTRL_1_VADERIE_SHIFT)) & PDM_VAD0_CTRL_1_VADERIE_MASK)
#define PDM_VAD0_CTRL_1_VADST10_MASK             (0x10U)
#define PDM_VAD0_CTRL_1_VADST10_SHIFT            (4U)
#define PDM_VAD0_CTRL_1_VADST10(x)               (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_CTRL_1_VADST10_SHIFT)) & PDM_VAD0_CTRL_1_VADST10_MASK)
#define PDM_VAD0_CTRL_1_VADINITT_MASK            (0x1F00U)
#define PDM_VAD0_CTRL_1_VADINITT_SHIFT           (8U)
#define PDM_VAD0_CTRL_1_VADINITT(x)              (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_CTRL_1_VADINITT_SHIFT)) & PDM_VAD0_CTRL_1_VADINITT_MASK)
#define PDM_VAD0_CTRL_1_VADCICOSR_MASK           (0xF0000U)
#define PDM_VAD0_CTRL_1_VADCICOSR_SHIFT          (16U)
#define PDM_VAD0_CTRL_1_VADCICOSR(x)             (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_CTRL_1_VADCICOSR_SHIFT)) & PDM_VAD0_CTRL_1_VADCICOSR_MASK)
#define PDM_VAD0_CTRL_1_VADCHSEL_MASK            (0x7000000U)
#define PDM_VAD0_CTRL_1_VADCHSEL_SHIFT           (24U)
#define PDM_VAD0_CTRL_1_VADCHSEL(x)              (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_CTRL_1_VADCHSEL_SHIFT)) & PDM_VAD0_CTRL_1_VADCHSEL_MASK)
/*! @} */

/*! @name VAD0_CTRL_2 - VAD0_CTRL_2 */
/*! @{ */
#define PDM_VAD0_CTRL_2_VADHPF_MASK              (0x3U)
#define PDM_VAD0_CTRL_2_VADHPF_SHIFT             (0U)
#define PDM_VAD0_CTRL_2_VADHPF(x)                (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_CTRL_2_VADHPF_SHIFT)) & PDM_VAD0_CTRL_2_VADHPF_MASK)
#define PDM_VAD0_CTRL_2_VADINPGAIN_MASK          (0xF00U)
#define PDM_VAD0_CTRL_2_VADINPGAIN_SHIFT         (8U)
#define PDM_VAD0_CTRL_2_VADINPGAIN(x)            (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_CTRL_2_VADINPGAIN_SHIFT)) & PDM_VAD0_CTRL_2_VADINPGAIN_MASK)
#define PDM_VAD0_CTRL_2_VADFRAMET_MASK           (0x3F0000U)
#define PDM_VAD0_CTRL_2_VADFRAMET_SHIFT          (16U)
#define PDM_VAD0_CTRL_2_VADFRAMET(x)             (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_CTRL_2_VADFRAMET_SHIFT)) & PDM_VAD0_CTRL_2_VADFRAMET_MASK)
#define PDM_VAD0_CTRL_2_VADFOUTDIS_MASK          (0x10000000U)
#define PDM_VAD0_CTRL_2_VADFOUTDIS_SHIFT         (28U)
#define PDM_VAD0_CTRL_2_VADFOUTDIS(x)            (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_CTRL_2_VADFOUTDIS_SHIFT)) & PDM_VAD0_CTRL_2_VADFOUTDIS_MASK)
#define PDM_VAD0_CTRL_2_VADPREFEN_MASK           (0x40000000U)
#define PDM_VAD0_CTRL_2_VADPREFEN_SHIFT          (30U)
#define PDM_VAD0_CTRL_2_VADPREFEN(x)             (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_CTRL_2_VADPREFEN_SHIFT)) & PDM_VAD0_CTRL_2_VADPREFEN_MASK)
#define PDM_VAD0_CTRL_2_VADFRENDIS_MASK          (0x80000000U)
#define PDM_VAD0_CTRL_2_VADFRENDIS_SHIFT         (31U)
#define PDM_VAD0_CTRL_2_VADFRENDIS(x)            (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_CTRL_2_VADFRENDIS_SHIFT)) & PDM_VAD0_CTRL_2_VADFRENDIS_MASK)
/*! @} */

/*! @name VAD0_STAT - VAD0_STAT */
/*! @{ */
#define PDM_VAD0_STAT_VADIF_MASK                 (0x1U)
#define PDM_VAD0_STAT_VADIF_SHIFT                (0U)
#define PDM_VAD0_STAT_VADIF(x)                   (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_STAT_VADIF_SHIFT)) & PDM_VAD0_STAT_VADIF_MASK)
#define PDM_VAD0_STAT_VADEF_MASK                 (0x8000U)
#define PDM_VAD0_STAT_VADEF_SHIFT                (15U)
#define PDM_VAD0_STAT_VADEF(x)                   (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_STAT_VADEF_SHIFT)) & PDM_VAD0_STAT_VADEF_MASK)
#define PDM_VAD0_STAT_VADINSATF_MASK             (0x10000U)
#define PDM_VAD0_STAT_VADINSATF_SHIFT            (16U)
#define PDM_VAD0_STAT_VADINSATF(x)               (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_STAT_VADINSATF_SHIFT)) & PDM_VAD0_STAT_VADINSATF_MASK)
#define PDM_VAD0_STAT_VADINITF_MASK              (0x80000000U)
#define PDM_VAD0_STAT_VADINITF_SHIFT             (31U)
#define PDM_VAD0_STAT_VADINITF(x)                (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_STAT_VADINITF_SHIFT)) & PDM_VAD0_STAT_VADINITF_MASK)
/*! @} */

/*! @name VAD0_SCONFIG - VAD0_SCONFIG */
/*! @{ */
#define PDM_VAD0_SCONFIG_VADSGAIN_MASK           (0xFU)
#define PDM_VAD0_SCONFIG_VADSGAIN_SHIFT          (0U)
#define PDM_VAD0_SCONFIG_VADSGAIN(x)             (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_SCONFIG_VADSGAIN_SHIFT)) & PDM_VAD0_SCONFIG_VADSGAIN_MASK)
#define PDM_VAD0_SCONFIG_VADSMAXEN_MASK          (0x40000000U)
#define PDM_VAD0_SCONFIG_VADSMAXEN_SHIFT         (30U)
#define PDM_VAD0_SCONFIG_VADSMAXEN(x)            (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_SCONFIG_VADSMAXEN_SHIFT)) & PDM_VAD0_SCONFIG_VADSMAXEN_MASK)
#define PDM_VAD0_SCONFIG_VADSFILEN_MASK          (0x80000000U)
#define PDM_VAD0_SCONFIG_VADSFILEN_SHIFT         (31U)
#define PDM_VAD0_SCONFIG_VADSFILEN(x)            (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_SCONFIG_VADSFILEN_SHIFT)) & PDM_VAD0_SCONFIG_VADSFILEN_MASK)
/*! @} */

/*! @name VAD0_NCONFIG - VAD0_NCONFIG */
/*! @{ */
#define PDM_VAD0_NCONFIG_VADNGAIN_MASK           (0xFU)
#define PDM_VAD0_NCONFIG_VADNGAIN_SHIFT          (0U)
#define PDM_VAD0_NCONFIG_VADNGAIN(x)             (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_NCONFIG_VADNGAIN_SHIFT)) & PDM_VAD0_NCONFIG_VADNGAIN_MASK)
#define PDM_VAD0_NCONFIG_VADNFILADJ_MASK         (0x1F00U)
#define PDM_VAD0_NCONFIG_VADNFILADJ_SHIFT        (8U)
#define PDM_VAD0_NCONFIG_VADNFILADJ(x)           (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_NCONFIG_VADNFILADJ_SHIFT)) & PDM_VAD0_NCONFIG_VADNFILADJ_MASK)
#define PDM_VAD0_NCONFIG_VADNOREN_MASK           (0x10000000U)
#define PDM_VAD0_NCONFIG_VADNOREN_SHIFT          (28U)
#define PDM_VAD0_NCONFIG_VADNOREN(x)             (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_NCONFIG_VADNOREN_SHIFT)) & PDM_VAD0_NCONFIG_VADNOREN_MASK)
#define PDM_VAD0_NCONFIG_VADNDECEN_MASK          (0x20000000U)
#define PDM_VAD0_NCONFIG_VADNDECEN_SHIFT         (29U)
#define PDM_VAD0_NCONFIG_VADNDECEN(x)            (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_NCONFIG_VADNDECEN_SHIFT)) & PDM_VAD0_NCONFIG_VADNDECEN_MASK)
#define PDM_VAD0_NCONFIG_VADNMINEN_MASK          (0x40000000U)
#define PDM_VAD0_NCONFIG_VADNMINEN_SHIFT         (30U)
#define PDM_VAD0_NCONFIG_VADNMINEN(x)            (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_NCONFIG_VADNMINEN_SHIFT)) & PDM_VAD0_NCONFIG_VADNMINEN_MASK)
#define PDM_VAD0_NCONFIG_VADNFILAUTO_MASK        (0x80000000U)
#define PDM_VAD0_NCONFIG_VADNFILAUTO_SHIFT       (31U)
#define PDM_VAD0_NCONFIG_VADNFILAUTO(x)          (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_NCONFIG_VADNFILAUTO_SHIFT)) & PDM_VAD0_NCONFIG_VADNFILAUTO_MASK)
/*! @} */

/*! @name VAD0_NDATA - VAD0_NDATA */
/*! @{ */
#define PDM_VAD0_NDATA_VADNDATA_MASK             (0xFFFFU)
#define PDM_VAD0_NDATA_VADNDATA_SHIFT            (0U)
#define PDM_VAD0_NDATA_VADNDATA(x)               (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_NDATA_VADNDATA_SHIFT)) & PDM_VAD0_NDATA_VADNDATA_MASK)
/*! @} */

/*! @name VAD0_ZCD - VAD0_ZCD */
/*! @{ */
#define PDM_VAD0_ZCD_VADZCDEN_MASK               (0x1U)
#define PDM_VAD0_ZCD_VADZCDEN_SHIFT              (0U)
#define PDM_VAD0_ZCD_VADZCDEN(x)                 (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_ZCD_VADZCDEN_SHIFT)) & PDM_VAD0_ZCD_VADZCDEN_MASK)
#define PDM_VAD0_ZCD_VADZCDAUTO_MASK             (0x4U)
#define PDM_VAD0_ZCD_VADZCDAUTO_SHIFT            (2U)
#define PDM_VAD0_ZCD_VADZCDAUTO(x)               (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_ZCD_VADZCDAUTO_SHIFT)) & PDM_VAD0_ZCD_VADZCDAUTO_MASK)
#define PDM_VAD0_ZCD_VADZCDAND_MASK              (0x10U)
#define PDM_VAD0_ZCD_VADZCDAND_SHIFT             (4U)
#define PDM_VAD0_ZCD_VADZCDAND(x)                (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_ZCD_VADZCDAND_SHIFT)) & PDM_VAD0_ZCD_VADZCDAND_MASK)
#define PDM_VAD0_ZCD_VADZCDADJ_MASK              (0xF00U)
#define PDM_VAD0_ZCD_VADZCDADJ_SHIFT             (8U)
#define PDM_VAD0_ZCD_VADZCDADJ(x)                (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_ZCD_VADZCDADJ_SHIFT)) & PDM_VAD0_ZCD_VADZCDADJ_MASK)
#define PDM_VAD0_ZCD_VADZCDTH_MASK               (0x3FF0000U)
#define PDM_VAD0_ZCD_VADZCDTH_SHIFT              (16U)
#define PDM_VAD0_ZCD_VADZCDTH(x)                 (((uint32_t)(((uint32_t)(x)) << PDM_VAD0_ZCD_VADZCDTH_SHIFT)) & PDM_VAD0_ZCD_VADZCDTH_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group PDM_Register_Masks */


/*!
 * @}
 */ /* end of group PDM_Peripheral_Access_Layer */

/*! @brief PDM return status*/
enum _pdm_status
{
    kStatus_PDM_Busy = MAKE_STATUS(kStatusGroup_MICFIL, 0),       /*!< PDM is busy. */
    kStatus_PDM_CLK_LOW = MAKE_STATUS(kStatusGroup_MICFIL, 1),    /*!< PDM clock frequency low */
    kStatus_PDM_FIFO_ERROR = MAKE_STATUS(kStatusGroup_MICFIL, 2), /*!< PDM FIFO underrun or overflow */
    kStatus_PDM_QueueFull = MAKE_STATUS(kStatusGroup_MICFIL, 3),  /*!< PDM FIFO underrun or overflow */
    kStatus_PDM_Idle = MAKE_STATUS(kStatusGroup_MICFIL, 4)        /*!< PDM is idle */
};

/*! @brief The PDM interrupt enable flag */
enum _pdm_interrupt_enable
{
    kPDM_ErrorInterruptEnable = PDM_CTRL_1_ERREN_MASK, /*!< PDM channel error interrupt enable. */
    kPDM_FIFOInterruptEnable = PDM_CTRL_1_DISEL(2U),   /*!< PDM channel FIFO interrupt */
};

/*! @brief The PDM status */
enum _pdm_internal_status
{
    kPDM_StatusDfBusyFlag = PDM_STAT_BSY_FIL_MASK,     /*!< Decimation filter is busy processing data */
    kPDM_StatusFIRFilterReady = PDM_STAT_FIR_RDY_MASK, /*!< FIR filter data is ready */
    kPDM_StatusFrequencyLow = PDM_STAT_LOWFREQF_MASK,  /*!< Mic app clock frequency not high enough */

    kPDM_StatusCh0FifoDataAvaliable = PDM_STAT_CH0F_MASK, /*!< channel 0 fifo data reached watermark level */
    kPDM_StatusCh1FifoDataAvaliable = PDM_STAT_CH1F_MASK, /*!< channel 1 fifo data reached watermark level */
    kPDM_StatusCh2FifoDataAvaliable = PDM_STAT_CH2F_MASK, /*!< channel 2 fifo data reached watermark level */
    kPDM_StatusCh3FifoDataAvaliable = PDM_STAT_CH3F_MASK, /*!< channel 3 fifo data reached watermark level */
    kPDM_StatusCh4FifoDataAvaliable = PDM_STAT_CH4F_MASK, /*!< channel 4 fifo data reached watermark level */
    kPDM_StatusCh5FifoDataAvaliable = PDM_STAT_CH5F_MASK, /*!< channel 5 fifo data reached watermark level */
    kPDM_StatusCh6FifoDataAvaliable = PDM_STAT_CH6F_MASK, /*!< channel 6 fifo data reached watermark level */
    kPDM_StatusCh7FifoDataAvaliable = PDM_STAT_CH7F_MASK, /*!< channel 7 fifo data reached watermark level */
};

/*! @brief PDM channel enable mask */
enum _pdm_channel_enable_mask
{
    kPDM_EnableChannel0 = PDM_STAT_CH0F_MASK, /*!< channgel 0 enable mask */
    kPDM_EnableChannel1 = PDM_STAT_CH1F_MASK, /*!< channgel 1 enable mask */
    kPDM_EnableChannel2 = PDM_STAT_CH2F_MASK, /*!< channgel 2 enable mask */
    kPDM_EnableChannel3 = PDM_STAT_CH3F_MASK, /*!< channgel 3 enable mask */
    kPDM_EnableChannel4 = PDM_STAT_CH4F_MASK, /*!< channgel 4 enable mask */
    kPDM_EnableChannel5 = PDM_STAT_CH5F_MASK, /*!< channgel 5 enable mask */
    kPDM_EnableChannel6 = PDM_STAT_CH6F_MASK, /*!< channgel 6 enable mask */
    kPDM_EnableChannel7 = PDM_STAT_CH7F_MASK, /*!< channgel 7 enable mask */

    kPDM_EnableChannelAll = kPDM_EnableChannel0 | kPDM_EnableChannel1 | kPDM_EnableChannel2 | kPDM_EnableChannel3 |
                            kPDM_EnableChannel4 | kPDM_EnableChannel5 | kPDM_EnableChannel6 | kPDM_EnableChannel7,
};

/*! @brief The PDM fifo status */
enum _pdm_fifo_status
{
    kPDM_FifoStatusUnderflowCh0 = PDM_FIFO_STAT_FIFOUND0_MASK, /*!< channel0 fifo status underflow */
    kPDM_FifoStatusUnderflowCh1 = PDM_FIFO_STAT_FIFOUND1_MASK, /*!< channel1 fifo status underflow */
    kPDM_FifoStatusUnderflowCh2 = PDM_FIFO_STAT_FIFOUND2_MASK, /*!< channel2 fifo status underflow */
    kPDM_FifoStatusUnderflowCh3 = PDM_FIFO_STAT_FIFOUND3_MASK, /*!< channel3 fifo status underflow */
    kPDM_FifoStatusUnderflowCh4 = PDM_FIFO_STAT_FIFOUND4_MASK, /*!< channel4 fifo status underflow */
    kPDM_FifoStatusUnderflowCh5 = PDM_FIFO_STAT_FIFOUND5_MASK, /*!< channel5 fifo status underflow */
    kPDM_FifoStatusUnderflowCh6 = PDM_FIFO_STAT_FIFOUND6_MASK, /*!< channel6 fifo status underflow */
    kPDM_FifoStatusUnderflowCh7 = PDM_FIFO_STAT_FIFOUND6_MASK, /*!< channel7 fifo status underflow */

    kPDM_FifoStatusOverflowCh0 = PDM_FIFO_STAT_FIFOOVF0_MASK, /*!< channel0 fifo status overflow */
    kPDM_FifoStatusOverflowCh1 = PDM_FIFO_STAT_FIFOOVF1_MASK, /*!< channel1 fifo status overflow */
    kPDM_FifoStatusOverflowCh2 = PDM_FIFO_STAT_FIFOOVF2_MASK, /*!< channel2 fifo status overflow */
    kPDM_FifoStatusOverflowCh3 = PDM_FIFO_STAT_FIFOOVF3_MASK, /*!< channel3 fifo status overflow */
    kPDM_FifoStatusOverflowCh4 = PDM_FIFO_STAT_FIFOOVF4_MASK, /*!< channel4 fifo status overflow */
    kPDM_FifoStatusOverflowCh5 = PDM_FIFO_STAT_FIFOOVF5_MASK, /*!< channel5 fifo status overflow */
    kPDM_FifoStatusOverflowCh6 = PDM_FIFO_STAT_FIFOOVF6_MASK, /*!< channel6 fifo status overflow */
    kPDM_FifoStatusOverflowCh7 = PDM_FIFO_STAT_FIFOOVF7_MASK, /*!< channel7 fifo status overflow */
};

/*! @brief The PDM output status */
enum _pdm_output_status
{
    kPDM_OutputStatusUnderFlowCh0 = PDM_OUT_STAT_OUTUNF0_MASK, /*!< channel0 output status underflow */
    kPDM_OutputStatusUnderFlowCh1 = PDM_OUT_STAT_OUTUNF1_MASK, /*!< channel1 output status underflow */
    kPDM_OutputStatusUnderFlowCh2 = PDM_OUT_STAT_OUTUNF2_MASK, /*!< channel2 output status underflow */
    kPDM_OutputStatusUnderFlowCh3 = PDM_OUT_STAT_OUTUNF3_MASK, /*!< channel3 output status underflow */
    kPDM_OutputStatusUnderFlowCh4 = PDM_OUT_STAT_OUTUNF4_MASK, /*!< channel4 output status underflow */
    kPDM_OutputStatusUnderFlowCh5 = PDM_OUT_STAT_OUTUNF5_MASK, /*!< channel5 output status underflow */
    kPDM_OutputStatusUnderFlowCh6 = PDM_OUT_STAT_OUTUNF6_MASK, /*!< channel6 output status underflow */
    kPDM_OutputStatusUnderFlowCh7 = PDM_OUT_STAT_OUTUNF7_MASK, /*!< channel7 output status underflow */

    kPDM_OutputStatusOverFlowCh0 = PDM_OUT_STAT_OUTOVF0_MASK, /*!< channel0 output status overflow */
    kPDM_OutputStatusOverFlowCh1 = PDM_OUT_STAT_OUTOVF1_MASK, /*!< channel1 output status overflow */
    kPDM_OutputStatusOverFlowCh2 = PDM_OUT_STAT_OUTOVF2_MASK, /*!< channel2 output status overflow */
    kPDM_OutputStatusOverFlowCh3 = PDM_OUT_STAT_OUTOVF3_MASK, /*!< channel3 output status overflow */
    kPDM_OutputStatusOverFlowCh4 = PDM_OUT_STAT_OUTOVF4_MASK, /*!< channel4 output status overflow */
    kPDM_OutputStatusOverFlowCh5 = PDM_OUT_STAT_OUTOVF5_MASK, /*!< channel5 output status overflow */
    kPDM_OutputStatusOverFlowCh6 = PDM_OUT_STAT_OUTOVF6_MASK, /*!< channel6 output status overflow */
    kPDM_OutputStatusOverFlowCh7 = PDM_OUT_STAT_OUTOVF7_MASK, /*!< channel7 output status overflow */
};

/*! @brief PDM DC remover configurations */
typedef enum _pdm_dc_remover
{
    kPDM_DcRemoverCutOff21Hz = 0U,  /*!< DC remover cut off 21HZ */
    kPDM_DcRemoverCutOff83Hz = 1U,  /*!< DC remover cut off 83HZ */
    kPDM_DcRemoverCutOff152Hz = 2U, /*!< DC remover cut off 152HZ */
    kPDM_DcRemoverBypass = 3U,      /*!< DC remover bypass */
} pdm_dc_remover_t;

/*! @brief PDM decimation filter quality mode */
typedef enum _pdm_df_quality_mode
{
    kPDM_QualityModeMedium = 0U,   /*!< quality mode memdium */
    kPDM_QualityModeHigh = 1U,     /*!< quality mode high */
    kPDM_QualityModeLow = 7U,      /*!< quality mode low */
    kPDM_QualityModeVeryLow0 = 6U, /*!< quality mode very low0 */
    kPDM_QualityModeVeryLow1 = 5U, /*!< quality mode very low1 */
    kPDM_QualityModeVeryLow2 = 4U, /*!< quality mode very low2 */
} pdm_df_quality_mode_t;

/*! @brief PDM  quality mode K factor */
enum _pdm_qulaity_mode_k_factor
{
    kPDM_QualityModeHighKFactor = 1U,     /*!< high quality mode K factor = 1 / 2 */
    kPDM_QualityModeMediumKFactor = 2U,   /*!< medium/very low0 quality mode K factor = 2 / 2 */
    kPDM_QualityModeLowKFactor = 4U,      /*!< low/very low1 quality mode K factor = 4 / 2 */
    kPDM_QualityModeVeryLow2KFactor = 8U, /*!< very low2 quality mode K factor = 8 / 2 */
};

/*! @brief PDM decimation filter output gain */
typedef enum _pdm_df_output_gain
{
    kPDM_DfOutputGain0 = 0U,    /*!< Decimation filter output gain 0 */
    kPDM_DfOutputGain1 = 1U,    /*!< Decimation filter output gain 1 */
    kPDM_DfOutputGain2 = 2U,    /*!< Decimation filter output gain 2 */
    kPDM_DfOutputGain3 = 3U,    /*!< Decimation filter output gain 3 */
    kPDM_DfOutputGain4 = 4U,    /*!< Decimation filter output gain 4 */
    kPDM_DfOutputGain5 = 5U,    /*!< Decimation filter output gain 5 */
    kPDM_DfOutputGain6 = 6U,    /*!< Decimation filter output gain 6 */
    kPDM_DfOutputGain7 = 7U,    /*!< Decimation filter output gain 7 */
    kPDM_DfOutputGain8 = 8U,    /*!< Decimation filter output gain 8 */
    kPDM_DfOutputGain9 = 9U,    /*!< Decimation filter output gain 9 */
    kPDM_DfOutputGain10 = 0xAU, /*!< Decimation filter output gain 10 */
    kPDM_DfOutputGain11 = 0xBU, /*!< Decimation filter output gain 11 */
    kPDM_DfOutputGain12 = 0xCU, /*!< Decimation filter output gain 12 */
    kPDM_DfOutputGain13 = 0xDU, /*!< Decimation filter output gain 13 */
    kPDM_DfOutputGain14 = 0xEU, /*!< Decimation filter output gain 14 */
    kPDM_DfOutputGain15 = 0xFU, /*!< Decimation filter output gain 15 */
} pdm_df_output_gain_t;

/*! @brief PDM channel configurations */
typedef struct _pdm_channel_config
{
    pdm_dc_remover_t cutOffFreq; /*!< DC remover cut off frequency */
    pdm_df_output_gain_t gain;   /*!< Decimation Filter Output Gain */
} pdm_channel_config_t;

/*! @brief PDM user configuration structure */
typedef struct _pdm_config
{
    bool
        enableDoze; /*!< This module will enter disable/low leakage mode if DOZEN is active with ipg_doze is asserted */
    uint8_t fifoWatermark;             /*!< Watermark value for FIFO */
    pdm_df_quality_mode_t qualityMode; /*!< Quality mode */
    uint8_t cicOverSampleRate;         /*!< CIC filter over sampling rate */
} pdm_config_t;

/*! @brief PDM voice activity detector interrupt type */
enum _pdm_hwvad_interrupt_enable
{
    kPDM_HwvadErrorInterruptEnable = PDM_VAD0_CTRL_1_VADERIE_MASK, /*!< PDM channel HWVAD error interrupt enable. */
    kPDM_HwvadInterruptEnable = PDM_VAD0_CTRL_1_VADIE_MASK,        /*!< PDM channel HWVAD interrupt */
};

/*! @brief The PDM hwvad interrupt status flag */
enum _pdm_hwvad_int_status
{
    kPDM_HwvadStatusInputSaturation = PDM_VAD0_STAT_VADINSATF_MASK, /*!< HWVAD saturation condition */
    kPDM_HwvadStatusVoiceDetectFlag = PDM_VAD0_STAT_VADIF_MASK,     /*!< HWVAD voice detect interrupt triggered */
};

/*! @brief High pass filter configure cut-off frequency*/
typedef enum _pdm_hwvad_hpf_config
{
    kPDM_HwvadHpfBypassed = 0x0U,         /*!< High-pass filter bypass */
    kPDM_HwvadHpfCutOffFreq1750Hz = 0x1U, /*!< High-pass filter cut off frequency 1750HZ */
    kPDM_HwvadHpfCutOffFreq215Hz = 0x2U,  /*!< High-pass filter cut off frequency 215HZ */
} pdm_hwvad_hpf_config_t;

/*! @brief HWVAD internal filter status */
typedef enum _pdm_hwvad_filter_status
{
    kPDM_HwvadInternalFilterNormalOperation = 0U,                   /*!< internal filter ready for normal operation */
    kPDM_HwvadInternalFilterInitial = PDM_VAD0_CTRL_1_VADST10_MASK, /*!< interla filter are initial */
} pdm_hwvad_filter_status_t;

/*! @brief PDM voice activity detector user configuration structure */
typedef struct _pdm_hwvad_config
{
    uint8_t channel;           /*!< Which channel uses voice activity detector */
    uint8_t initializeTime;    /*!< Number of frames or samples to initialize voice activity detector. */
    uint8_t cicOverSampleRate; /*!< CIC filter over sampling rate */

    uint8_t inputGain;                 /*!< Voice activity detector input gain */
    uint32_t frameTime;                /*!< Voice activity frame time */
    pdm_hwvad_hpf_config_t cutOffFreq; /*!< High pass filter cut off frequency */
    bool enableFrameEnergy;            /*!< If frame energy enabled, true means enable */
    bool enablePreFilter;              /*!< If pre-filter enabled */
} pdm_hwvad_config_t;

/*! @brief PDM voice activity detector noise filter user configuration structure */
typedef struct _pdm_hwvad_noise_filter
{
    bool enableAutoNoiseFilter;     /*!< If noise fileter automatically activated, true means enable */
    bool enableNoiseMin;            /*!< If Noise minimum block enabled, true means enabled */
    bool enableNoiseDecimation;     /*!< If enable noise input decimation */
    bool enableNoiseDetectOR;       /*!< Enables a OR logic in the output of minimum noise estimator block */
    uint32_t noiseFilterAdjustment; /*!< The adjustment value of the noise filter */
    uint32_t noiseGain;             /*!< Gain value for the noise energy or envelope estimated */
} pdm_hwvad_noise_filter_t;

/*! @brief PDM voice activity detector zero cross detector result */
typedef enum _pdm_hwvad_zcd_result
{
    kPDM_HwvadResultOREnergyBasedDetection =
        0U, /*!< zero cross detector result will be OR with energy based detection */
    kPDM_HwvadResultANDEnergyBasedDetection =
        1U, /*!< zero cross detector result will be AND with energy based detection */
} pdm_hwvad_zcd_result_t;

/*! @brief PDM voice activity detector zero cross detector configuration structure */
typedef struct _pdm_hwvad_zero_cross_detector
{
    bool enableAutoThreshold;      /*!< If ZCD auto-threshold enabled, true means enabled. */
    pdm_hwvad_zcd_result_t zcdAnd; /*!< Is ZCD result is AND'ed with energy-based detection, false means OR'ed */
    uint32_t threshold;            /*!< The adjustment value of the noise filter */
    uint32_t adjustmentThreshold;  /*!< Gain value for the noise energy or envelope estimated */
} pdm_hwvad_zero_cross_detector_t;

/*! @brief PDM SDMA transfer structure */
typedef struct _pdm_transfer
{
    volatile uint8_t *data;   /*!< Data start address to transfer. */
    volatile size_t dataSize; /*!< Total Transfer bytes size. */
} pdm_transfer_t;

/*! @brief PDM handle */
typedef struct _pdm_handle pdm_handle_t;

/*! @brief PDM transfer callback prototype */
typedef void (*pdm_transfer_callback_t)(PDM_Type *base, pdm_handle_t *handle, status_t status, void *userData);

/*! @brief PDM handle structure */
struct _pdm_handle
{
    uint32_t state;                   /*!< Transfer status */
    uint32_t irq_mask;                /*!< Interrupt mask */
    void *buf;                        /*!< ISR buffer */
    size_t buf_size;                  /*!< ISR buffer size */
    ssize_t xfer_remaining;           /*!< Remaining bytes of  current transfer */
    #define RX_CBUF_N_FIFO_WATERMARK 8
    size_t cbuf_watermark;          /*!< Circular buffer watermark */

#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
    pdm_transfer_callback_t callback; /*!< Callback function called at transfer event*/
    void *userData;                   /*!< Callback parameter passed to callback function*/

    pdm_transfer_t pdmQueue[PDM_XFER_QUEUE_SIZE]; /*!< Transfer queue storing queued transfer */
    size_t transferSize[PDM_XFER_QUEUE_SIZE];     /*!< Data bytes need to transfer */
    volatile uint8_t queueUser;                   /*!< Index for user to queue transfer */
    volatile uint8_t queueDriver;                 /*!< Index for driver to get the transfer data and size */
#endif
    uint8_t watermark;    /*!< Watermark value */
    uint8_t startChannel; /*!< end channel */
    uint8_t channelNums;  /*!< Enabled channel number */
    uint64_t ts_start;
    uint64_t ts_stop;
};

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /*_cplusplus*/

/*!
 * @name Initialization and deinitialization
 * @{
 */

/*!
 * @brief Initializes the PDM peripheral.
 *
 * Ungates the PDM clock, resets the module, and configures PDM with a configuration structure.
 * The configuration structure can be custom filled or set with default values by
 * PDM_GetDefaultConfig().
 *
 * @note  This API should be called at the beginning of the application to use
 * the PDM driver. Otherwise, accessing the PDM module can cause a hard fault
 * because the clock is not enabled.
 *
 * @param base PDM base pointer
 * @param config PDM configuration structure.
*/
void PDM_Init(PDM_Type *base, const pdm_config_t *config);

/*!
 * @brief De-initializes the PDM peripheral.
 *
 * This API gates the PDM clock. The PDM module can't operate unless PDM_Init
 * is called to enable the clock.
 *
 * @param base PDM base pointer
*/
void PDM_Deinit(PDM_Type *base);

/*!
 * @brief Resets the PDM module.
 *
 * @param base PDM base pointer
 */
static inline void PDM_Reset(PDM_Type *base)
{
    base->CTRL_1 |= PDM_CTRL_1_SRES_MASK;
}

/*!
 * @brief Enables/disables PDM interface.
 *
 * @param base PDM base pointer
 * @param enable True means PDM interface is enabled, false means PDM interface is disabled.
 */
static inline void PDM_Enable(PDM_Type *base, bool enable)
{
    if (enable)
    {
        base->CTRL_1 |= PDM_CTRL_1_PDMIEN_MASK;
    }
    else
    {
        base->CTRL_1 &= ~PDM_CTRL_1_PDMIEN_MASK;
    }
}

/*!
 * @brief Enables/disables DOZE.
 *
 * @param base PDM base pointer
 * @param enable True means the module will enter Disable/Low Leakage mode when ipg_doze is asserted, false means the
 * module will not enter Disable/Low Leakage mode when ipg_doze is asserted.
 */
static inline void PDM_EnableDoze(PDM_Type *base, bool enable)
{
    if (enable)
    {
        base->CTRL_1 |= PDM_CTRL_1_DOZEN_MASK;
    }
    else
    {
        base->CTRL_1 &= ~PDM_CTRL_1_DOZEN_MASK;
    }
}

/*!
 * @brief Enables/disables debug mode for PDM.
 * The PDM interface cannot enter debug mode once in Disable/Low Leakage or Low Power mode.
 * @param base PDM base pointer
 * @param enable True means PDM interface enter debug mode, false means PDM interface in normal mode.
 */
static inline void PDM_EnableDebugMode(PDM_Type *base, bool enable)
{
    if (enable)
    {
        base->CTRL_1 |= PDM_CTRL_1_DBG_MASK;
    }
    else
    {
        base->CTRL_1 &= ~PDM_CTRL_1_DBG_MASK;
    }
}

/*!
 * @brief Enables/disables PDM interface in debug mode.
 *
 * @param base PDM base pointer
 * @param enable True means PDM interface is enabled debug mode, false means PDM interface is disabled after
 * after completing the current frame in debug mode.
 */
static inline void PDM_EnableInDebugMode(PDM_Type *base, bool enable)
{
    if (enable)
    {
        base->CTRL_1 |= PDM_CTRL_1_DBGE_MASK;
    }
    else
    {
        base->CTRL_1 &= ~PDM_CTRL_1_DBGE_MASK;
    }
}

/*!
 * @brief Enables/disables PDM interface disable/Low Leakage mode.
 *
 * @param base PDM base pointer
 * @param enable True means PDM interface is in disable/low leakage mode, False means PDM interface is in normal mode.
 */
static inline void PDM_EnterLowLeakageMode(PDM_Type *base, bool enable)
{
    if (enable)
    {
        base->CTRL_1 |= PDM_CTRL_1_MDIS_MASK;
    }
    else
    {
        base->CTRL_1 &= ~PDM_CTRL_1_MDIS_MASK;
    }
}

/*!
 * @brief Enables/disables the PDM channel.
 *
 * @param base PDM base pointer
 * @param channel PDM channel number need to enable or disable.
 * @param enable True means enable PDM channel, false means disable.
 */
static inline void PDM_EnableChannel(PDM_Type *base, uint8_t channel, bool enable)
{
    if (enable)
    {
        base->CTRL_1 |= (1U << channel);
    }
    else
    {
        base->CTRL_1 &= ~(1U << channel);
    }
}

/*!
 * @brief PDM one channel configurations.
 *
 * @param base PDM base pointer
 * @param config PDM channel configurations.
 * @param channel channel number.
 * after completing the current frame in debug mode.
 */
void PDM_SetChannelConfig(PDM_Type *base, uint32_t channel, const pdm_channel_config_t *config);

/*!
 * @brief PDM set sample rate.
 *
 * @param base PDM base pointer
 * @param enableChannelMask PDM channel enable mask.
 * @param qualityMode quality mode.
 * @param osr cic oversample rate
 * @param clkDiv clock divider
 * after completing the current frame in debug mode.
 */
status_t PDM_SetSampleRate(
    PDM_Type *base, uint32_t enableChannelMask, pdm_df_quality_mode_t qualityMode, uint8_t osr, uint32_t clkDiv);

/*! @} */

/*!
 * @name Status
 * @{
 */

/*!
 * @brief Gets the PDM internal status flag.
 * Use the Status Mask in _pdm_internal_status to get the status value needed
 * @param base PDM base pointer
 * @return PDM status flag value.
 */
static inline uint32_t PDM_GetStatus(PDM_Type *base)
{
    return base->STAT;
}

/*!
 * @brief Gets the PDM FIFO status flag.
 * Use the Status Mask in _pdm_fifo_status to get the status value needed
 * @param base PDM base pointer
 * @return FIFO status.
 */
static inline uint32_t PDM_GetFifoStatus(PDM_Type *base)
{
    return base->FIFO_STAT;
}

/*!
 * @brief Gets the PDM output status flag.
 * Use the Status Mask in _pdm_output_status to get the status value needed
 * @param base PDM base pointer
 * @return output status.
 */
static inline uint32_t PDM_GetOutputStatus(PDM_Type *base)
{
    return base->OUT_STAT;
}

/*!
 * @brief Clears the PDM Tx status.
 *
 * @param base PDM base pointer
 * @param mask State mask. It can be a combination of the status between kPDM_StatusFrequencyLow and
 * kPDM_StatusCh7FifoDataAvaliable.
 */
static inline void PDM_ClearStatus(PDM_Type *base, uint32_t mask)
{
    base->STAT = mask;
}

/*!
 * @brief Clears the PDM Tx status.
 *
 * @param base PDM base pointer
 * @param mask State mask.It can be a combination of the status in _pdm_fifo_status.
 */
static inline void PDM_ClearFIFOStatus(PDM_Type *base, uint32_t mask)
{
    base->FIFO_STAT = mask;
}

/*!
 * @brief Clears the PDM output status.
 *
 * @param base PDM base pointer
 * @param mask State mask. It can be a combination of the status in _pdm_output_status.
 */
static inline void PDM_ClearOutputStatus(PDM_Type *base, uint32_t mask)
{
    base->OUT_STAT = mask;
}

/*! @} */

/*!
 * @name Interrupts
 * @{
 */

/*!
 * @brief Enables the PDM interrupt requests.
 *
 * @param base PDM base pointer
 * @param mask interrupt source
 *     The parameter can be a combination of the following sources if defined.
 *     @arg kPDM_ErrorInterruptEnable
 *     @arg kPDM_FIFOInterruptEnable
 */
void PDM_EnableInterrupts(PDM_Type *base, uint32_t mask);

/*!
 * @brief Disables the PDM interrupt requests.
 *
 * @param base PDM base pointer
 * @param mask interrupt source
 *     The parameter can be a combination of the following sources if defined.
 *     @arg kPDM_ErrorInterruptEnable
 *     @arg kPDM_FIFOInterruptEnable
 */
static inline void PDM_DisableInterrupts(PDM_Type *base, uint32_t mask)
{
    base->CTRL_1 &= ~mask;
}

/*! @} */

/*!
 * @name DMA Control
 * @{
 */

/*!
 * @brief Enables/disables the PDM DMA requests.
 *
 * @param base PDM base pointer
 * @param enable True means enable DMA, false means disable DMA.
 */
static inline void PDM_EnableDMA(PDM_Type *base, bool enable)
{
    if (enable)
    {
        base->CTRL_1 = (base->CTRL_1 & (~PDM_CTRL_1_DISEL_MASK)) | PDM_CTRL_1_DISEL(0x1U);
    }
    else
    {
        base->CTRL_1 &= ~PDM_CTRL_1_DISEL_MASK;
    }
}

/*!
 * @brief  Gets the PDM data register address.
 *
 * This API is used to provide a transfer address for the PDM DMA transfer configuration.
 *
 * @param base PDM base pointer.
 * @param channel Which data channel used.
 * @return data register address.
 */
static inline uintptr_t PDM_GetDataRegisterAddress(PDM_Type *base, uint32_t channel)
{
    return (uintptr_t)(&(base->DATACH)[channel]);
}

/*! @} */

/*!
 * @name Bus Operations
 * @{
 */

/*!
 * @brief Reads data from the PDM FIFO.
 *
 * @param base PDM base pointer.
 * @param channel Data channel used.
 * @return Data in PDM FIFO.
 */
static inline int16_t PDM_ReadData(PDM_Type *base, uint32_t channel)
{
    return (int16_t)(base->DATACH[channel]);
}

/*! @} */

/*!
 * @name Voice Activity Detector
 * @{
 */

/*!
 * @brief Configure voice activity detector.
 *
 * @param base PDM base pointer
 * @param config Voice activity detector configure structure pointer .
 */
void PDM_SetHwvadConfig(PDM_Type *base, const pdm_hwvad_config_t *config);

/*!
 * @brief PDM hwvad force output disable.
 *
 * @param base PDM base pointer
 * @param enable, true is output force disable, false is output not force.
 */
static inline void PDM_ForceHwvadOutputDisable(PDM_Type *base, bool enable)
{
    if (enable)
    {
        base->VAD0_CTRL_2 &= ~PDM_VAD0_CTRL_2_VADFOUTDIS_MASK;
    }
    else
    {
        base->VAD0_CTRL_2 |= PDM_VAD0_CTRL_2_VADFOUTDIS_MASK;
    }
}

/*!
 * @brief PDM hwvad reset.
 * It will reset VADNDATA register and will clean all internal buffers, should be called when the PDM isn't running.
 *
 * @param base PDM base pointer
 */
static inline void PDM_ResetHwvad(PDM_Type *base)
{
    base->VAD0_CTRL_1 |= PDM_VAD0_CTRL_1_VADRST_MASK;
}
/*!
 * @brief Enable/Disable Voice activity detector.
 * Should be called when the PDM isn't running.
 * @param base PDM base pointer.
 * @param enable True means enable voice activity detector, false means disable.
 */
static inline void PDM_EnableHwvad(PDM_Type *base, bool enable)
{
    if (enable)
    {
        base->VAD0_CTRL_1 |= PDM_VAD0_CTRL_1_VADEN_MASK;
    }
    else
    {
        base->VAD0_CTRL_1 &= ~PDM_VAD0_CTRL_1_VADEN_MASK;
    }
}

/*!
 * @brief Enables the PDM Voice Detector interrupt requests.
 *
 * @param base PDM base pointer
 * @param mask interrupt source
 *     The parameter can be a combination of the following sources if defined.
 *     @arg kPDM_HWVADErrorInterruptEnable
 *     @arg kPDM_HWVADInterruptEnable
 */
static inline void PDM_EnableHwvadInterrupts(PDM_Type *base, uint32_t mask)
{
    base->VAD0_CTRL_1 |= mask;
}

/*!
 * @brief Disables the PDM Voice Detector interrupt requests.
 *
 * @param base PDM base pointer
 * @param mask interrupt source
 *     The parameter can be a combination of the following sources if defined.
 *     @arg kPDM_HWVADErrorInterruptEnable
 *     @arg kPDM_HWVADInterruptEnable
 */
static inline void PDM_DisableHwvadInterrupts(PDM_Type *base, uint32_t mask)
{
    base->VAD0_CTRL_1 &= ~mask;
}

/*!
 * @brief Clears the PDM voice activity detector status flags.
 *
 * @param base PDM base pointer
 * @param mask State mask,reference _pdm_hwvad_int_status.
 */
static inline void PDM_ClearHwvadInterruptStatusFlags(PDM_Type *base, uint32_t mask)
{
    base->VAD0_STAT = mask;
}

/*!
 * @brief Clears the PDM voice activity detector status flags.
 *
 * @param base PDM base pointer
 * @return status, reference _pdm_hwvad_int_status
 */
static inline uint32_t PDM_GetHwvadInterruptStatusFlags(PDM_Type *base)
{
    return base->VAD0_STAT & (PDM_VAD0_STAT_VADIF_MASK | PDM_VAD0_STAT_VADINSATF_MASK);
}

/*!
 * @brief Get the PDM voice activity detector initial flags.
 *
 * @param base PDM base pointer
 * @return initial flag.
 */
static inline uint32_t PDM_GetHwvadInitialFlag(PDM_Type *base)
{
    return base->VAD0_STAT & PDM_VAD0_STAT_VADINITF_MASK;
}

/*!
 * @brief Get the PDM voice activity detector voice detected flags.
 * NOte: this flag is auto cleared when voice gone.
 * @param base PDM base pointer
 * @return voice detected flag.
 */
static inline uint32_t PDM_GetHwvadVoiceDetectedFlag(PDM_Type *base)
{
    return base->VAD0_STAT & PDM_VAD0_STAT_VADEF_MASK;
}

/*!
 * @brief Enables/disables voice activity detector signal filter.
 *
 * @param base PDM base pointer
 * @param enable True means enable signal filter, false means disable.
 */
static inline void PDM_EnableHwvadSignalFilter(PDM_Type *base, bool enable)
{
    if (enable)
    {
        base->VAD0_SCONFIG |= PDM_VAD0_SCONFIG_VADSFILEN_MASK;
    }
    else
    {
        base->VAD0_SCONFIG &= ~PDM_VAD0_SCONFIG_VADSFILEN_MASK;
    }
}

/*!
 * @brief Configure voice activity detector signal filter.
 *
 * @param base PDM base pointer
 * @param enableMaxBlock If signal maximum block enabled.
 * @param signalGain Gain value for the signal energy.
 */
void PDM_SetHwvadSignalFilterConfig(PDM_Type *base, bool enableMaxBlock, uint32_t signalGain);

/*!
 * @brief Configure voice activity detector noise filter.
 *
 * @param base PDM base pointer
 * @param config Voice activity detector noise filter configure structure pointer .
 */
void PDM_SetHwvadNoiseFilterConfig(PDM_Type *base, const pdm_hwvad_noise_filter_t *config);

/*!
 * @brief Enables/disables voice activity detector zero cross detector.
 *
 * @param base PDM base pointer
 * @param enable True means enable zero cross detector, false means disable.
 */
static inline void PDM_EnableHwvadZeroCrossDetector(PDM_Type *base, bool enable)
{
    if (enable)
    {
        base->VAD0_ZCD |= PDM_VAD0_ZCD_VADZCDEN_MASK;
    }
    else
    {
        base->VAD0_ZCD &= ~PDM_VAD0_ZCD_VADZCDEN_MASK;
    }
}

/*!
 * @brief Configure voice activity detector zero cross detector.
 *
 * @param base PDM base pointer
 * @param config Voice activity detector zero cross detector configure structure pointer .
 */
void PDM_SetHwvadZeroCrossDetectorConfig(PDM_Type *base, const pdm_hwvad_zero_cross_detector_t *config);

/*!
 * @brief Reads noise data.
 *
 * @param base PDM base pointer.
 * @return Data in PDM noise data register.
 */
static inline uint16_t PDM_GetNoiseData(PDM_Type *base)
{
    return base->VAD0_NDATA;
}

/*!
 * @brief set hwvad internal filter status .
 * Note: filter initial status should be asserted for two more cycles, then set it to normal operation.
 * @param base PDM base pointer.
 * @param status internal filter status.
 */
static inline void PDM_SetHwvadInternalFilterStatus(PDM_Type *base, pdm_hwvad_filter_status_t status)
{
    base->VAD0_CTRL_1 = (base->VAD0_CTRL_1 & (~PDM_VAD0_CTRL_1_VADST10_MASK)) | status;
}

/*! @} */

/*!
 * @name Transactional
 * @{
 */

/*!
 * @brief Initializes the PDM handle.
 *
 * This function initializes the handle for the PDM transactional APIs. Call
 * this function once to get the handle initialized.
 *
 * @param base PDM base pointer.
 * @param handle PDM handle pointer.
 * @param callback Pointer to the user callback function.
 * @param userData User parameter passed to the callback function.
 */
void PDM_TransferCreateHandle(PDM_Type *base, pdm_handle_t *handle, pdm_transfer_callback_t callback, void *userData);

/*!
 * @brief PDM read data non blocking.
 * So the actually read data byte size in this function is (size * 2 * channelNums).
 * @param base PDM base pointer.
 * @param startChannel start channel number.
 * @param channelNums total enabled channelnums.
 * @param buffer received buffer address.
 * @param size number of 16bit data to read.
 */
void PDM_ReadNonBlocking(PDM_Type *base, uint32_t startChannel, uint32_t channelNums, int16_t *buffer, size_t size);

/*!
 * @brief Performs an interrupt non-blocking receive transfer on PDM.
 *
 * @note This API returns immediately after the transfer initiates.
 * Call the PDM_RxGetTransferStatusIRQ to poll the transfer status and check whether
 * the transfer is finished. If the return status is not kStatus_PDM_Busy, the transfer
 * is finished.
 *
 * @param base PDM base pointer
 * @param handle Pointer to the pdm_handle_t structure which stores the transfer state.
 * @param xfer Pointer to the pdm_transfer_t structure.
 * @retval kStatus_Success Successfully started the data receive.
 * @retval kStatus_PDM_Busy Previous receive still not finished.
 */
status_t PDM_TransferReceiveNonBlocking(PDM_Type *base, pdm_handle_t *handle, pdm_transfer_t *xfer);

/*!
 * @brief Aborts the current IRQ receive.
 *
 * @note This API can be called when an interrupt non-blocking transfer initiates
 * to abort the transfer early.
 *
 * @param base PDM base pointer
 * @param handle Pointer to the pdm_handle_t structure which stores the transfer state.
 */
void PDM_TransferAbortReceive(PDM_Type *base, pdm_handle_t *handle);

/*!
 * @brief Tx interrupt handler.
 *
 * @param base PDM base pointer.
 * @param handle Pointer to the pdm_handle_t structure.
 */
void PDM_TransferHandleIRQ(PDM_Type *base, pdm_handle_t *handle);

/*!
 * @brief Dumps PDM register.
 *
 * @param base PDM base pointer.
 */
void PDM_Dump(PDM_Type *base);

/*! @} */

#if defined(__cplusplus)
}
#endif /*_cplusplus*/

/*! @} */

#endif /* _FSL_PDM_H_ */
