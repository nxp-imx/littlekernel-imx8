/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017, 2020 NXP
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

#ifndef __SAI_H_
#define __SAI_H_

/*!
 * @addtogroup sai
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#include "fsl_common.h"

#define FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL true
#define FSL_SDK_DISABLE_IRQ true


/* SAI module features */

/* @brief Receive/transmit FIFO size in item count (register bit fields TCSR[FRDE], TCSR[FRIE], TCSR[FRF], TCR1[TFW], RCSR[FRDE], RCSR[FRIE], RCSR[FRF], RCR1[RFW], registers TFRn, RFRn). */
#define FSL_FEATURE_SAI_FIFO_COUNT (128)
/* @brief Receive/transmit channel number (register bit fields TCR3[TCE], RCR3[RCE], registers TDRn and RDRn). */
#define FSL_FEATURE_SAI_CHANNEL_COUNT (8)
/* @brief Maximum words per frame (register bit fields TCR3[WDFL], TCR4[FRSZ], TMR[TWM], RCR3[WDFL], RCR4[FRSZ], RMR[RWM]). */
#define FSL_FEATURE_SAI_MAX_WORDS_PER_FRAME (32)
/* @brief Has support of combining multiple data channel FIFOs into single channel FIFO (register bit fields TCR3[CFR], TCR4[FCOMB], TFR0[WCP], TFR1[WCP], RCR3[CFR], RCR4[FCOMB], RFR0[RCP], RFR1[RCP]). */
#define FSL_FEATURE_SAI_HAS_FIFO_COMBINE_MODE (1)
/* @brief Has packing of 8-bit and 16-bit data into each 32-bit FIFO word (register bit fields TCR4[FPACK], RCR4[FPACK]). */
#define FSL_FEATURE_SAI_HAS_FIFO_PACKING (1)
/* @brief Configures when the SAI will continue transmitting after a FIFO error has been detected (register bit fields TCR4[FCONT], RCR4[FCONT]). */
#define FSL_FEATURE_SAI_HAS_FIFO_FUNCTION_AFTER_ERROR (1)
/* @brief Configures if the frame sync is generated internally, a frame sync is only generated when the FIFO warning flag is clear or continuously (register bit fields TCR4[ONDEM], RCR4[ONDEM]). */
#define FSL_FEATURE_SAI_HAS_ON_DEMAND_MODE (1)
/* @brief Simplified bit clock source and asynchronous/synchronous mode selection (register bit fields TCR2[CLKMODE], RCR2[CLKMODE]), in comparison with the exclusively implemented TCR2[SYNC,BCS,BCI,MSEL], RCR2[SYNC,BCS,BCI,MSEL]. */
#define FSL_FEATURE_SAI_HAS_CLOCKING_MODE (0)
/* @brief Has register for configuration of the MCLK divide ratio (register bit fields MDR[FRACT], MDR[DIVIDE]). */
#define FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER (0)
/* @brief Interrupt source number */
#define FSL_FEATURE_SAI_INT_SOURCE_NUM (1)

#if defined SOC_IMX8MQ
/* IMX8MQ */
/* @brief Has register of MCR. */
#define FSL_FEATURE_SAI_HAS_MCR (0)
#elif defined SOC_IMX8MM
/* IMX8MM */
#define FSL_FEATURE_SAI_HAS_MCR (1)
/* @brief Has bit field MICS of the MCR register. */
#define FSL_FEATURE_SAI_HAS_NO_MCR_MICS (1)
#elif defined SOC_IMX8MN
/* IMX8MN */
#include "MIMX8MN6_ca53_features.h"
#else
#error "SOC not supported by I2S driver"
#endif

/* @brief Has register of MDR */
#define FSL_FEATURE_SAI_HAS_MDR (0)


/*! @name Driver version */
/*@{*/
/*@}*/
/* ----------------------------------------------------------------------------
   -- I2S Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2S_Peripheral_Access_Layer I2S Peripheral Access Layer
 * @{
 */

/** I2S - Register Layout Typedef */
typedef struct {
    __I  uint32_t VERID;                             /**< VERID, offset: 0x0 */
    __I  uint32_t PARAM;                             /**< PARAM, offset: 0x4 */
    __IO uint32_t TCSR;                              /**< TCSR, offset: 0x8 */
    __IO uint32_t TCR1;                              /**< TCR1, offset: 0xC */
    __IO uint32_t TCR2;                              /**< TCR2, offset: 0x10 */
    __IO uint32_t TCR3;                              /**< TCR3, offset: 0x14 */
    __IO uint32_t TCR4;                              /**< TCR4, offset: 0x18 */
    __IO uint32_t TCR5;                              /**< TCR5, offset: 0x1C */
    __O  uint32_t TDR[8];                            /**< TDR0..TDR7, array offset: 0x20, array step: 0x4 */
    __I  uint32_t TFR[8];                            /**< TFR0..TFR7, array offset: 0x40, array step: 0x4 */
    __IO uint32_t TMR;                               /**< TMR, offset: 0x60 */
    uint8_t RESERVED_0[12];
    __IO uint32_t TTCR;                              /**< TTCR, offset: 0x70 */
    __I  uint32_t TTSR;                              /**< TTSR, offset: 0x74 */
    __I  uint32_t TBCR;                              /**< TBCR, offset: 0x78 */
    __I  uint32_t TBCTR;                             /**< TBCTR, offset: 0x7C */
    uint8_t RESERVED_1[8];
    __IO uint32_t RCSR;                              /**< RCSR, offset: 0x88 */
    __IO uint32_t RCR1;                              /**< RCR1, offset: 0x8C */
    __IO uint32_t RCR2;                              /**< RCR2, offset: 0x90 */
    __IO uint32_t RCR3;                              /**< RCR3, offset: 0x94 */
    __IO uint32_t RCR4;                              /**< RCR4, offset: 0x98 */
    __IO uint32_t RCR5;                              /**< RCR5, offset: 0x9C */
    __I  uint32_t RDR[8];                            /**< RDR0..RDR7, array offset: 0xA0, array step: 0x4 */
    __I  uint32_t RFR[8];                            /**< RFR0..RFR7, array offset: 0xC0, array step: 0x4 */
    __IO uint32_t RMR;                               /**< RMR, offset: 0xE0 */
    uint8_t RESERVED_2[12];
    __IO uint32_t RTCR;                              /**< RTCR, offset: 0xF0 */
    __I  uint32_t RTSR;                              /**< RTSR, offset: 0xF4 */
    __I  uint32_t RBCR;                              /**< RBCR, offset: 0xF8 */
    __I  uint32_t RBCTR;                             /**< RBCTR, offset: 0xFC */
    __IO uint32_t MCR;                               /**< MCR, offset: 0x100 */
} I2S_Type;

/* ----------------------------------------------------------------------------
   -- I2S Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2S_Register_Masks I2S Register Masks
 * @{
 */

/*! @name VERID - VERID */
/*! @{ */
#define I2S_VERID_FEATURE_MASK                   (0xFFFFU)
#define I2S_VERID_FEATURE_SHIFT                  (0U)
#define I2S_VERID_FEATURE(x)                     (((uint32_t)(((uint32_t)(x)) << I2S_VERID_FEATURE_SHIFT)) & I2S_VERID_FEATURE_MASK)
#define I2S_VERID_MINOR_MASK                     (0xFF0000U)
#define I2S_VERID_MINOR_SHIFT                    (16U)
#define I2S_VERID_MINOR(x)                       (((uint32_t)(((uint32_t)(x)) << I2S_VERID_MINOR_SHIFT)) & I2S_VERID_MINOR_MASK)
#define I2S_VERID_MAJOR_MASK                     (0xFF000000U)
#define I2S_VERID_MAJOR_SHIFT                    (24U)
#define I2S_VERID_MAJOR(x)                       (((uint32_t)(((uint32_t)(x)) << I2S_VERID_MAJOR_SHIFT)) & I2S_VERID_MAJOR_MASK)
/*! @} */

/*! @name PARAM - PARAM */
/*! @{ */
#define I2S_PARAM_DATALINE_MASK                  (0xFU)
#define I2S_PARAM_DATALINE_SHIFT                 (0U)
#define I2S_PARAM_DATALINE(x)                    (((uint32_t)(((uint32_t)(x)) << I2S_PARAM_DATALINE_SHIFT)) & I2S_PARAM_DATALINE_MASK)
#define I2S_PARAM_FIFO_MASK                      (0xF00U)
#define I2S_PARAM_FIFO_SHIFT                     (8U)
#define I2S_PARAM_FIFO(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_PARAM_FIFO_SHIFT)) & I2S_PARAM_FIFO_MASK)
#define I2S_PARAM_FRAME_MASK                     (0xF0000U)
#define I2S_PARAM_FRAME_SHIFT                    (16U)
#define I2S_PARAM_FRAME(x)                       (((uint32_t)(((uint32_t)(x)) << I2S_PARAM_FRAME_SHIFT)) & I2S_PARAM_FRAME_MASK)
/*! @} */

/*! @name TCSR - TCSR */
/*! @{ */
#define I2S_TCSR_FRDE_MASK                       (0x1U)
#define I2S_TCSR_FRDE_SHIFT                      (0U)
#define I2S_TCSR_FRDE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_FRDE_SHIFT)) & I2S_TCSR_FRDE_MASK)
#define I2S_TCSR_FWDE_MASK                       (0x2U)
#define I2S_TCSR_FWDE_SHIFT                      (1U)
#define I2S_TCSR_FWDE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_FWDE_SHIFT)) & I2S_TCSR_FWDE_MASK)
#define I2S_TCSR_FRIE_MASK                       (0x100U)
#define I2S_TCSR_FRIE_SHIFT                      (8U)
#define I2S_TCSR_FRIE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_FRIE_SHIFT)) & I2S_TCSR_FRIE_MASK)
#define I2S_TCSR_FWIE_MASK                       (0x200U)
#define I2S_TCSR_FWIE_SHIFT                      (9U)
#define I2S_TCSR_FWIE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_FWIE_SHIFT)) & I2S_TCSR_FWIE_MASK)
#define I2S_TCSR_FEIE_MASK                       (0x400U)
#define I2S_TCSR_FEIE_SHIFT                      (10U)
#define I2S_TCSR_FEIE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_FEIE_SHIFT)) & I2S_TCSR_FEIE_MASK)
#define I2S_TCSR_SEIE_MASK                       (0x800U)
#define I2S_TCSR_SEIE_SHIFT                      (11U)
#define I2S_TCSR_SEIE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_SEIE_SHIFT)) & I2S_TCSR_SEIE_MASK)
#define I2S_TCSR_WSIE_MASK                       (0x1000U)
#define I2S_TCSR_WSIE_SHIFT                      (12U)
#define I2S_TCSR_WSIE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_WSIE_SHIFT)) & I2S_TCSR_WSIE_MASK)
#define I2S_TCSR_FRF_MASK                        (0x10000U)
#define I2S_TCSR_FRF_SHIFT                       (16U)
#define I2S_TCSR_FRF(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_FRF_SHIFT)) & I2S_TCSR_FRF_MASK)
#define I2S_TCSR_FWF_MASK                        (0x20000U)
#define I2S_TCSR_FWF_SHIFT                       (17U)
#define I2S_TCSR_FWF(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_FWF_SHIFT)) & I2S_TCSR_FWF_MASK)
#define I2S_TCSR_FEF_MASK                        (0x40000U)
#define I2S_TCSR_FEF_SHIFT                       (18U)
#define I2S_TCSR_FEF(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_FEF_SHIFT)) & I2S_TCSR_FEF_MASK)
#define I2S_TCSR_SEF_MASK                        (0x80000U)
#define I2S_TCSR_SEF_SHIFT                       (19U)
#define I2S_TCSR_SEF(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_SEF_SHIFT)) & I2S_TCSR_SEF_MASK)
#define I2S_TCSR_WSF_MASK                        (0x100000U)
#define I2S_TCSR_WSF_SHIFT                       (20U)
#define I2S_TCSR_WSF(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_WSF_SHIFT)) & I2S_TCSR_WSF_MASK)
#define I2S_TCSR_SR_MASK                         (0x1000000U)
#define I2S_TCSR_SR_SHIFT                        (24U)
#define I2S_TCSR_SR(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_SR_SHIFT)) & I2S_TCSR_SR_MASK)
#define I2S_TCSR_FR_MASK                         (0x2000000U)
#define I2S_TCSR_FR_SHIFT                        (25U)
#define I2S_TCSR_FR(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_FR_SHIFT)) & I2S_TCSR_FR_MASK)
#define I2S_TCSR_BCE_MASK                        (0x10000000U)
#define I2S_TCSR_BCE_SHIFT                       (28U)
#define I2S_TCSR_BCE(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_BCE_SHIFT)) & I2S_TCSR_BCE_MASK)
#define I2S_TCSR_DBGE_MASK                       (0x20000000U)
#define I2S_TCSR_DBGE_SHIFT                      (29U)
#define I2S_TCSR_DBGE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_DBGE_SHIFT)) & I2S_TCSR_DBGE_MASK)
#define I2S_TCSR_TE_MASK                         (0x80000000U)
#define I2S_TCSR_TE_SHIFT                        (31U)
#define I2S_TCSR_TE(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_TCSR_TE_SHIFT)) & I2S_TCSR_TE_MASK)
/*! @} */

/*! @name TCR1 - TCR1 */
/*! @{ */
#define I2S_TCR1_TFW_MASK                        (0x7FU)
#define I2S_TCR1_TFW_SHIFT                       (0U)
#define I2S_TCR1_TFW(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR1_TFW_SHIFT)) & I2S_TCR1_TFW_MASK)
/*! @} */

/*! @name TCR2 - TCR2 */
/*! @{ */
#define I2S_TCR2_DIV_MASK                        (0xFFU)
#define I2S_TCR2_DIV_SHIFT                       (0U)
#define I2S_TCR2_DIV(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR2_DIV_SHIFT)) & I2S_TCR2_DIV_MASK)
#define I2S_TCR2_BYP_MASK                        (0x800000U)
#define I2S_TCR2_BYP_SHIFT                       (23U)
#define I2S_TCR2_BYP(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR2_BYP_SHIFT)) & I2S_TCR2_BYP_MASK)
#define I2S_TCR2_BCD_MASK                        (0x1000000U)
#define I2S_TCR2_BCD_SHIFT                       (24U)
#define I2S_TCR2_BCD(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR2_BCD_SHIFT)) & I2S_TCR2_BCD_MASK)
#define I2S_TCR2_BCP_MASK                        (0x2000000U)
#define I2S_TCR2_BCP_SHIFT                       (25U)
#define I2S_TCR2_BCP(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR2_BCP_SHIFT)) & I2S_TCR2_BCP_MASK)
#define I2S_TCR2_MSEL_MASK                       (0xC000000U)
#define I2S_TCR2_MSEL_SHIFT                      (26U)
#define I2S_TCR2_MSEL(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCR2_MSEL_SHIFT)) & I2S_TCR2_MSEL_MASK)
#define I2S_TCR2_BCI_MASK                        (0x10000000U)
#define I2S_TCR2_BCI_SHIFT                       (28U)
#define I2S_TCR2_BCI(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR2_BCI_SHIFT)) & I2S_TCR2_BCI_MASK)
#define I2S_TCR2_BCS_MASK                        (0x20000000U)
#define I2S_TCR2_BCS_SHIFT                       (29U)
#define I2S_TCR2_BCS(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR2_BCS_SHIFT)) & I2S_TCR2_BCS_MASK)
#define I2S_TCR2_SYNC_MASK                       (0xC0000000U)
#define I2S_TCR2_SYNC_SHIFT                      (30U)
#define I2S_TCR2_SYNC(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCR2_SYNC_SHIFT)) & I2S_TCR2_SYNC_MASK)
/*! @} */

/*! @name TCR3 - TCR3 */
/*! @{ */
#define I2S_TCR3_WDFL_MASK                       (0x1FU)
#define I2S_TCR3_WDFL_SHIFT                      (0U)
#define I2S_TCR3_WDFL(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCR3_WDFL_SHIFT)) & I2S_TCR3_WDFL_MASK)
#define I2S_TCR3_TCE_MASK                        (0xFF0000U)
#define I2S_TCR3_TCE_SHIFT                       (16U)
#define I2S_TCR3_TCE(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR3_TCE_SHIFT)) & I2S_TCR3_TCE_MASK)
#define I2S_TCR3_CFR_MASK                        (0xFF000000U)
#define I2S_TCR3_CFR_SHIFT                       (24U)
#define I2S_TCR3_CFR(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR3_CFR_SHIFT)) & I2S_TCR3_CFR_MASK)
/*! @} */

/*! @name TCR4 - TCR4 */
/*! @{ */
#define I2S_TCR4_FSD_MASK                        (0x1U)
#define I2S_TCR4_FSD_SHIFT                       (0U)
#define I2S_TCR4_FSD(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR4_FSD_SHIFT)) & I2S_TCR4_FSD_MASK)
#define I2S_TCR4_FSP_MASK                        (0x2U)
#define I2S_TCR4_FSP_SHIFT                       (1U)
#define I2S_TCR4_FSP(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR4_FSP_SHIFT)) & I2S_TCR4_FSP_MASK)
#define I2S_TCR4_ONDEM_MASK                      (0x4U)
#define I2S_TCR4_ONDEM_SHIFT                     (2U)
#define I2S_TCR4_ONDEM(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_TCR4_ONDEM_SHIFT)) & I2S_TCR4_ONDEM_MASK)
#define I2S_TCR4_FSE_MASK                        (0x8U)
#define I2S_TCR4_FSE_SHIFT                       (3U)
#define I2S_TCR4_FSE(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR4_FSE_SHIFT)) & I2S_TCR4_FSE_MASK)
#define I2S_TCR4_MF_MASK                         (0x10U)
#define I2S_TCR4_MF_SHIFT                        (4U)
#define I2S_TCR4_MF(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_TCR4_MF_SHIFT)) & I2S_TCR4_MF_MASK)
#define I2S_TCR4_CHMOD_MASK                      (0x20U)
#define I2S_TCR4_CHMOD_SHIFT                     (5U)
#define I2S_TCR4_CHMOD(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_TCR4_CHMOD_SHIFT)) & I2S_TCR4_CHMOD_MASK)
#define I2S_TCR4_SYWD_MASK                       (0x1F00U)
#define I2S_TCR4_SYWD_SHIFT                      (8U)
#define I2S_TCR4_SYWD(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCR4_SYWD_SHIFT)) & I2S_TCR4_SYWD_MASK)
#define I2S_TCR4_FRSZ_MASK                       (0x1F0000U)
#define I2S_TCR4_FRSZ_SHIFT                      (16U)
#define I2S_TCR4_FRSZ(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TCR4_FRSZ_SHIFT)) & I2S_TCR4_FRSZ_MASK)
#define I2S_TCR4_FPACK_MASK                      (0x3000000U)
#define I2S_TCR4_FPACK_SHIFT                     (24U)
#define I2S_TCR4_FPACK(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_TCR4_FPACK_SHIFT)) & I2S_TCR4_FPACK_MASK)
#define I2S_TCR4_FCOMB_MASK                      (0xC000000U)
#define I2S_TCR4_FCOMB_SHIFT                     (26U)
#define I2S_TCR4_FCOMB(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_TCR4_FCOMB_SHIFT)) & I2S_TCR4_FCOMB_MASK)
#define I2S_TCR4_FCONT_MASK                      (0x10000000U)
#define I2S_TCR4_FCONT_SHIFT                     (28U)
#define I2S_TCR4_FCONT(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_TCR4_FCONT_SHIFT)) & I2S_TCR4_FCONT_MASK)
/*! @} */

/*! @name TCR5 - TCR5 */
/*! @{ */
#define I2S_TCR5_FBT_MASK                        (0x1F00U)
#define I2S_TCR5_FBT_SHIFT                       (8U)
#define I2S_TCR5_FBT(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR5_FBT_SHIFT)) & I2S_TCR5_FBT_MASK)
#define I2S_TCR5_W0W_MASK                        (0x1F0000U)
#define I2S_TCR5_W0W_SHIFT                       (16U)
#define I2S_TCR5_W0W(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR5_W0W_SHIFT)) & I2S_TCR5_W0W_MASK)
#define I2S_TCR5_WNW_MASK                        (0x1F000000U)
#define I2S_TCR5_WNW_SHIFT                       (24U)
#define I2S_TCR5_WNW(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TCR5_WNW_SHIFT)) & I2S_TCR5_WNW_MASK)

/*! @name TDR - TDR0..TDR7 */
/*! @{ */
#define I2S_TDR_TDR_MASK                         (0xFFFFFFFFU)
#define I2S_TDR_TDR_SHIFT                        (0U)
#define I2S_TDR_TDR(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_TDR_TDR_SHIFT)) & I2S_TDR_TDR_MASK)
/*! @} */

/* The count of I2S_TDR */
#define I2S_TDR_COUNT                            (8U)

/*! @name TFR - TFR0..TFR7 */
/*! @{ */
#define I2S_TFR_RFP_MASK                         (0xFFU)
#define I2S_TFR_RFP_SHIFT                        (0U)
#define I2S_TFR_RFP(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_TFR_RFP_SHIFT)) & I2S_TFR_RFP_MASK)
#define I2S_TFR_WFP_MASK                         (0xFF0000U)
#define I2S_TFR_WFP_SHIFT                        (16U)
#define I2S_TFR_WFP(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_TFR_WFP_SHIFT)) & I2S_TFR_WFP_MASK)
#define I2S_TFR_WCP_MASK                         (0x80000000U)
#define I2S_TFR_WCP_SHIFT                        (31U)
#define I2S_TFR_WCP(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_TFR_WCP_SHIFT)) & I2S_TFR_WCP_MASK)
/*! @} */

/* The count of I2S_TFR */
#define I2S_TFR_COUNT                            (8U)

/*! @name TMR - TMR */
/*! @{ */
#define I2S_TMR_TWM_MASK                         (0xFFFFFFFFU)
#define I2S_TMR_TWM_SHIFT                        (0U)
#define I2S_TMR_TWM(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_TMR_TWM_SHIFT)) & I2S_TMR_TWM_MASK)
/*! @} */

/*! @name TTCR - TTCR */
/*! @{ */
#define I2S_TTCR_TSEN_MASK                       (0x1U)
#define I2S_TTCR_TSEN_SHIFT                      (0U)
#define I2S_TTCR_TSEN(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TTCR_TSEN_SHIFT)) & I2S_TTCR_TSEN_MASK)
#define I2S_TTCR_TSINC_MASK                      (0x2U)
#define I2S_TTCR_TSINC_SHIFT                     (1U)
#define I2S_TTCR_TSINC(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_TTCR_TSINC_SHIFT)) & I2S_TTCR_TSINC_MASK)
#define I2S_TTCR_RTSC_MASK                       (0x100U)
#define I2S_TTCR_RTSC_SHIFT                      (8U)
#define I2S_TTCR_RTSC(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TTCR_RTSC_SHIFT)) & I2S_TTCR_RTSC_MASK)
#define I2S_TTCR_RBC_MASK                        (0x200U)
#define I2S_TTCR_RBC_SHIFT                       (9U)
#define I2S_TTCR_RBC(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TTCR_RBC_SHIFT)) & I2S_TTCR_RBC_MASK)
/*! @} */

/*! @name TTSR - TTSR */
/*! @{ */
#define I2S_TTSR_TSC_MASK                        (0xFFFFFFFFU)
#define I2S_TTSR_TSC_SHIFT                       (0U)
#define I2S_TTSR_TSC(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_TTSR_TSC_SHIFT)) & I2S_TTSR_TSC_MASK)
/*! @} */

/*! @name TBCR - TBCR */
/*! @{ */
#define I2S_TBCR_BCNT_MASK                       (0xFFFFFFFFU)
#define I2S_TBCR_BCNT_SHIFT                      (0U)
#define I2S_TBCR_BCNT(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_TBCR_BCNT_SHIFT)) & I2S_TBCR_BCNT_MASK)
/*! @} */

/*! @name TBCTR - TBCTR */
/*! @{ */
#define I2S_TBCTR_BCTS_MASK                      (0xFFFFFFFFU)
#define I2S_TBCTR_BCTS_SHIFT                     (0U)
#define I2S_TBCTR_BCTS(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_TBCTR_BCTS_SHIFT)) & I2S_TBCTR_BCTS_MASK)
/*! @} */

/*! @name RCSR - RCSR */
/*! @{ */
#define I2S_RCSR_FRDE_MASK                       (0x1U)
#define I2S_RCSR_FRDE_SHIFT                      (0U)
#define I2S_RCSR_FRDE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_FRDE_SHIFT)) & I2S_RCSR_FRDE_MASK)
#define I2S_RCSR_FWDE_MASK                       (0x2U)
#define I2S_RCSR_FWDE_SHIFT                      (1U)
#define I2S_RCSR_FWDE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_FWDE_SHIFT)) & I2S_RCSR_FWDE_MASK)
#define I2S_RCSR_FRIE_MASK                       (0x100U)
#define I2S_RCSR_FRIE_SHIFT                      (8U)
#define I2S_RCSR_FRIE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_FRIE_SHIFT)) & I2S_RCSR_FRIE_MASK)
#define I2S_RCSR_FWIE_MASK                       (0x200U)
#define I2S_RCSR_FWIE_SHIFT                      (9U)
#define I2S_RCSR_FWIE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_FWIE_SHIFT)) & I2S_RCSR_FWIE_MASK)
#define I2S_RCSR_FEIE_MASK                       (0x400U)
#define I2S_RCSR_FEIE_SHIFT                      (10U)
#define I2S_RCSR_FEIE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_FEIE_SHIFT)) & I2S_RCSR_FEIE_MASK)
#define I2S_RCSR_SEIE_MASK                       (0x800U)
#define I2S_RCSR_SEIE_SHIFT                      (11U)
#define I2S_RCSR_SEIE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_SEIE_SHIFT)) & I2S_RCSR_SEIE_MASK)
#define I2S_RCSR_WSIE_MASK                       (0x1000U)
#define I2S_RCSR_WSIE_SHIFT                      (12U)
#define I2S_RCSR_WSIE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_WSIE_SHIFT)) & I2S_RCSR_WSIE_MASK)
#define I2S_RCSR_FRF_MASK                        (0x10000U)
#define I2S_RCSR_FRF_SHIFT                       (16U)
#define I2S_RCSR_FRF(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_FRF_SHIFT)) & I2S_RCSR_FRF_MASK)
#define I2S_RCSR_FWF_MASK                        (0x20000U)
#define I2S_RCSR_FWF_SHIFT                       (17U)
#define I2S_RCSR_FWF(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_FWF_SHIFT)) & I2S_RCSR_FWF_MASK)
#define I2S_RCSR_FEF_MASK                        (0x40000U)
#define I2S_RCSR_FEF_SHIFT                       (18U)
#define I2S_RCSR_FEF(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_FEF_SHIFT)) & I2S_RCSR_FEF_MASK)
#define I2S_RCSR_SEF_MASK                        (0x80000U)
#define I2S_RCSR_SEF_SHIFT                       (19U)
#define I2S_RCSR_SEF(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_SEF_SHIFT)) & I2S_RCSR_SEF_MASK)
#define I2S_RCSR_WSF_MASK                        (0x100000U)
#define I2S_RCSR_WSF_SHIFT                       (20U)
#define I2S_RCSR_WSF(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_WSF_SHIFT)) & I2S_RCSR_WSF_MASK)
#define I2S_RCSR_SR_MASK                         (0x1000000U)
#define I2S_RCSR_SR_SHIFT                        (24U)
#define I2S_RCSR_SR(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_SR_SHIFT)) & I2S_RCSR_SR_MASK)
#define I2S_RCSR_FR_MASK                         (0x2000000U)
#define I2S_RCSR_FR_SHIFT                        (25U)
#define I2S_RCSR_FR(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_FR_SHIFT)) & I2S_RCSR_FR_MASK)
#define I2S_RCSR_BCE_MASK                        (0x10000000U)
#define I2S_RCSR_BCE_SHIFT                       (28U)
#define I2S_RCSR_BCE(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_BCE_SHIFT)) & I2S_RCSR_BCE_MASK)
#define I2S_RCSR_DBGE_MASK                       (0x20000000U)
#define I2S_RCSR_DBGE_SHIFT                      (29U)
#define I2S_RCSR_DBGE(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_DBGE_SHIFT)) & I2S_RCSR_DBGE_MASK)
#define I2S_RCSR_RE_MASK                         (0x80000000U)
#define I2S_RCSR_RE_SHIFT                        (31U)
#define I2S_RCSR_RE(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_RCSR_RE_SHIFT)) & I2S_RCSR_RE_MASK)
/*! @} */

/*! @name RCR1 - RCR1 */
/*! @{ */
#define I2S_RCR1_RFW_MASK                        (0x7FU)
#define I2S_RCR1_RFW_SHIFT                       (0U)
#define I2S_RCR1_RFW(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR1_RFW_SHIFT)) & I2S_RCR1_RFW_MASK)
/*! @} */

/*! @name RCR2 - RCR2 */
/*! @{ */
#define I2S_RCR2_DIV_MASK                        (0xFFU)
#define I2S_RCR2_DIV_SHIFT                       (0U)
#define I2S_RCR2_DIV(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR2_DIV_SHIFT)) & I2S_RCR2_DIV_MASK)
#define I2S_RCR2_BYP_MASK                        (0x800000U)
#define I2S_RCR2_BYP_SHIFT                       (23U)
#define I2S_RCR2_BYP(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR2_BYP_SHIFT)) & I2S_RCR2_BYP_MASK)
#define I2S_RCR2_BCD_MASK                        (0x1000000U)
#define I2S_RCR2_BCD_SHIFT                       (24U)
#define I2S_RCR2_BCD(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR2_BCD_SHIFT)) & I2S_RCR2_BCD_MASK)
#define I2S_RCR2_BCP_MASK                        (0x2000000U)
#define I2S_RCR2_BCP_SHIFT                       (25U)
#define I2S_RCR2_BCP(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR2_BCP_SHIFT)) & I2S_RCR2_BCP_MASK)
#define I2S_RCR2_MSEL_MASK                       (0xC000000U)
#define I2S_RCR2_MSEL_SHIFT                      (26U)
#define I2S_RCR2_MSEL(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCR2_MSEL_SHIFT)) & I2S_RCR2_MSEL_MASK)
#define I2S_RCR2_BCI_MASK                        (0x10000000U)
#define I2S_RCR2_BCI_SHIFT                       (28U)
#define I2S_RCR2_BCI(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR2_BCI_SHIFT)) & I2S_RCR2_BCI_MASK)
#define I2S_RCR2_BCS_MASK                        (0x20000000U)
#define I2S_RCR2_BCS_SHIFT                       (29U)
#define I2S_RCR2_BCS(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR2_BCS_SHIFT)) & I2S_RCR2_BCS_MASK)
#define I2S_RCR2_SYNC_MASK                       (0xC0000000U)
#define I2S_RCR2_SYNC_SHIFT                      (30U)
#define I2S_RCR2_SYNC(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCR2_SYNC_SHIFT)) & I2S_RCR2_SYNC_MASK)
/*! @} */

/*! @name RCR3 - RCR3 */
/*! @{ */
#define I2S_RCR3_WDFL_MASK                       (0x1FU)
#define I2S_RCR3_WDFL_SHIFT                      (0U)
#define I2S_RCR3_WDFL(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCR3_WDFL_SHIFT)) & I2S_RCR3_WDFL_MASK)
#define I2S_RCR3_RCE_MASK                        (0xFF0000U)
#define I2S_RCR3_RCE_SHIFT                       (16U)
#define I2S_RCR3_RCE(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR3_RCE_SHIFT)) & I2S_RCR3_RCE_MASK)
#define I2S_RCR3_CFR_MASK                        (0xFF000000U)
#define I2S_RCR3_CFR_SHIFT                       (24U)
#define I2S_RCR3_CFR(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR3_CFR_SHIFT)) & I2S_RCR3_CFR_MASK)
/*! @} */

/*! @name RCR4 - RCR4 */
/*! @{ */
#define I2S_RCR4_FSD_MASK                        (0x1U)
#define I2S_RCR4_FSD_SHIFT                       (0U)
#define I2S_RCR4_FSD(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR4_FSD_SHIFT)) & I2S_RCR4_FSD_MASK)
#define I2S_RCR4_FSP_MASK                        (0x2U)
#define I2S_RCR4_FSP_SHIFT                       (1U)
#define I2S_RCR4_FSP(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR4_FSP_SHIFT)) & I2S_RCR4_FSP_MASK)
#define I2S_RCR4_ONDEM_MASK                      (0x4U)
#define I2S_RCR4_ONDEM_SHIFT                     (2U)
#define I2S_RCR4_ONDEM(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_RCR4_ONDEM_SHIFT)) & I2S_RCR4_ONDEM_MASK)
#define I2S_RCR4_FSE_MASK                        (0x8U)
#define I2S_RCR4_FSE_SHIFT                       (3U)
#define I2S_RCR4_FSE(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR4_FSE_SHIFT)) & I2S_RCR4_FSE_MASK)
#define I2S_RCR4_MF_MASK                         (0x10U)
#define I2S_RCR4_MF_SHIFT                        (4U)
#define I2S_RCR4_MF(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_RCR4_MF_SHIFT)) & I2S_RCR4_MF_MASK)
#define I2S_RCR4_SYWD_MASK                       (0x1F00U)
#define I2S_RCR4_SYWD_SHIFT                      (8U)
#define I2S_RCR4_SYWD(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCR4_SYWD_SHIFT)) & I2S_RCR4_SYWD_MASK)
#define I2S_RCR4_FRSZ_MASK                       (0x1F0000U)
#define I2S_RCR4_FRSZ_SHIFT                      (16U)
#define I2S_RCR4_FRSZ(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RCR4_FRSZ_SHIFT)) & I2S_RCR4_FRSZ_MASK)
#define I2S_RCR4_FPACK_MASK                      (0x3000000U)
#define I2S_RCR4_FPACK_SHIFT                     (24U)
#define I2S_RCR4_FPACK(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_RCR4_FPACK_SHIFT)) & I2S_RCR4_FPACK_MASK)
#define I2S_RCR4_FCOMB_MASK                      (0xC000000U)
#define I2S_RCR4_FCOMB_SHIFT                     (26U)
#define I2S_RCR4_FCOMB(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_RCR4_FCOMB_SHIFT)) & I2S_RCR4_FCOMB_MASK)
#define I2S_RCR4_FCONT_MASK                      (0x10000000U)
#define I2S_RCR4_FCONT_SHIFT                     (28U)
#define I2S_RCR4_FCONT(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_RCR4_FCONT_SHIFT)) & I2S_RCR4_FCONT_MASK)
/*! @} */

/*! @name RCR5 - RCR5 */
/*! @{ */
#define I2S_RCR5_FBT_MASK                        (0x1F00U)
#define I2S_RCR5_FBT_SHIFT                       (8U)
#define I2S_RCR5_FBT(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR5_FBT_SHIFT)) & I2S_RCR5_FBT_MASK)
#define I2S_RCR5_W0W_MASK                        (0x1F0000U)
#define I2S_RCR5_W0W_SHIFT                       (16U)
#define I2S_RCR5_W0W(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR5_W0W_SHIFT)) & I2S_RCR5_W0W_MASK)
#define I2S_RCR5_WNW_MASK                        (0x1F000000U)
#define I2S_RCR5_WNW_SHIFT                       (24U)
#define I2S_RCR5_WNW(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RCR5_WNW_SHIFT)) & I2S_RCR5_WNW_MASK)
/*! @} */

/*! @name RDR - RDR0..RDR7 */
/*! @{ */
#define I2S_RDR_RDR_MASK                         (0xFFFFFFFFU)
#define I2S_RDR_RDR_SHIFT                        (0U)
#define I2S_RDR_RDR(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_RDR_RDR_SHIFT)) & I2S_RDR_RDR_MASK)
/*! @} */

/* The count of I2S_RDR */
#define I2S_RDR_COUNT                            (8U)

/*! @name RFR - RFR0..RFR7 */
/*! @{ */
#define I2S_RFR_RFP_MASK                         (0xFFU)
#define I2S_RFR_RFP_SHIFT                        (0U)
#define I2S_RFR_RFP(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_RFR_RFP_SHIFT)) & I2S_RFR_RFP_MASK)
#define I2S_RFR_RCP_MASK                         (0x8000U)
#define I2S_RFR_RCP_SHIFT                        (15U)
#define I2S_RFR_RCP(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_RFR_RCP_SHIFT)) & I2S_RFR_RCP_MASK)
#define I2S_RFR_WFP_MASK                         (0xFF0000U)
#define I2S_RFR_WFP_SHIFT                        (16U)
#define I2S_RFR_WFP(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_RFR_WFP_SHIFT)) & I2S_RFR_WFP_MASK)
/*! @} */

/* The count of I2S_RFR */
#define I2S_RFR_COUNT                            (8U)

/*! @name RMR - RMR */
/*! @{ */
#define I2S_RMR_RWM_MASK                         (0xFFFFFFFFU)
#define I2S_RMR_RWM_SHIFT                        (0U)
#define I2S_RMR_RWM(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_RMR_RWM_SHIFT)) & I2S_RMR_RWM_MASK)
/*! @} */


/*! @name RTCR - RTCR */
/*! @{ */
#define I2S_RTCR_TSEN_MASK                       (0x1U)
#define I2S_RTCR_TSEN_SHIFT                      (0U)
#define I2S_RTCR_TSEN(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RTCR_TSEN_SHIFT)) & I2S_RTCR_TSEN_MASK)
#define I2S_RTCR_TSINC_MASK                      (0x2U)
#define I2S_RTCR_TSINC_SHIFT                     (1U)
#define I2S_RTCR_TSINC(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_RTCR_TSINC_SHIFT)) & I2S_RTCR_TSINC_MASK)
#define I2S_RTCR_RTSC_MASK                       (0x100U)
#define I2S_RTCR_RTSC_SHIFT                      (8U)
#define I2S_RTCR_RTSC(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RTCR_RTSC_SHIFT)) & I2S_RTCR_RTSC_MASK)
#define I2S_RTCR_RBC_MASK                        (0x200U)
#define I2S_RTCR_RBC_SHIFT                       (9U)
#define I2S_RTCR_RBC(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RTCR_RBC_SHIFT)) & I2S_RTCR_RBC_MASK)
/*! @} */

/*! @name RTSR - RTSR */
/*! @{ */
#define I2S_RTSR_TSC_MASK                        (0xFFFFFFFFU)
#define I2S_RTSR_TSC_SHIFT                       (0U)
#define I2S_RTSR_TSC(x)                          (((uint32_t)(((uint32_t)(x)) << I2S_RTSR_TSC_SHIFT)) & I2S_RTSR_TSC_MASK)
/*! @} */

/*! @name RBCR - RBCR */
/*! @{ */
#define I2S_RBCR_BCNT_MASK                       (0xFFFFFFFFU)
#define I2S_RBCR_BCNT_SHIFT                      (0U)
#define I2S_RBCR_BCNT(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_RBCR_BCNT_SHIFT)) & I2S_RBCR_BCNT_MASK)
/*! @} */

/*! @name RBCTR - RBCTR */
/*! @{ */
#define I2S_RBCTR_BCTS_MASK                      (0xFFFFFFFFU)
#define I2S_RBCTR_BCTS_SHIFT                     (0U)
#define I2S_RBCTR_BCTS(x)                        (((uint32_t)(((uint32_t)(x)) << I2S_RBCTR_BCTS_SHIFT)) & I2S_RBCTR_BCTS_MASK)
/*! @} */

/*! @name MCR - MCR */
/*! @{ */
#define I2S_MCR_DIV_MASK                         (0xFFU)
#define I2S_MCR_DIV_SHIFT                        (0U)
#define I2S_MCR_DIV(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_MCR_DIV_SHIFT)) & I2S_MCR_DIV_MASK)
#define I2S_MCR_DIVEN_MASK                       (0x800000U)
#define I2S_MCR_DIVEN_SHIFT                      (23U)
#define I2S_MCR_DIVEN(x)                         (((uint32_t)(((uint32_t)(x)) << I2S_MCR_DIVEN_SHIFT)) & I2S_MCR_DIVEN_MASK)
#define I2S_MCR_MOE_MASK                         (0x40000000U)
#define I2S_MCR_MOE_SHIFT                        (30U)
#define I2S_MCR_MOE(x)                           (((uint32_t)(((uint32_t)(x)) << I2S_MCR_MOE_SHIFT)) & I2S_MCR_MOE_MASK)
/*! @} */
/*!
 * @}
 */ /* end of group I2S_Register_Masks */


/* I2S - Peripheral instance base addresses */
/** Peripheral I2S1 base address */
#define I2S1_BASE   SOC_REGS(0x30010000u)
/*!
 * @}
 */ /* end of group I2S_Peripheral_Access_Layer */

/*! @brief SAI return status*/
enum _sai_status_t {
    kStatus_SAI_TxBusy = MAKE_STATUS(kStatusGroup_SAI, 0),    /*!< SAI Tx is busy. */
    kStatus_SAI_RxBusy = MAKE_STATUS(kStatusGroup_SAI, 1),    /*!< SAI Rx is busy. */
    kStatus_SAI_TxError = MAKE_STATUS(kStatusGroup_SAI, 2),   /*!< SAI Tx FIFO error. */
    kStatus_SAI_RxError = MAKE_STATUS(kStatusGroup_SAI, 3),   /*!< SAI Rx FIFO error. */
    kStatus_SAI_QueueFull = MAKE_STATUS(kStatusGroup_SAI, 4), /*!< SAI transfer queue is full. */
    kStatus_SAI_TxIdle = MAKE_STATUS(kStatusGroup_SAI, 5),    /*!< SAI Tx is idle */
    kStatus_SAI_RxIdle = MAKE_STATUS(kStatusGroup_SAI, 6)     /*!< SAI Rx is idle */
};

/*< sai channel mask value, actual channel numbers is depend soc specific */
enum _sai_channel_mask {
    kSAI_Channel0Mask = 1 << 0U, /*!< channel 0 mask value */
    kSAI_Channel1Mask = 1 << 1U, /*!< channel 1 mask value */
    kSAI_Channel2Mask = 1 << 2U, /*!< channel 2 mask value */
    kSAI_Channel3Mask = 1 << 3U, /*!< channel 3 mask value */
    kSAI_Channel4Mask = 1 << 4U, /*!< channel 4 mask value */
    kSAI_Channel5Mask = 1 << 5U, /*!< channel 5 mask value */
    kSAI_Channel6Mask = 1 << 6U, /*!< channel 6 mask value */
    kSAI_Channel7Mask = 1 << 7U, /*!< channel 7 mask value */
};
/*! @brief Define the SAI bus type */
typedef enum _sai_protocol {
    kSAI_BusLeftJustified = 0x0U, /*!< Uses left justified format.*/
    kSAI_BusRightJustified,       /*!< Uses right justified format. */
    kSAI_BusI2S,                  /*!< Uses I2S format. */
    kSAI_BusPCMA,                 /*!< Uses I2S PCM A format.*/
    kSAI_BusPCMB,                 /*!< Uses I2S PCM B format. */
    kSAI_BusI2SAes3,               /*!< Uses I2S AES3 format. */
    kSAI_BusI2SPDM 
} sai_protocol_t;

/*! @brief Master or slave mode */
typedef enum _sai_master_slave {
    kSAI_Slave = 0x0U,  /*!< Slave mode */
    kSAI_Master = 0x1U  /*!< Master mode */
} sai_master_slave_t;

/*! @brief Mono or stereo audio format */
typedef enum _sai_mono_stereo {
    kSAI_Stereo = 0x0U, /*!< Stereo sound. */
    kSAI_MonoRight,     /*!< Only Right channel have sound. */
    kSAI_MonoLeft       /*!< Only left channel have sound. */
} sai_mono_stereo_t;

/*! @brief SAI data order, MSB or LSB */
typedef enum _sai_data_order {
    kSAI_DataLSB = 0x0U, /*!< LSB bit transferred first */
    kSAI_DataMSB         /*!< MSB bit transferred first */
} sai_data_order_t;

/*! @brief SAI clock polarity, active high or low */
typedef enum _sai_clock_polarity {
    kSAI_PolarityActiveHigh = 0x0U, /*!< Clock active high */
    kSAI_PolarityActiveLow          /*!< Clock active low */
} sai_clock_polarity_t;
/*! @brief Synchronous or asynchronous mode */
typedef enum _sai_sync_mode {
    kSAI_ModeAsync = 0x0U,                  /*!< Asynchronous mode */
    kSAI_ModeSyncWithSameSaiOtherDirection, /*!< Synchronous mode (with receiver or transmit) */
    kSAI_ModeSyncWithOtherSaiSameDirection, /*!< Synchronous with another SAI same direction */
    kSAI_ModeSyncWithOtherSaiOtherDirection /*!< Synchronous with another SAI opposite direction */
} sai_sync_mode_t;

/*! @brief Mater clock source */
typedef enum _sai_mclk_source {
    kSAI_MclkSourceSysclk = 0x0U, /*!< Master clock from the system clock */
    kSAI_MclkSourceSelect1,       /*!< Master clock from source 1 */
    kSAI_MclkSourceSelect2,       /*!< Master clock from source 2 */
    kSAI_MclkSourceSelect3        /*!< Master clock from source 3 */
} sai_mclk_source_t;

/*! @brief Bit clock source */
typedef enum _sai_bclk_source {
    kSAI_BclkSourceBusclk = 0x0U, /*!< Bit clock using bus clock */
    kSAI_BclkSourceMclkDiv,       /*!< Bit clock using master clock divider */
    kSAI_BclkSourceOtherSai0,     /*!< Bit clock from other SAI device  */
    kSAI_BclkSourceOtherSai1      /*!< Bit clock from other SAI device */
} sai_bclk_source_t;

/*! @brief The SAI interrupt enable flag */
enum _sai_interrupt_enable_t {
    kSAI_WordStartInterruptEnable =
        I2S_TCSR_WSIE_MASK, /*!< Word start flag, means the first word in a frame detected */
    kSAI_SyncErrorInterruptEnable = I2S_TCSR_SEIE_MASK,   /*!< Sync error flag, means the sync error is detected */
    kSAI_FIFOWarningInterruptEnable = I2S_TCSR_FWIE_MASK, /*!< FIFO warning flag, means the FIFO is empty */
    kSAI_FIFOErrorInterruptEnable = I2S_TCSR_FEIE_MASK,   /*!< FIFO error flag */
#if defined(FSL_FEATURE_SAI_FIFO_COUNT) && (FSL_FEATURE_SAI_FIFO_COUNT > 1)
    kSAI_FIFORequestInterruptEnable = I2S_TCSR_FRIE_MASK, /*!< FIFO request, means reached watermark */
#endif                                                    /* FSL_FEATURE_SAI_FIFO_COUNT */
};

/*! @brief The DMA request sources */
enum _sai_dma_enable_t {
    kSAI_FIFOWarningDMAEnable = I2S_TCSR_FWDE_MASK, /*!< FIFO warning caused by the DMA request */
#if defined(FSL_FEATURE_SAI_FIFO_COUNT) && (FSL_FEATURE_SAI_FIFO_COUNT > 1)
    kSAI_FIFORequestDMAEnable = I2S_TCSR_FRDE_MASK, /*!< FIFO request caused by the DMA request */
#endif                                              /* FSL_FEATURE_SAI_FIFO_COUNT */
};

/*! @brief SAI DMA mode */
typedef enum _dma_mode {
    kSAI_DMA_Private_Buffer = 0x0U, /*!< DMA fills an intermediate buffer further flushed by CPU (default) */
    kSAI_DMA_Zerocopy_Uncached,     /*!< Zero-copy into RX uncached circular buffer */
    kSAI_DMA_Zerocopy_Cached,       /*!< Zero-copy into RX cached circular buffer (cache management) */
} dma_mode_t;

/*! @brief The SAI status flag */
enum _sai_flags {
    kSAI_WordStartFlag = I2S_TCSR_WSF_MASK, /*!< Word start flag, means the first word in a frame detected */
    kSAI_SyncErrorFlag = I2S_TCSR_SEF_MASK, /*!< Sync error flag, means the sync error is detected */
    kSAI_FIFOErrorFlag = I2S_TCSR_FEF_MASK, /*!< FIFO error flag */
#if defined(FSL_FEATURE_SAI_FIFO_COUNT) && (FSL_FEATURE_SAI_FIFO_COUNT > 1)
    kSAI_FIFORequestFlag = I2S_TCSR_FRF_MASK, /*!< FIFO request flag. */
#endif                                        /* FSL_FEATURE_SAI_FIFO_COUNT */
    kSAI_FIFOWarningFlag = I2S_TCSR_FWF_MASK, /*!< FIFO warning flag */
};

/*! @brief The reset type */
typedef enum _sai_reset_type {
    kSAI_ResetTypeSoftware = I2S_TCSR_SR_MASK,          /*!< Software reset, reset the logic state */
    kSAI_ResetTypeFIFO = I2S_TCSR_FR_MASK,              /*!< FIFO reset, reset the FIFO read and write pointer */
    kSAI_ResetAll = I2S_TCSR_SR_MASK | I2S_TCSR_FR_MASK /*!< All reset. */
} sai_reset_type_t;

#if defined(FSL_FEATURE_SAI_HAS_FIFO_PACKING) && FSL_FEATURE_SAI_HAS_FIFO_PACKING
/*!
 * @brief The SAI packing mode
 * The mode includes 8 bit and 16 bit packing.
 */
typedef enum _sai_fifo_packing {
    kSAI_FifoPackingDisabled = 0x0U, /*!< Packing disabled */
    kSAI_FifoPacking8bit = 0x2U,     /*!< 8 bit packing enabled */
    kSAI_FifoPacking16bit = 0x3U     /*!< 16bit packing enabled */
} sai_fifo_packing_t;
#endif /* FSL_FEATURE_SAI_HAS_FIFO_PACKING */

/*! @brief SAI user configuration structure */
typedef struct _sai_config {
    sai_protocol_t protocol;  /*!< Audio bus protocol in SAI */
    sai_sync_mode_t syncMode; /*!< SAI sync mode, control Tx/Rx clock sync */
#if defined(FSL_FEATURE_SAI_HAS_MCR) && (FSL_FEATURE_SAI_HAS_MCR)
    bool mclkOutputEnable;          /*!< Master clock output enable, true means master clock divider enabled */
#endif                              /* FSL_FEATURE_SAI_HAS_MCR */
    sai_mclk_source_t mclkSource;   /*!< Master Clock source */
    sai_bclk_source_t bclkSource;   /*!< Bit Clock source */
    sai_master_slave_t masterSlave; /*!< Master or slave */
} sai_config_t;

/*!@brief SAI transfer queue size, user can refine it according to use case. */
#define SAI_RX_XFER_QUEUE_SIZE (8)
#define SAI_TX_XFER_QUEUE_SIZE (2)

#if (SAI_TX_XFER_QUEUE_SIZE > SAI_RX_XFER_QUEUE_SIZE)
#define SAI_MAX_XFER_QUEUE_SIZE (SAI_TX_XFER_QUEUE_SIZE)
#else
#define SAI_MAX_XFER_QUEUE_SIZE (SAI_RX_XFER_QUEUE_SIZE)
#endif

#define SAI_TX_MAX_AUDIO_CHANNEL  16
#define SAI_TX_XFER_MAX_SILENCE_SIZE (FSL_FEATURE_SAI_FIFO_COUNT * \
        sizeof(uint32_t) * SAI_TX_MAX_AUDIO_CHANNEL)

/*! @brief Audio sample rate */
typedef enum _sai_sample_rate {
    kSAI_SampleRate8KHz = 8000U,     /*!< Sample rate 8000 Hz */
    kSAI_SampleRate11025Hz = 11025U, /*!< Sample rate 11025 Hz */
    kSAI_SampleRate12KHz = 12000U,   /*!< Sample rate 12000 Hz */
    kSAI_SampleRate16KHz = 16000U,   /*!< Sample rate 16000 Hz */
    kSAI_SampleRate22050Hz = 22050U, /*!< Sample rate 22050 Hz */
    kSAI_SampleRate24KHz = 24000U,   /*!< Sample rate 24000 Hz */
    kSAI_SampleRate32KHz = 32000U,   /*!< Sample rate 32000 Hz */
    kSAI_SampleRate44100Hz = 44100U, /*!< Sample rate 44100 Hz */
    kSAI_SampleRate48KHz = 48000U,   /*!< Sample rate 48000 Hz */
    kSAI_SampleRate96KHz = 96000U,   /*!< Sample rate 96000 Hz */
    kSAI_SampleRate192KHz = 192000U, /*!< Sample rate 192000 Hz */
    kSAI_SampleRate384KHz = 384000U, /*!< Sample rate 384000 Hz */
} sai_sample_rate_t;

/*! @brief Audio word width */
typedef enum _sai_word_width {
    kSAI_WordWidth8bits = 8U,   /*!< Audio data width 8 bits */
    kSAI_WordWidth16bits = 16U, /*!< Audio data width 16 bits */
    kSAI_WordWidth24bits = 24U, /*!< Audio data width 24 bits */
    kSAI_WordWidth32bits = 32U  /*!< Audio data width 32 bits */
} sai_word_width_t;

/*! @brief sai transfer format */
typedef struct _sai_transfer_format {
    uint32_t sampleRate_Hz;   /*!< Sample rate of audio data */
    uint32_t bitWidth;        /*!< Data length of audio data, usually 8/16/24/32 bits */
    sai_mono_stereo_t stereo; /*!< Mono or stereo */
    uint32_t audio_channel;   /*!< Number of Audio Channels */
    uint8_t slot;             /*!< Number of Audio Channels */
    uint32_t masterClockHz;   /*!< Master clock frequency in Hz */
#if defined(FSL_FEATURE_SAI_FIFO_COUNT) && (FSL_FEATURE_SAI_FIFO_COUNT > 1)
    uint8_t watermark; /*!< Watermark value */
#endif                 /* FSL_FEATURE_SAI_FIFO_COUNT */

    /* for the multi channel usage, user can provide channelMask Oonly, then sai driver will handle
    * other parameter carefully, such as
    * channelMask = kSAI_Channel0Mask | kSAI_Channel1Mask | kSAI_Channel4Mask
    * then in SAI_RxSetFormat/SAI_TxSetFormat function, channel/endChannel/channelNums will be calculated.
    * for the single channel usage, user can provide channel or channel mask only, such as,
    * channel = 0 or channelMask = kSAI_Channel0Mask.
    */
    uint8_t channel;     /*!< Transfer start channel */
    uint8_t channelMask; /*!< enabled channel mask value, reference _sai_channel_mask */
    uint8_t endChannel;  /*!< end channel number */
    uint8_t channelNums; /*!< Total enabled channel numbers */
    sai_protocol_t protocol; /*!< Which audio protocol used */
    bool isFrameSyncCompact; /*!< True means Frame sync length is configurable according to bitWidth, false means frame
                                sync length is 64 times of bit clock. */
} sai_transfer_format_t;

/*! @brief SAI transfer structure */
typedef struct _sai_transfer {
    uint8_t *buf;   /*!< Data start address to transfer. */
    volatile uint8_t *data;   /*!< Data start address to transfer. */
    volatile size_t dataSize; /*!< Transfer size. */
    size_t capacity; /*!< Transfer size. */
    uint32_t period_size;                           /* period size */
    uint8_t periods;                                /* periods */
} sai_transfer_t;

typedef struct _sai_handle sai_handle_t;

/*! @brief SAI transfer callback prototype */
typedef void (*sai_transfer_callback_t)(I2S_Type *base, sai_handle_t *handle, status_t status, void *userData);

/*! @brief SAI handle structure */
struct _sai_handle {
    I2S_Type *base; /*!< base address */

    uint32_t state;                   /*!< Transfer status */
    sai_transfer_callback_t callback; /*!< Callback function called at transfer event*/
    void *userData;                   /*!< Callback parameter passed to callback function*/
    uint8_t bitWidth;                 /*!< Bit width for transfer, 8/16/24/32 bits */

    /* for the multi channel usage, user can provide channelMask Oonly, then sai driver will handle
    * other parameter carefully, such as
    * channelMask = kSAI_Channel0Mask | kSAI_Channel1Mask | kSAI_Channel4Mask
    * then in SAI_RxSetFormat/SAI_TxSetFormat function, channel/endChannel/channelNums will be calculated.
    * for the single channel usage, user can provide channel or channel mask only, such as,
    * channel = 0 or channelMask = kSAI_Channel0Mask.
    */
    uint8_t channel;     /*!< Transfer start channel */
    uint8_t channelMask; /*!< enabled channel mask value, refernece _sai_channel_mask */
    uint8_t endChannel;  /*!< end channel number */
    uint8_t channelNums; /*!< Total enabled channel numbers */
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
    sai_transfer_t saiQueue[SAI_MAX_XFER_QUEUE_SIZE]; /*!< Transfer queue storing queued transfer */
    size_t transferSize[SAI_MAX_XFER_QUEUE_SIZE];     /*!< Data bytes need to transfer */
    volatile uint8_t queueUser;                   /*!< Index for user to queue transfer */
    volatile uint8_t queueDriver;                 /*!< Index for driver to get the transfer data and size */
#endif
    uint8_t *buf;
    size_t buf_size;
    ssize_t xfer_remaining;
    unsigned burst_length;
#define TX_CBUF_N_FIFO_WATERMARK 4
#define RX_CBUF_N_FIFO_WATERMARK 4
    size_t cbuf_watermark;
#if defined(FSL_FEATURE_SAI_FIFO_COUNT) && (FSL_FEATURE_SAI_FIFO_COUNT > 1)
    uint8_t watermark; /*!< Watermark value */
#endif
    bool dma_enable;
    uint32_t dma_mode;
    struct dma_chan *dma_chan;                    /*!< client DMA channel */
    uint64_t ts_start;
    uint64_t ts_stop;
};

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /*_cplusplus*/

#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
extern void I2S1_DriverIRQHandler(void);
extern void I2S2_DriverIRQHandler(void);
#endif

#define SAI_dump_reg(base, lvl) \
{\
    if (lvl < LOCAL_TRACE) {\
        printlk(LK_NOTICE, "%s:%d: VERI : %#x\n", __PRETTY_FUNCTION__, __LINE__, base->VERID);\
        printlk(LK_NOTICE, "%s:%d: PARA : %#x\n", __PRETTY_FUNCTION__, __LINE__, base->PARAM);\
        printlk(LK_NOTICE, "%s:%d: TCSR : %#x\n", __PRETTY_FUNCTION__, __LINE__, base->TCSR);\
        printlk(LK_NOTICE, "%s:%d: TCR1 : %#x\n", __PRETTY_FUNCTION__, __LINE__, base->TCR1);\
        printlk(LK_NOTICE, "%s:%d: TCR2 : %#x\n", __PRETTY_FUNCTION__, __LINE__, base->TCR2);\
        printlk(LK_NOTICE, "%s:%d: TCR3 : %#x\n", __PRETTY_FUNCTION__, __LINE__, base->TCR3);\
        printlk(LK_NOTICE, "%s:%d: TCR4 : %#x\n", __PRETTY_FUNCTION__, __LINE__, base->TCR4);\
        printlk(LK_NOTICE, "%s:%d: TCR5 : %#x\n", __PRETTY_FUNCTION__, __LINE__, base->TCR5);\
        printlk(LK_NOTICE, "%s:%d: TFR  : %#x %#x %#x %#x %#x %#x %#x %#x\n",\
           __PRETTY_FUNCTION__, __LINE__, \
           base->TFR[0],\
           base->TFR[1],\
           base->TFR[2],\
           base->TFR[3],\
           base->TFR[4],\
           base->TFR[5],\
           base->TFR[6],\
           base->TFR[7]);\
        printlk(LK_NOTICE, "%s:%d: TMR  : %#x\n", __PRETTY_FUNCTION__, __LINE__, base->TMR);\
        printlk(LK_NOTICE, "%s:%d: RCSR : %#x\n", __PRETTY_FUNCTION__, __LINE__, base->RCSR);\
        printlk(LK_NOTICE, "%s:%d: RCR1 : %#x\n", __PRETTY_FUNCTION__, __LINE__, base->RCR1);\
        printlk(LK_NOTICE, "%s:%d: RCR2 : %#x\n", __PRETTY_FUNCTION__, __LINE__, base->RCR2);\
        printlk(LK_NOTICE, "%s:%d: RCR3 : %#x\n", __PRETTY_FUNCTION__, __LINE__, base->RCR3);\
        printlk(LK_NOTICE, "%s:%d: RCR4 : %#x\n", __PRETTY_FUNCTION__, __LINE__, base->RCR4);\
        printlk(LK_NOTICE, "%s:%d: RCR5 : %#x\n", __PRETTY_FUNCTION__, __LINE__, base->RCR5);\
        printlk(LK_NOTICE, "%s:%d: RFR  : %#x %#x %#x %#x %#x %#x %#x %#x\n",\
           __PRETTY_FUNCTION__, __LINE__, \
           base->RFR[0],\
           base->RFR[1],\
           base->RFR[2],\
           base->RFR[3],\
           base->RFR[4],\
           base->RFR[5],\
           base->RFR[6],\
           base->RFR[7]);\
        printlk(LK_NOTICE, "%s:%d: RMR  : %#x\n", __PRETTY_FUNCTION__, __LINE__, base->RMR);\
    }\
}

/*!
 * @name Initialization and deinitialization
 * @{
 */

/*!
 * @brief Initializes the SAI Tx peripheral.
 *
 * Ungates the SAI clock, resets the module, and configures SAI Tx with a configuration structure.
 * The configuration structure can be custom filled or set with default values by
 * SAI_TxGetDefaultConfig().
 *
 * @note  This API should be called at the beginning of the application to use
 * the SAI driver. Otherwise, accessing the SAIM module can cause a hard fault
 * because the clock is not enabled.
 *
 * @param base SAI base pointer
 * @param config SAI configuration structure.
*/
void SAI_TxInit(I2S_Type *base, const sai_config_t *config);

/*!
 * @brief Initializes the SAI Rx peripheral.
 *
 * Ungates the SAI clock, resets the module, and configures the SAI Rx with a configuration structure.
 * The configuration structure can be custom filled or set with default values by
 * SAI_RxGetDefaultConfig().
 *
 * @note  This API should be called at the beginning of the application to use
 * the SAI driver. Otherwise, accessing the SAI module can cause a hard fault
 * because the clock is not enabled.
 *
 * @param base SAI base pointer
 * @param config SAI configuration structure.
 */
void SAI_RxInit(I2S_Type *base, const sai_config_t *config);

/*!
 * @brief  Sets the SAI Tx configuration structure to default values.
 *
 * This API initializes the configuration structure for use in SAI_TxConfig().
 * The initialized structure can remain unchanged in SAI_TxConfig(), or it can be modified
 *  before calling SAI_TxConfig().
 * This is an example.
   @code
   sai_config_t config;
   SAI_TxGetDefaultConfig(&config);
   @endcode
 *
 * @param config pointer to master configuration structure
 */
void SAI_TxGetDefaultConfig(sai_config_t *config);

/*!
 * @brief  Sets the SAI Rx configuration structure to default values.
 *
 * This API initializes the configuration structure for use in SAI_RxConfig().
 * The initialized structure can remain unchanged in SAI_RxConfig() or it can be modified
 *  before calling SAI_RxConfig().
 * This is an example.
   @code
   sai_config_t config;
   SAI_RxGetDefaultConfig(&config);
   @endcode
 *
 * @param config pointer to master configuration structure
 */
void SAI_RxGetDefaultConfig(sai_config_t *config);

/*!
 * @brief De-initializes the SAI peripheral.
 *
 * This API gates the SAI clock. The SAI module can't operate unless SAI_TxInit
 * or SAI_RxInit is called to enable the clock.
 *
 * @param base SAI base pointer
*/
void SAI_Deinit(I2S_Type *base);

/*!
 * @brief Configures the MCLK output enable bit
 *
 * If Master Output Enable bit (MOE) is present in the SAI register map, this
 * will set it according to enable parameter.
 *
 * @param enable When set, MOE bit is set in SAI register.
*/
void SAI_SetMclkOutputEnable(I2S_Type *base, bool enable);

/*!
 * @brief Resets the SAI Tx.
 *
 * This function enables the software reset and FIFO reset of SAI Tx. After reset, clear the reset bit.
 *
 * @param base SAI base pointer
 */
void SAI_TxReset(I2S_Type *base);

/*!
 * @brief Resets the SAI Rx.
 *
 * This function enables the software reset and FIFO reset of SAI Rx. After reset, clear the reset bit.
 *
 * @param base SAI base pointer
 */
void SAI_RxReset(I2S_Type *base);

/*!
 * @brief Enables/disables the SAI Tx.
 *
 * @param base SAI base pointer
 * @param enable True means enable SAI Tx, false means disable.
 */
void SAI_TxEnable(I2S_Type *base, bool enable);

/*!
 * @brief Enables/disables the SAI Rx.
 *
 * @param base SAI base pointer
 * @param enable True means enable SAI Rx, false means disable.
 */
void SAI_RxEnable(I2S_Type *base, bool enable);

/*! @} */

/*!
 * @name Status
 * @{
 */

/*!
 * @brief Gets the SAI Tx status flag state.
 *
 * @param base SAI base pointer
 * @return SAI Tx status flag value. Use the Status Mask to get the status value needed.
 */
static inline uint32_t SAI_TxGetStatusFlag(I2S_Type *base)
{
    return base->TCSR;
}

/*!
 * @brief Clears the SAI Tx status flag state.
 *
 * @param base SAI base pointer
 * @param mask State mask. It can be a combination of the following source if defined:
 *        @arg kSAI_WordStartFlag
 *        @arg kSAI_SyncErrorFlag
 *        @arg kSAI_FIFOErrorFlag
 */
static inline void SAI_TxClearStatusFlags(I2S_Type *base, uint32_t mask)
{
    base->TCSR = ((base->TCSR & 0xFFE3FFFFU) | mask);
}

/*!
 * @brief Gets the SAI Tx status flag state.
 *
 * @param base SAI base pointer
 * @return SAI Rx status flag value. Use the Status Mask to get the status value needed.
 */
static inline uint32_t SAI_RxGetStatusFlag(I2S_Type *base)
{
    return base->RCSR;
}

/*!
 * @brief Clears the SAI Rx status flag state.
 *
 * @param base SAI base pointer
 * @param mask State mask. It can be a combination of the following sources if defined.
 *        @arg kSAI_WordStartFlag
 *        @arg kSAI_SyncErrorFlag
 *        @arg kSAI_FIFOErrorFlag
 */
static inline void SAI_RxClearStatusFlags(I2S_Type *base, uint32_t mask)
{
    base->RCSR = ((base->RCSR & 0xFFE3FFFFU) | mask);
}

/*!
 * @brief Do software reset or FIFO reset .
 *
 * FIFO reset means clear all the data in the FIFO, and make the FIFO pointer both to 0.
 * Software reset means claer the Tx internal logic, including the bit clock, frame count etc. But software
 * reset will not clear any configuration registers like TCR1~TCR5.
 * This function will also clear all the error flags such as FIFO error, sync error etc.
 *
 * @param base SAI base pointer
 * @param type Reset type, FIFO reset or software reset
 */
void SAI_TxSoftwareReset(I2S_Type *base, sai_reset_type_t type);

/*!
 * @brief Do software reset or FIFO reset .
 *
 * FIFO reset means clear all the data in the FIFO, and make the FIFO pointer both to 0.
 * Software reset means claer the Rx internal logic, including the bit clock, frame count etc. But software
 * reset will not clear any configuration registers like RCR1~RCR5.
 * This function will also clear all the error flags such as FIFO error, sync error etc.
 *
 * @param base SAI base pointer
 * @param type Reset type, FIFO reset or software reset
 */
void SAI_RxSoftwareReset(I2S_Type *base, sai_reset_type_t type);

/*!
 * @brief Set the Tx channel FIFO enable mask.
 *
 * @param base SAI base pointer
 * @param mask Channel enable mask, 0 means all channel FIFO disabled, 1 means channel 0 enabled,
 * 3 means both channel 0 and channel 1 enabled.
 */
void SAI_TxSetChannelFIFOMask(I2S_Type *base, uint8_t mask);

/*!
 * @brief Set the Rx channel FIFO enable mask.
 *
 * @param base SAI base pointer
 * @param mask Channel enable mask, 0 means all channel FIFO disabled, 1 means channel 0 enabled,
 * 3 means both channel 0 and channel 1 enabled.
 */
void SAI_RxSetChannelFIFOMask(I2S_Type *base, uint8_t mask);
/*!
 * @brief Set the Tx data order.
 *
 * @param base SAI base pointer
 * @param order Data order MSB or LSB
 */
void SAI_TxSetDataOrder(I2S_Type *base, sai_data_order_t order);

/*!
 * @brief Set the Rx data order.
 *
 * @param base SAI base pointer
 * @param order Data order MSB or LSB
 */
void SAI_RxSetDataOrder(I2S_Type *base, sai_data_order_t order);

/*!
 * @brief Set the Tx data order.
 *
 * @param base SAI base pointer
 * @param order Data order MSB or LSB
 */
void SAI_TxSetBitClockPolarity(I2S_Type *base, sai_clock_polarity_t polarity);

/*!
 * @brief Set the Rx data order.
 *
 * @param base SAI base pointer
 * @param order Data order MSB or LSB
 */
void SAI_RxSetBitClockPolarity(I2S_Type *base, sai_clock_polarity_t polarity);

/*!
 * @brief Set the Tx data order.
 *
 * @param base SAI base pointer
 * @param order Data order MSB or LSB
 */
void SAI_TxSetFrameSyncPolarity(I2S_Type *base, sai_clock_polarity_t polarity);

/*!
 * @brief Set the Rx data order.
 *
 * @param base SAI base pointer
 * @param order Data order MSB or LSB
 */
void SAI_RxSetFrameSyncPolarity(I2S_Type *base, sai_clock_polarity_t polarity);

/*!
 * @brief Set the Rx frame sync width.
 *
 * @param base SAI base pointer
 * @param Word width in bit
 */
void SAI_RxSetFrameSyncWidth(I2S_Type *base, uint32_t bitWidth);

#if defined(FSL_FEATURE_SAI_HAS_FIFO_PACKING) && FSL_FEATURE_SAI_HAS_FIFO_PACKING
/*!
 * @brief Set Tx FIFO packing feature.
 *
 * @param base SAI base pointer.
 * @param pack FIFO pack type. It is element of sai_fifo_packing_t.
 */
void SAI_TxSetFIFOPacking(I2S_Type *base, sai_fifo_packing_t pack);

/*!
* @brief Set Rx FIFO packing feature.
*
* @param base SAI base pointer.
* @param pack FIFO pack type. It is element of sai_fifo_packing_t.
*/
void SAI_RxSetFIFOPacking(I2S_Type *base, sai_fifo_packing_t pack);
#endif /* FSL_FEATURE_SAI_HAS_FIFO_PACKING */

#if defined(FSL_FEATURE_SAI_HAS_FIFO_FUNCTION_AFTER_ERROR) && FSL_FEATURE_SAI_HAS_FIFO_FUNCTION_AFTER_ERROR
/*!
* @brief Set Tx FIFO error continue.
*
* FIFO error continue mode means SAI will keep running while FIFO error occured. If this feature
* not enabled, SAI will hang and users need to clear FEF flag in TCSR register.
*
* @param base SAI base pointer.
* @param isEnabled Is FIFO error continue enabled, true means enable, false means disable.
*/
static inline void SAI_TxSetFIFOErrorContinue(I2S_Type *base, bool isEnabled)
{
    if (isEnabled) {
        base->TCR4 |= I2S_TCR4_FCONT_MASK;
    }
    else {
        base->TCR4 &= ~I2S_TCR4_FCONT_MASK;
    }
}

/*!
* @brief Set Rx FIFO error continue.
*
* FIFO error continue mode means SAI will keep running while FIFO error occured. If this feature
* not enabled, SAI will hang and users need to clear FEF flag in RCSR register.
*
* @param base SAI base pointer.
* @param isEnabled Is FIFO error continue enabled, true means enable, false means disable.
*/
static inline void SAI_RxSetFIFOErrorContinue(I2S_Type *base, bool isEnabled)
{
    if (isEnabled) {
        base->RCR4 |= I2S_RCR4_FCONT_MASK;
    }
    else {
        base->RCR4 &= ~I2S_RCR4_FCONT_MASK;
    }
}
#endif
/*! @} */

/*!
 * @name Interrupts
 * @{
 */

/*!
 * @brief Enables the SAI Tx interrupt requests.
 *
 * @param base SAI base pointer
 * @param mask interrupt source
 *     The parameter can be a combination of the following sources if defined.
 *     @arg kSAI_WordStartInterruptEnable
 *     @arg kSAI_SyncErrorInterruptEnable
 *     @arg kSAI_FIFOWarningInterruptEnable
 *     @arg kSAI_FIFORequestInterruptEnable
 *     @arg kSAI_FIFOErrorInterruptEnable
 */
static inline void SAI_TxEnableInterrupts(I2S_Type *base, uint32_t mask)
{
    base->TCSR = ((base->TCSR & 0xFFE3FFFFU) | mask);
}

/*!
 * @brief Enables the SAI Rx interrupt requests.
 *
 * @param base SAI base pointer
 * @param mask interrupt source
 *     The parameter can be a combination of the following sources if defined.
 *     @arg kSAI_WordStartInterruptEnable
 *     @arg kSAI_SyncErrorInterruptEnable
 *     @arg kSAI_FIFOWarningInterruptEnable
 *     @arg kSAI_FIFORequestInterruptEnable
 *     @arg kSAI_FIFOErrorInterruptEnable
 */
static inline void SAI_RxEnableInterrupts(I2S_Type *base, uint32_t mask)
{
    base->RCSR = ((base->RCSR & 0xFFE3FFFFU) | mask);
}

/*!
 * @brief Disables the SAI Tx interrupt requests.
 *
 * @param base SAI base pointer
 * @param mask interrupt source
 *     The parameter can be a combination of the following sources if defined.
 *     @arg kSAI_WordStartInterruptEnable
 *     @arg kSAI_SyncErrorInterruptEnable
 *     @arg kSAI_FIFOWarningInterruptEnable
 *     @arg kSAI_FIFORequestInterruptEnable
 *     @arg kSAI_FIFOErrorInterruptEnable
 */
static inline void SAI_TxDisableInterrupts(I2S_Type *base, uint32_t mask)
{
    base->TCSR = ((base->TCSR & 0xFFE3FFFFU) & (~mask));
}

/*!
 * @brief Disables the SAI Rx interrupt requests.
 *
 * @param base SAI base pointer
 * @param mask interrupt source
 *     The parameter can be a combination of the following sources if defined.
 *     @arg kSAI_WordStartInterruptEnable
 *     @arg kSAI_SyncErrorInterruptEnable
 *     @arg kSAI_FIFOWarningInterruptEnable
 *     @arg kSAI_FIFORequestInterruptEnable
 *     @arg kSAI_FIFOErrorInterruptEnable
 */
static inline void SAI_RxDisableInterrupts(I2S_Type *base, uint32_t mask)
{
    base->RCSR = ((base->RCSR & 0xFFE3FFFFU) & (~mask));
}

/*! @} */

/*!
 * @name DMA Control
 * @{
 */

/*!
 * @brief Enables/disables the SAI Tx DMA requests.
 * @param base SAI base pointer
 * @param mask DMA source
 *     The parameter can be combination of the following sources if defined.
 *     @arg kSAI_FIFOWarningDMAEnable
 *     @arg kSAI_FIFORequestDMAEnable
 * @param enable True means enable DMA, false means disable DMA.
 */
static inline void SAI_TxEnableDMA(I2S_Type *base, uint32_t mask, bool enable)
{
    if (enable) {
        base->TCSR = ((base->TCSR & 0xFFE3FFFFU) | mask);
    }
    else {
        base->TCSR = ((base->TCSR & 0xFFE3FFFFU) & (~mask));
    }
}

/*!
 * @brief Enables/disables the SAI Rx DMA requests.
 * @param base SAI base pointer
 * @param mask DMA source
 *     The parameter can be a combination of the following sources if defined.
 *     @arg kSAI_FIFOWarningDMAEnable
 *     @arg kSAI_FIFORequestDMAEnable
 * @param enable True means enable DMA, false means disable DMA.
 */
static inline void SAI_RxEnableDMA(I2S_Type *base, uint32_t mask, bool enable)
{
    if (enable) {
        base->RCSR = ((base->RCSR & 0xFFE3FFFFU) | mask);
    }
    else {
        base->RCSR = ((base->RCSR & 0xFFE3FFFFU) & (~mask));
    }
}

/*!
 * @brief  Gets the SAI Tx data register address.
 *
 * This API is used to provide a transfer address for the SAI DMA transfer configuration.
 *
 * @param base SAI base pointer.
 * @param channel Which data channel used.
 * @return data register address.
 */
static inline uintptr_t SAI_TxGetDataRegisterAddress(I2S_Type *base, uint32_t channel)
{
    return (uintptr_t)(&(base->TDR)[channel]);
}

/*!
 * @brief  Gets the SAI Rx data register address.
 *
 * This API is used to provide a transfer address for the SAI DMA transfer configuration.
 *
 * @param base SAI base pointer.
 * @param channel Which data channel used.
 * @return data register address.
 */
static inline uintptr_t SAI_RxGetDataRegisterAddress(I2S_Type *base, uint32_t channel)
{
    return (uintptr_t)(&(base->RDR)[channel]);
}

/*! @} */

/*!
 * @name Bus Operations
 * @{
 */

/*!
 * @brief Configures the SAI Tx audio format.
 *
 * The audio format can be changed at run-time. This function configures the sample rate and audio data
 * format to be transferred.
 *
 * @param base SAI base pointer.
 * @param format Pointer to the SAI audio data format structure.
 * @param mclkSourceClockHz SAI master clock source frequency in Hz.
 * @param bclkSourceClockHz SAI bit clock source frequency in Hz. If the bit clock source is a master
 * clock, this value should equal the masterClockHz.
*/
void SAI_TxSetFormat(I2S_Type *base,
                     sai_transfer_format_t *format,
                     uint32_t mclkSourceClockHz,
                     uint32_t bclkSourceClockHz);

/*!
 * @brief Configures the SAI Rx audio format.
 *
 * The audio format can be changed at run-time. This function configures the sample rate and audio data
 * format to be transferred.
 *
 * @param base SAI base pointer.
 * @param format Pointer to the SAI audio data format structure.
 * @param mclkSourceClockHz SAI master clock source frequency in Hz.
 * @param bclkSourceClockHz SAI bit clock source frequency in Hz. If the bit clock source is a master
 * clock, this value should equal the masterClockHz.
*/
void SAI_RxSetFormat(I2S_Type *base,
                     sai_transfer_format_t *format,
                     uint32_t mclkSourceClockHz,
                     uint32_t bclkSourceClockHz);

/*!
 * @brief Sends data using a blocking method.
 *
 * @note This function blocks by polling until data is ready to be sent.
 *
 * @param base SAI base pointer.
 * @param channel Data channel used.
 * @param bitWidth How many bits in an audio word; usually 8/16/24/32 bits.
 * @param buffer Pointer to the data to be written.
 * @param size Bytes to be written.
 */
void SAI_WriteBlocking(I2S_Type *base, uint32_t channel, uint32_t bitWidth, uint8_t *buffer, uint32_t size);

/*!
 * @brief Sends data to multi channel using a blocking method.
 *
 * @note This function blocks by polling until data is ready to be sent.
 *
 * @param base SAI base pointer.
 * @param channel Data channel used.
 * @param channelMask channel mask.
 * @param bitWidth How many bits in an audio word; usually 8/16/24/32 bits.
 * @param buffer Pointer to the data to be written.
 * @param size Bytes to be written.
 */
void SAI_WriteMultiChannelBlocking(
    I2S_Type *base, uint32_t channel, uint32_t channelMask, uint32_t bitWidth, uint8_t *buffer, uint32_t size);

/*!
 * @brief Writes data into SAI FIFO.
 *
 * @param base SAI base pointer.
 * @param channel Data channel used.
 * @param data Data needs to be written.
 */
static inline void SAI_WriteData(I2S_Type *base, uint32_t channel, uint32_t data)
{
    base->TDR[channel] = data;
}

/*!
 * @brief Receives data using a blocking method.
 *
 * @note This function blocks by polling until data is ready to be sent.
 *
 * @param base SAI base pointer.
 * @param channel Data channel used.
 * @param bitWidth How many bits in an audio word; usually 8/16/24/32 bits.
 * @param buffer Pointer to the data to be read.
 * @param size Bytes to be read.
 */
void SAI_ReadBlocking(I2S_Type *base, uint32_t channel, uint32_t bitWidth, uint8_t *buffer, uint32_t size);

/*!
 * @brief Receives multi channel data using a blocking method.
 *
 * @note This function blocks by polling until data is ready to be sent.
 *
 * @param base SAI base pointer.
 * @param channel Data channel used.
 * @param channelMask channel mask.
 * @param bitWidth How many bits in an audio word; usually 8/16/24/32 bits.
 * @param buffer Pointer to the data to be read.
 * @param size Bytes to be read.
 */
void SAI_ReadMultiChannelBlocking(
    I2S_Type *base, uint32_t channel, uint32_t channelMask, uint32_t bitWidth, uint8_t *buffer, uint32_t size);

/*!
 * @brief Reads data from the SAI FIFO.
 *
 * @param base SAI base pointer.
 * @param channel Data channel used.
 * @return Data in SAI FIFO.
 */
static inline uint32_t SAI_ReadData(I2S_Type *base, uint32_t channel)
{
    return base->RDR[channel];
}

/*!
 * @brief Sends a piece of data in a non-blocking way.
 *
 * @note This function blocks by polling until data is ready to be sent.
 *
 * @param base SAI base pointer.
 * @param channel Data channel used.
 * @param bitWidth How many bits in an audio word; usually 8/16/24/32 bits.
 * @param buffer Pointer to the data to be written.
 * @param size Bytes to be written.
 */
void SAI_WriteNonBlocking(I2S_Type *base, uint32_t channel, uint32_t bitWidth, uint8_t *buffer, uint32_t size);

/*!
 * @brief Returns number of FIFO stage on all channels ready to be read.
 *
 * @param base SAI base pointer
 * @return Number of FIFO stages ready for reading
 */
uint32_t SAI_GetRxFifoLevel(I2S_Type *base);

/*!
 * @brief Receive a piece of data in non-blocking way.
 *
 * @param base SAI base pointer
 * @param channel Data channel used.
 * @param bitWidth How many bits in a audio word, usually 8/16/24/32 bits.
 * @param buffer Pointer to the data to be read.
 * @param size Bytes to be read.
 */
void SAI_ReadNonBlocking(I2S_Type *base, uint32_t channel, uint32_t bitWidth, uint8_t *buffer, uint32_t size);

/*! @} */

/*!
 * @name Transactional
 * @{
 */

#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
/*!
 * @brief Initializes the SAI Tx handle.
 *
 * This function initializes the Tx handle for the SAI Tx transactional APIs. Call
 * this function once to get the handle initialized.
 *
 * @param base SAI base pointer
 * @param handle SAI handle pointer.
 * @param callback Pointer to the user callback function.
 * @param userData User parameter passed to the callback function
 */
void SAI_TransferTxCreateHandle(I2S_Type *base, sai_handle_t *handle, sai_transfer_callback_t callback, void *userData);

/*!
 * @brief Initializes the SAI Rx handle.
 *
 * This function initializes the Rx handle for the SAI Rx transactional APIs. Call
 * this function once to get the handle initialized.
 *
 * @param base SAI base pointer.
 * @param handle SAI handle pointer.
 * @param callback Pointer to the user callback function.
 * @param userData User parameter passed to the callback function.
 */
void SAI_TransferRxCreateHandle(I2S_Type *base, sai_handle_t *handle, sai_transfer_callback_t callback, void *userData);
#endif

/*!
 * @brief Configures the SAI Tx audio format.
 *
 * The audio format can be changed at run-time. This function configures the sample rate and audio data
 * format to be transferred.
 *
 * @param base SAI base pointer.
 * @param handle SAI handle pointer.
 * @param format Pointer to the SAI audio data format structure.
 * @param mclkSourceClockHz SAI master clock source frequency in Hz.
 * @param bclkSourceClockHz SAI bit clock source frequency in Hz. If a bit clock source is a master
 * clock, this value should equal the masterClockHz in format.
 * @return Status of this function. Return value is the status_t.
*/
status_t SAI_TransferTxSetFormat(I2S_Type *base,
                                 sai_handle_t *handle,
                                 sai_transfer_format_t *format,
                                 uint32_t mclkSourceClockHz,
                                 uint32_t bclkSourceClockHz);

/*!
 * @brief Configures the SAI Rx audio format.
 *
 * The audio format can be changed at run-time. This function configures the sample rate and audio data
 * format to be transferred.
 *
 * @param base SAI base pointer.
 * @param handle SAI handle pointer.
 * @param format Pointer to the SAI audio data format structure.
 * @param mclkSourceClockHz SAI master clock source frequency in Hz.
 * @param bclkSourceClockHz SAI bit clock source frequency in Hz. If a bit clock source is a master
 * clock, this value should equal the masterClockHz in format.
 * @return Status of this function. Return value is one of status_t.
*/
status_t SAI_TransferRxSetFormat(I2S_Type *base,
                                 sai_handle_t *handle,
                                 sai_transfer_format_t *format,
                                 uint32_t mclkSourceClockHz,
                                 uint32_t bclkSourceClockHz);

#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
/*!
 * @brief Performs an interrupt non-blocking send transfer on SAI.
 *
 * @note This API returns immediately after the transfer initiates.
 * Call the SAI_TxGetTransferStatusIRQ to poll the transfer status and check whether
 * the transfer is finished. If the return status is not kStatus_SAI_Busy, the transfer
 * is finished.
 *
 * @param base SAI base pointer.
 * @param handle Pointer to the sai_handle_t structure which stores the transfer state.
 * @param xfer Pointer to the sai_transfer_t structure.
 * @retval kStatus_Success Successfully started the data receive.
 * @retval kStatus_SAI_TxBusy Previous receive still not finished.
 * @retval kStatus_InvalidArgument The input parameter is invalid.
 */
status_t SAI_TransferSendNonBlocking(I2S_Type *base, sai_handle_t *handle, sai_transfer_t *xfer);
#else
status_t SAI_TransferSendNonBlocking(I2S_Type *base, sai_handle_t *handle);
#endif

/*!
 * @brief Performs an interrupt non-blocking receive transfer on SAI.
 *
 * @note This API returns immediately after the transfer initiates.
 * Call the SAI_RxGetTransferStatusIRQ to poll the transfer status and check whether
 * the transfer is finished. If the return status is not kStatus_SAI_Busy, the transfer
 * is finished.
 *
 * @param base SAI base pointer
 * @param handle Pointer to the sai_handle_t structure which stores the transfer state.
 * @param xfer Pointer to the sai_transfer_t structure.
 * @retval kStatus_Success Successfully started the data receive.
 * @retval kStatus_SAI_RxBusy Previous receive still not finished.
 * @retval kStatus_InvalidArgument The input parameter is invalid.
 */

#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
status_t SAI_TransferReceiveNonBlocking(I2S_Type *base, sai_handle_t *handle, sai_transfer_t *xfer);
#else
status_t SAI_TransferReceiveNonBlocking(I2S_Type *base, sai_handle_t *handle);
#endif

#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
/*!
 * @brief Gets a set byte count.
 *
 * @param base SAI base pointer.
 * @param handle Pointer to the sai_handle_t structure which stores the transfer state.
 * @param count Bytes count sent.
 * @retval kStatus_Success Succeed get the transfer count.
 * @retval kStatus_NoTransferInProgress There is not a non-blocking transaction currently in progress.
 */
status_t SAI_TransferGetSendCount(I2S_Type *base, sai_handle_t *handle, size_t *count);

/*!
 * @brief Gets a received byte count.
 *
 * @param base SAI base pointer.
 * @param handle Pointer to the sai_handle_t structure which stores the transfer state.
 * @param count Bytes count received.
 * @retval kStatus_Success Succeed get the transfer count.
 * @retval kStatus_NoTransferInProgress There is not a non-blocking transaction currently in progress.
 */
status_t SAI_TransferGetReceiveCount(I2S_Type *base, sai_handle_t *handle, size_t *count);
#endif

/*!
 * @brief Aborts the current send.
 *
 * @note This API can be called any time when an interrupt non-blocking transfer initiates
 * to abort the transfer early.
 *
 * @param base SAI base pointer.
 * @param handle Pointer to the sai_handle_t structure which stores the transfer state.
 */
void SAI_TransferAbortSend(I2S_Type *base, sai_handle_t *handle);

/*!
 * @brief Aborts the current IRQ receive.
 *
 * @note This API can be called when an interrupt non-blocking transfer initiates
 * to abort the transfer early.
 *
 * @param base SAI base pointer
 * @param handle Pointer to the sai_handle_t structure which stores the transfer state.
 */
void SAI_TransferAbortReceive(I2S_Type *base, sai_handle_t *handle);

/*!
 * @brief Terminate all SAI send.
 *
 * This function will clear all transfer slots buffered in the sai queue. If users only want to abort the
 * current transfer slot, please call SAI_TransferAbortSend.
 *
 * @param base SAI base pointer.
 * @param handle SAI eDMA handle pointer.
 */
void SAI_TransferTerminateSend(I2S_Type *base, sai_handle_t *handle);

/*!
 * @brief Terminate all SAI receive.
 *
 * This function will clear all transfer slots buffered in the sai queue. If users only want to abort the
 * current transfer slot, please call SAI_TransferAbortReceive.
 *
 * @param base SAI base pointer.
 * @param handle SAI eDMA handle pointer.
 */
void SAI_TransferTerminateReceive(I2S_Type *base, sai_handle_t *handle);

/*!
 * @brief Tx interrupt handler.
 *
 * @param base SAI base pointer.
 * @param handle Pointer to the sai_handle_t structure.
 */
void SAI_TransferTxHandleIRQ(I2S_Type *base, sai_handle_t *handle);

/*!
 * @brief Tx interrupt handler.
 *
 * @param base SAI base pointer.
 * @param handle Pointer to the sai_handle_t structure.
 */
void SAI_TransferRxHandleIRQ(I2S_Type *base, sai_handle_t *handle);


void SAI_TransferGetBitCounters(I2S_Type *base,
                                uint32_t *bitc, uint32_t *ts, bool is_read);

void SAI_TransferEnCounters(I2S_Type *base, bool en, bool is_read);

/*! @} */

#if defined(__cplusplus)
}
#endif /*_cplusplus*/

/*! @} */

#endif /* __SAI_H_ */
