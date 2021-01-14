/*
 * Copyright 2019-2020 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "spdif_hw.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.spdif"
#endif

/*! @brief Typedef for spdif tx interrupt handler. */
typedef void (*spdif_isr_t)(SPDIF_Type *base, spdif_handle_t *handle);
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
/*!
 * @brief Get the instance number for SPDIF.
 *
 * @param base SPDIF base pointer.
 */
uint32_t SPDIF_GetInstance(SPDIF_Type *base);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Base pointer array */
static SPDIF_Type *const s_spdifBases[] = SPDIF_BASE_PTRS;
#endif /* FSL_SDK_DISABLE_IRQ */
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/* Clock name array */
static const clock_ip_name_t s_spdifClock[] = SPDIF_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
/*! @brief Used for spdif gain */
static uint8_t s_spdif_gain[8] = {24U, 16U, 12U, 8U, 6U, 4U, 3U, 1U};
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
static uint8_t s_spdif_tx_watermark[4] = {16, 12, 8, 4};
static uint8_t s_spdif_rx_watermark[4] = {1, 4, 8, 16};
#endif /* FSL_SDK_DISABLE_IRQ */

/*******************************************************************************
 * Code
 ******************************************************************************/

#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
uint32_t SPDIF_GetInstance(SPDIF_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < ARRAY_SIZE(s_spdifBases); instance++)
    {
        if (s_spdifBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < ARRAY_SIZE(s_spdifBases));

    return instance;
}
#endif /* FSL_SDK_DISABLE_IRQ */

/*!
 * brief Initializes the SPDIF peripheral.
 *
 * Ungates the SPDIF clock, resets the module, and configures SPDIF with a configuration structure.
 * The configuration structure can be custom filled or set with default values by
 * SPDIF_GetDefaultConfig().
 *
 * note  This API should be called at the beginning of the application to use
 * the SPDIF driver. Otherwise, accessing the SPDIF module can cause a hard fault
 * because the clock is not enabled.
 *
 * param base SPDIF base pointer
 * param config SPDIF configuration structure.
*/
void SPDIF_Init(SPDIF_Type *base, const spdif_config_t *config)
{
    uint32_t val = 0;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Enable the SPDIF clock */
    CLOCK_EnableClock(s_spdifClock[SPDIF_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

    /* Reset the internal logic */
    base->SCR |= SPDIF_SCR_SOFT_RESET_MASK;

    /* Waiting for reset finish */
    while (base->SCR & SPDIF_SCR_SOFT_RESET_MASK)
    {
    }

    /* Setting the SPDIF settings */
    base->SCR = SPDIF_SCR_RXFIFOFULL_SEL(config->rxFullSelect) | SPDIF_SCR_RXAUTOSYNC(config->isRxAutoSync) |
                SPDIF_SCR_TXAUTOSYNC(config->isRxAutoSync) | SPDIF_SCR_TXFIFOEMPTY_SEL(config->txFullSelect) |
                SPDIF_SCR_RAW_CAPTURE_MODE(config->isRxRawMode) |
                SPDIF_SCR_TXFIFO_CTRL(1U) | SPDIF_SCR_VALCTRL(config->validityConfig) |
                SPDIF_SCR_TXSEL(config->txSource) | SPDIF_SCR_USRC_SEL(config->uChannelSrc);

    /* Set DPLL clock source */
    base->SRPC = SPDIF_SRPC_CLKSRC_SEL(config->DPLLClkSource) | SPDIF_SRPC_GAINSEL(config->gain);

    /* Set SPDIF tx clock source */
    val = base->STC & ~SPDIF_STC_TXCLK_SOURCE_MASK;
    val |= SPDIF_STC_TXCLK_SOURCE(config->txClkSource);
    base->STC = val;
}

/*!
 * brief De-initializes the SPDIF peripheral.
 *
 * This API gates the SPDIF clock. The SPDIF module can't operate unless SPDIF_Init is called to enable the clock.
 *
 * param base SPDIF base pointer
*/
void SPDIF_Deinit(SPDIF_Type *base)
{
    SPDIF_TxEnable(base, false);
    SPDIF_RxEnable(base, false);
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    CLOCK_DisableClock(s_spdifClock[SPDIF_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

/*!
 * brief  Sets the SPDIF configuration structure to default values.
 *
 * This API initializes the configuration structure for use in SPDIF_Init.
 * The initialized structure can remain unchanged in SPDIF_Init, or it can be modified
 *  before calling SPDIF_Init.
 * This is an example.
   code
   spdif_config_t config;
   SPDIF_GetDefaultConfig(&config);
   endcode
 *
 * param config pointer to master configuration structure
 */
void SPDIF_GetDefaultConfig(spdif_config_t *config)
{
    /* Initializes the configure structure to zero. */
    memset(config, 0, sizeof(*config));

    config->isTxAutoSync = true;
    config->isRxAutoSync = true;
    config->isRxRawMode = false;
    config->DPLLClkSource = 1;
    config->txClkSource = 1;
    config->rxFullSelect = kSPDIF_RxFull8Samples;
    config->txFullSelect = kSPDIF_TxEmpty8Samples;
    config->uChannelSrc = kSPDIF_UChannelFromTx;
    config->txSource = kSPDIF_txNormal;
    config->validityConfig = kSPDIF_validityFlagAlwaysClear;
    config->gain = kSPDIF_GAIN_8;
}

/*!
 * brief Enables/disables the SPDIF Tx.
 *
 * param base SPDIF base pointer
 * param enable True means enable SPDIF Tx, false means disable.
 */
void SPDIF_TxEnable(SPDIF_Type *base, bool enable)
{
    uint32_t val = 0;

    if (enable)
    {
        /* Open Tx FIFO */
        val = base->SCR & (~SPDIF_SCR_TXFIFO_CTRL_MASK);
        val |= SPDIF_SCR_TXFIFO_CTRL(1U);
        base->SCR = val;
        /* Enable transfer clock */
        base->STC |= SPDIF_STC_TX_ALL_CLK_EN_MASK;
    }
    else
    {
        base->SCR &= ~(SPDIF_SCR_TXFIFO_CTRL_MASK | SPDIF_SCR_TXSEL_MASK);
        /* Disable transfer clock */
        base->STC &= ~SPDIF_STC_TX_ALL_CLK_EN_MASK;
    }
}

/*!
 * brief Configures the SPDIF Tx sample rate.
 *
 * The audio format can be changed at run-time. This function configures the sample rate.
 *
 * param base SPDIF base pointer.
 * param sampleRate_Hz SPDIF sample rate frequency in Hz.
 * param sourceClockFreq_Hz SPDIF tx clock source frequency in Hz.
*/
void SPDIF_TxSetSampleRate(SPDIF_Type *base, uint32_t sampleRate_Hz, uint32_t sourceClockFreq_Hz)
{
    uint32_t clkDiv = sourceClockFreq_Hz / (sampleRate_Hz * 64);
    uint32_t mod = sourceClockFreq_Hz % (sampleRate_Hz * 64);
    uint32_t val = 0;
    uint8_t clockSource = (((base->STC) & SPDIF_STC_TXCLK_SOURCE_MASK) >> SPDIF_STC_TXCLK_SOURCE_SHIFT);

    /* Compute the nearest divider */
    if (mod > ((sampleRate_Hz * 64) / 2))
    {
        clkDiv += 1U;
    }

    /* If use divided systeme clock */
    if (clockSource == 5U)
    {
        if (clkDiv > 256)
        {
            val = base->STC & (~(SPDIF_STC_TXCLK_DF_MASK | SPDIF_STC_SYSCLK_DF_MASK));
            val |= SPDIF_STC_SYSCLK_DF(clkDiv / 128U - 1U) | SPDIF_STC_TXCLK_DF(127U);
            base->STC = val;
        }
        else
        {
            val = base->STC & (~(SPDIF_STC_TXCLK_DF_MASK | SPDIF_STC_SYSCLK_DF_MASK));
            val |= SPDIF_STC_SYSCLK_DF(1U) | SPDIF_STC_TXCLK_DF(clkDiv - 1U);
            base->STC = val;
        }
    }
    else
    {
        /* Other clock only uses txclk div */
        val = base->STC & (~(SPDIF_STC_TXCLK_DF_MASK | SPDIF_STC_SYSCLK_DF_MASK));
        val |= SPDIF_STC_TXCLK_DF(clkDiv - 1U);
        base->STC = val;
    }
}

/*!
 * brief Configures the SPDIF Rx audio format.
 *
 * The audio format can be changed at run-time. This function configures the sample rate and audio data
 * format to be transferred.
 *
 * param base SPDIF base pointer.
 * param clockSourceFreq_Hz SPDIF system clock frequency in hz.
 */
uint32_t SPDIF_GetRxSampleRate(SPDIF_Type *base, uint32_t clockSourceFreq_Hz)
{
    uint32_t gain = s_spdif_gain[((base->SRPC & SPDIF_SRPC_GAINSEL_MASK) >> SPDIF_SRPC_GAINSEL_SHIFT)];
    uint32_t measure = 0, sampleRate = 0;
    uint64_t temp = 0;

#ifdef SPDIF_HW_CHECK_PLL_IS_LOCK
    /* Wait the DPLL locked */
    while ((base->SRPC & SPDIF_SRPC_LOCK_MASK) == 0U)
    {
    }
#endif

    /* Get the measure value */
    measure = base->SRFM;
    temp = (uint64_t)measure * (uint64_t)clockSourceFreq_Hz;
    temp /= (uint64_t)(1024 * 1024 * 128 * gain);
    sampleRate = (uint32_t)temp;

    return sampleRate;
}

/*!
 * @brief Dumps SPDIF register.
 *
 * @param base SPDIF base pointer.
 */
void SPDIF_Dump(SPDIF_Type *base)
{
    printf("SPDIF base address: %p\n", base);
    printf("\tSCR: %#x\n", base->SCR);
    printf("\tSRCD: %#x\n", base->SRCD);
    printf("\tSRPC: %#x\n", base->SRPC);
    printf("\tSIE: %#x\n", base->SIE);
    printf("\tSIS: %#x\n", base->SIS);
    printf("\tSRFM: %#x\n", base->SRFM);
    printf("\tSTC: %#x\n", base->STC);
}
