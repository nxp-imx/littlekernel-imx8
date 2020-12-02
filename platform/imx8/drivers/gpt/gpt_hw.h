/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017, 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _GPT_HW_H_
#define _GPT_HW_H_

#include "fsl_common.h"
#include "gpt.h"
#include "gpt_reg_hw.h"

/*!
 * @addtogroup gpt
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
#define FSL_GPT_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0 */
                                                       /*@}*/

/*! @brief List of GPT interrupts */
typedef enum _gpt_interrupt_enable
{
    kGPT_OutputCompare1InterruptEnable = GPT_IR_OF1IE_MASK, /*!< Output Compare Channel1 interrupt enable*/
    kGPT_OutputCompare2InterruptEnable = GPT_IR_OF2IE_MASK, /*!< Output Compare Channel2 interrupt enable*/
    kGPT_OutputCompare3InterruptEnable = GPT_IR_OF3IE_MASK, /*!< Output Compare Channel3 interrupt enable*/
    kGPT_InputCapture1InterruptEnable = GPT_IR_IF1IE_MASK,  /*!< Input Capture Channel1 interrupt enable*/
    kGPT_InputCapture2InterruptEnable = GPT_IR_IF2IE_MASK,  /*!< Input Capture Channel1 interrupt enable*/
    kGPT_RollOverFlagInterruptEnable = GPT_IR_ROVIE_MASK,   /*!< Counter rolled over interrupt enable*/
} gpt_interrupt_enable_t;

/*! @brief Status flag. */
typedef enum _gpt_status_flag
{
    kGPT_OutputCompare1Flag = GPT_SR_OF1_MASK, /*!< Output compare channel 1 event.*/
    kGPT_OutputCompare2Flag = GPT_SR_OF2_MASK, /*!< Output compare channel 2 event.*/
    kGPT_OutputCompare3Flag = GPT_SR_OF3_MASK, /*!< Output compare channel 3 event.*/
    kGPT_InputCapture1Flag = GPT_SR_IF1_MASK,  /*!< Input Capture channel 1 event.*/
    kGPT_InputCapture2Flag = GPT_SR_IF2_MASK,  /*!< Input Capture channel 2 event.*/
    kGPT_RollOverFlag = GPT_SR_ROV_MASK,       /*!< Counter reaches maximum value and rolled over to 0 event.*/
} gpt_status_flag_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Initialization and deinitialization
 * @{
 */

/*!
 * @brief Initialize GPT to reset state and initialize running mode.
 *
 * @param base GPT peripheral base address.
 * @param initConfig GPT mode setting configuration.
 */
void GPT_Init(GPT_Type *base, const gpt_config_t *initConfig);

/*!
 * @brief Disables the module and gates the GPT clock.
 *
 * @param base GPT peripheral base address.
 */
void GPT_Deinit(GPT_Type *base);

/*!
 * @brief Fills in the GPT configuration structure with default settings.
 *
 * The default values are:
 * @code
 *    config->clockSource = kGPT_ClockSource_Periph;
 *    config->divider = 1U;
 *    config->enableRunInStop = true;
 *    config->enableRunInWait = true;
 *    config->enableRunInDoze = false;
 *    config->enableRunInDbg = false;
 *    config->enableFreeRun = true;
 *    config->enableMode  = true;
 * @endcode
 * @param config Pointer to the user configuration structure.
 */
void GPT_GetDefaultConfig(gpt_config_t *config);

/*!
 * @name Software Reset
 * @{
 */

/*!
 * @brief Software reset of GPT module.
 *
 * @param base GPT peripheral base address.
 */
static inline void GPT_SoftwareReset(GPT_Type *base)
{
    base->CR |= GPT_CR_SWR_MASK;
    /* Wait reset finished. */
    while ((base->CR & GPT_CR_SWR_MASK) == GPT_CR_SWR_MASK)
    {
    }
}

/*!
 * @name Clock source and frequency control
 * @{
 */

/*!
 * @brief Set clock source of GPT.
 *
 * @param base GPT peripheral base address.
 * @param source Clock source (see @ref gpt_clock_source_t typedef enumeration).
 */
static inline void GPT_SetClockSource(GPT_Type *base, gpt_clock_source_t source)
{
    if (source == kGPT_ClockSource_Osc)
    {
        base->CR = (base->CR & ~GPT_CR_CLKSRC_MASK) | GPT_CR_EN_24M_MASK | GPT_CR_CLKSRC(source);
    }
    else
    {
        base->CR = (base->CR & ~(GPT_CR_CLKSRC_MASK | GPT_CR_EN_24M_MASK)) | GPT_CR_CLKSRC(source);
    }
}

/*!
 * @brief Get clock source of GPT.
 *
 * @param base GPT peripheral base address.
 * @return clock source (see @ref gpt_clock_source_t typedef enumeration).
 */
static inline gpt_clock_source_t GPT_GetClockSource(GPT_Type *base)
{
    return (gpt_clock_source_t)((base->CR & GPT_CR_CLKSRC_MASK) >> GPT_CR_CLKSRC_SHIFT);
}

/*!
 * @brief Set pre scaler of GPT.
 *
 * @param base GPT peripheral base address.
 * @param divider Divider of GPT (1-4096).
 */
static inline void GPT_SetClockDivider(GPT_Type *base, uint32_t divider)
{
    assert(divider - 1 <= GPT_PR_PRESCALER_MASK);

    base->PR = (base->PR & ~GPT_PR_PRESCALER_MASK) | GPT_PR_PRESCALER(divider - 1);
}

/*!
 * @brief Get clock divider in GPT module.
 *
 * @param base GPT peripheral base address.
 * @return clock divider in GPT module (1-4096).
 */
static inline uint32_t GPT_GetClockDivider(GPT_Type *base)
{
    return ((base->PR & GPT_PR_PRESCALER_MASK) >> GPT_PR_PRESCALER_SHIFT) + 1;
}

/*!
 * @brief OSC 24M pre-scaler before selected by clock source.
 *
 * @param base GPT peripheral base address.
 * @param divider OSC Divider(1-16).
 */
static inline void GPT_SetOscClockDivider(GPT_Type *base, uint32_t divider)
{
    assert(divider - 1 <= (GPT_PR_PRESCALER24M_MASK >> GPT_PR_PRESCALER24M_SHIFT));

    base->PR = (base->PR & ~GPT_PR_PRESCALER24M_MASK) | GPT_PR_PRESCALER24M(divider - 1);
}

/*!
 * @brief Get OSC 24M clock divider in GPT module.
 *
 * @param base GPT peripheral base address.
 * @return OSC clock divider in GPT module (1-16).
 */
static inline uint32_t GPT_GetOscClockDivider(GPT_Type *base)
{
    return ((base->PR & GPT_PR_PRESCALER24M_MASK) >> GPT_PR_PRESCALER24M_SHIFT) + 1;
}

/*! @}*/

/*!
 * @name Timer Start and Stop
 * @{
 */
/*!
 * @brief Start GPT timer.
 *
 * @param base GPT peripheral base address.
 */
static inline void GPT_StartTimer(GPT_Type *base)
{
    base->CR |= GPT_CR_EN_MASK;
}

/*!
 * @brief Stop GPT timer.
 *
 * @param base GPT peripheral base address.
 */
static inline void GPT_StopTimer(GPT_Type *base)
{
    base->CR &= ~GPT_CR_EN_MASK;
}

/*!
 * @name Read the timer period
 * @{
 */

/*!
 * @brief Reads the current GPT counting value.
 *
 * @param base GPT peripheral base address.
 * @return Current GPT counter value.
 */
static inline uint32_t GPT_GetCurrentTimerCount(GPT_Type *base)
{
    return base->CNT;
}

/*@}*/

/*!
 * @name GPT Input/Output Signal Control
 * @{
 */

/*!
 * @brief Set GPT operation mode of input capture channel.
 *
 * @param base GPT peripheral base address.
 * @param channel GPT capture channel (see @ref gpt_input_capture_channel_t typedef enumeration).
 * @param mode GPT input capture operation mode (see @ref gpt_input_operation_mode_t typedef enumeration).
 */
static inline void GPT_SetInputOperationMode(GPT_Type *base,
                                             gpt_input_capture_channel_t channel,
                                             gpt_input_operation_mode_t mode)
{
    assert(channel <= kGPT_InputCapture_Channel2);

    base->CR = (base->CR & ~(GPT_CR_IM1_MASK << (channel * 2))) | (GPT_CR_IM1(mode) << (channel * 2));
}

/*!
 * @brief Get GPT operation mode of input capture channel.
 *
 * @param base GPT peripheral base address.
 * @param channel GPT capture channel (see @ref gpt_input_capture_channel_t typedef enumeration).
 * @return GPT input capture operation mode (see @ref gpt_input_operation_mode_t typedef enumeration).
 */
static inline gpt_input_operation_mode_t GPT_GetInputOperationMode(GPT_Type *base, gpt_input_capture_channel_t channel)
{
    assert(channel <= kGPT_InputCapture_Channel2);

    return (gpt_input_operation_mode_t)((base->CR >> (GPT_CR_IM1_SHIFT + channel * 2)) &
                                        (GPT_CR_IM1_MASK >> GPT_CR_IM1_SHIFT));
}

/*!
 * @brief Get GPT input capture value of certain channel.
 *
 * @param base GPT peripheral base address.
 * @param channel GPT capture channel (see @ref gpt_input_capture_channel_t typedef enumeration).
 * @return GPT input capture value.
 */
static inline uint32_t GPT_GetInputCaptureValue(GPT_Type *base, gpt_input_capture_channel_t channel)
{
    assert(channel <= kGPT_InputCapture_Channel2);

    return *(&base->ICR[0] + channel);
}

/*!
 * @brief Set GPT operation mode of output compare channel.
 *
 * @param base GPT peripheral base address.
 * @param channel GPT output compare channel (see @ref gpt_output_compare_channel_t typedef enumeration).
 * @param mode GPT output operation mode (see @ref gpt_output_operation_mode_t typedef enumeration).
 */
static inline void GPT_SetOutputOperationMode(GPT_Type *base,
                                              gpt_output_compare_channel_t channel,
                                              gpt_output_operation_mode_t mode)
{
    assert(channel <= kGPT_OutputCompare_Channel3);

    base->CR = (base->CR & ~(GPT_CR_OM1_MASK << (channel * 3))) | (GPT_CR_OM1(mode) << (channel * 3));
}

/*!
 * @brief Get GPT operation mode of output compare channel.
 *
 * @param base GPT peripheral base address.
 * @param channel GPT output compare channel (see @ref gpt_output_compare_channel_t typedef enumeration).
 * @return GPT output operation mode (see @ref gpt_output_operation_mode_t typedef enumeration).
 */
static inline gpt_output_operation_mode_t GPT_GetOutputOperationMode(GPT_Type *base,
                                                                     gpt_output_compare_channel_t channel)
{
    assert(channel <= kGPT_OutputCompare_Channel3);

    return (gpt_output_operation_mode_t)((base->CR >> (GPT_CR_OM1_SHIFT + channel * 3)) &
                                         (GPT_CR_OM1_MASK >> GPT_CR_OM1_SHIFT));
}

/*!
 * @brief Set GPT output compare value of output compare channel.
 *
 * @param base GPT peripheral base address.
 * @param channel GPT output compare channel (see @ref gpt_output_compare_channel_t typedef enumeration).
 * @param value GPT output compare value.
 */
static inline void GPT_SetOutputCompareValue(GPT_Type *base, gpt_output_compare_channel_t channel, uint32_t value)
{
    assert(channel <= kGPT_OutputCompare_Channel3);

    *(&base->OCR[0] + channel) = value;
}

/*!
 * @brief Get GPT output compare value of output compare channel.
 *
 * @param base GPT peripheral base address.
 * @param channel GPT output compare channel (see @ref gpt_output_compare_channel_t typedef enumeration).
 * @return GPT output compare value.
 */
static inline uint32_t GPT_GetOutputCompareValue(GPT_Type *base, gpt_output_compare_channel_t channel)
{
    assert(channel <= kGPT_OutputCompare_Channel3);

    return *(&base->OCR[0] + channel);
}

/*!
 * @brief Force GPT output action on output compare channel, ignoring comparator.
 *
 * @param base GPT peripheral base address.
 * @param channel GPT output compare channel (see @ref gpt_output_compare_channel_t typedef enumeration).
 */
static inline void GPT_ForceOutput(GPT_Type *base, gpt_output_compare_channel_t channel)
{
    assert(channel <= kGPT_OutputCompare_Channel3);

    base->CR |= (GPT_CR_FO1_MASK << channel);
}

/*@}*/

/*!
 * @name GPT Interrupt and Status Interface
 * @{
 */

/*!
 * @brief Enables the selected GPT interrupts.
 *
 * @param base GPT peripheral base address.
 * @param mask The interrupts to enable. This is a logical OR of members of the
 *             enumeration ::gpt_interrupt_enable_t
 */
static inline void GPT_EnableInterrupts(GPT_Type *base, uint32_t mask)
{
    base->IR |= mask;
}

/*!
 * @brief Disables the selected GPT interrupts.
 *
 * @param base    GPT peripheral base address
 * @param mask    The interrupts to disable. This is a logical OR of members of the
 *                enumeration ::gpt_interrupt_enable_t
 */
static inline void GPT_DisableInterrupts(GPT_Type *base, uint32_t mask)
{
    base->IR &= ~mask;
}

/*!
 * @brief Gets the enabled GPT interrupts.
 *
 * @param base    GPT peripheral base address
 *
 * @return The enabled interrupts. This is the logical OR of members of the
 *         enumeration ::gpt_interrupt_enable_t
 */
static inline uint32_t GPT_GetEnabledInterrupts(GPT_Type *base)
{
    return (base->IR & (GPT_IR_OF1IE_MASK | GPT_IR_OF2IE_MASK | GPT_IR_OF3IE_MASK | GPT_IR_IF1IE_MASK |
                        GPT_IR_IF2IE_MASK | GPT_IR_ROVIE_MASK));
}

/*!
 * @name Status Interface
 * @{
 */

/*!
 * @brief Get GPT status flags.
 *
 * @param base GPT peripheral base address.
 * @param flags GPT status flag mask (see @ref gpt_status_flag_t for bit definition).
 * @return GPT status, each bit represents one status flag.
 */
static inline uint32_t GPT_GetStatusFlags(GPT_Type *base, gpt_status_flag_t flags)
{
    return base->SR & flags;
}

/*!
 * @brief Clears the GPT status flags.
 *
 * @param base GPT peripheral base address.
 * @param flags GPT status flag mask (see @ref gpt_status_flag_t for bit definition).
 */
static inline void GPT_ClearStatusFlags(GPT_Type *base, gpt_status_flag_t flags)
{
    base->SR = flags;
}

/*@}*/

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* _GPT_HW_H_ */
