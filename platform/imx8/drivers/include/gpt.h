/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017, 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _GPT_H_
#define _GPT_H_

#include <dev/class/gpt.h>

/*!
 * @addtogroup gpt
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief List of clock sources
 * @note Actual number of clock sources is SoC dependent
 */
enum gpt_clock_source
{
    kGPT_ClockSource_Off = 0U,      /*!< GPT Clock Source Off.*/
    kGPT_ClockSource_Periph = 1U,   /*!< GPT Clock Source from Peripheral Clock.*/
    kGPT_ClockSource_HighFreq = 2U, /*!< GPT Clock Source from High Frequency Reference Clock.*/
    kGPT_ClockSource_Ext = 3U,      /*!< GPT Clock Source from external pin.*/
    kGPT_ClockSource_LowFreq = 4U,  /*!< GPT Clock Source from Low Frequency Reference Clock.*/
    kGPT_ClockSource_Osc = 5U,      /*!< GPT Clock Source from Crystal oscillator.*/
};
typedef enum gpt_clock_source gpt_clock_source_t;

/*! @brief List of input capture channel number. */
enum gpt_input_capture_channel
{
    kGPT_InputCapture_Channel1 = 0U, /*!< GPT Input Capture Channel1.*/
    kGPT_InputCapture_Channel2 = 1U, /*!< GPT Input Capture Channel2.*/
};
typedef enum gpt_input_capture_channel gpt_input_capture_channel_t;

/*! @brief List of input capture operation mode. */
enum gpt_input_operation_mode
{
    kGPT_InputOperation_Disabled = 0U, /*!< Don't capture.*/
    kGPT_InputOperation_RiseEdge = 1U, /*!< Capture on rising edge of input pin.*/
    kGPT_InputOperation_FallEdge = 2U, /*!< Capture on falling edge of input pin.*/
    kGPT_InputOperation_BothEdge = 3U, /*!< Capture on both edges of input pin.*/
};
typedef enum gpt_input_operation_mode gpt_input_operation_mode_t;

/*! @brief List of output compare channel number. */
enum gpt_output_compare_channel
{
    kGPT_OutputCompare_Channel1 = 0U, /*!< Output Compare Channel1.*/
    kGPT_OutputCompare_Channel2 = 1U, /*!< Output Compare Channel2.*/
    kGPT_OutputCompare_Channel3 = 2U, /*!< Output Compare Channel3.*/
};
typedef enum gpt_output_compare_channel gpt_output_compare_channel_t;

/*! @brief List of output compare operation mode. */
enum gpt_output_operation_mode
{
    kGPT_OutputOperation_Disconnected = 0U, /*!< Don't change output pin.*/
    kGPT_OutputOperation_Toggle = 1U,       /*!< Toggle output pin.*/
    kGPT_OutputOperation_Clear = 2U,        /*!< Set output pin low.*/
    kGPT_OutputOperation_Set = 3U,          /*!< Set output pin high.*/
    kGPT_OutputOperation_Activelow = 4U,    /*!< Generate a active low pulse on output pin.*/
};
typedef enum gpt_output_operation_mode gpt_output_operation_mode_t;

/*! @brief Structure to configure the running mode. */
struct gpt_config
{
    gpt_clock_source_t clockSource; /*!< clock source for GPT module. */
    uint32_t divider;               /*!< clock divider (prescaler+1) from clock source to counter. */
    bool enableFreeRun;             /*!< true: FreeRun mode, false: Restart mode. */
    bool enableRunInWait;           /*!< GPT enabled in wait mode. */
    bool enableRunInStop;           /*!< GPT enabled in stop mode. */
    bool enableRunInDoze;           /*!< GPT enabled in doze mode. */
    bool enableRunInDbg;            /*!< GPT enabled in debug mode. */
    bool enableMode;                /*!< true:  counter reset to 0 when enabled;
                                    false: counter retain its value when enabled. */
};
typedef struct gpt_config gpt_config_t;

/*! @brief Structure to configure the capture mode. */
struct gpt_capture_config
{
    gpt_input_capture_channel_t channel; /*!< capture channel configuration. */
    gpt_input_operation_mode_t mode;     /*!< capture mode. */
    bool irq_enable; /*!< IRQ enable */
};
typedef struct gpt_capture_config gpt_capture_config_t;


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*! @brief Sets GPT driver configuration. */
status_t gpt_open(int id, struct gpt_config *cfg, gpt_timer_callback_t callback, void *userData);
/*! @brief Allows GPT driver to be reconfigured. */
status_t gpt_close(int id);
/*! @brief Stop the counter. */
status_t gpt_stop(int id);
/*! @brief Start the counter. */
status_t gpt_start(int id);
/*! @brief Get GPT counter value. */
status_t gpt_get_counter(int id, uint32_t *counter);
/*! @brief Setup capture. */
status_t gpt_setup_capture(int id, struct gpt_capture_config *cfg, gpt_capture_callback_t callback, void *userData);
/*! @brief Get capture counter value. */
status_t gpt_get_capture(int id, unsigned int channel, uint32_t *counter);
/*! @brief Enable/Disable capture. */
status_t gpt_enable_capture(int id, unsigned int channel, bool on);

/*! @} */

#endif /* _GPT_H_ */
