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

#ifndef _CCGR_CLOCK_H_
#define _CCGR_CLOCK_H_

/*! @brief CCM CCGR gate control. */
#define kCLOCK_None      0xFFFFFFFF 0xFFFFFFFF /*!< DEBUG Clock Gate.*/
#define kCLOCK_Debug        4U 32U  /*!< DEBUG Clock Gate.*/
#define kCLOCK_Dram         5U 64U  /*!< DRAM Clock Gate.*/
#define kCLOCK_Ecspi1       7U 101U  /*!< ECSPI1 Clock Gate.*/
#define kCLOCK_Ecspi2       8U 102U  /*!< ECSPI2 Clock Gate.*/
#define kCLOCK_Ecspi3       9U 131U  /*!< ECSPI3 Clock Gate.*/
#define kCLOCK_Gpio1        11U 33U  /*!< GPIO1 Clock Gate.*/
#define kCLOCK_Gpio2        12U 33U  /*!< GPIO2 Clock Gate.*/
#define kCLOCK_Gpio3        13U 33U  /*!< GPIO3 Clock Gate.*/
#define kCLOCK_Gpio4        14U 33U  /*!< GPIO4 Clock Gate.*/
#define kCLOCK_Gpio5        15U 33U  /*!< GPIO5 Clock Gate.*/
#define kCLOCK_Gpt1         16U 107U  /*!< GPT1 Clock Gate.*/
#define kCLOCK_Gpt2         17U 108U  /*!< GPT2 Clock Gate.*/
#define kCLOCK_Gpt3         18U 109U  /*!< GPT3 Clock Gate.*/
#define kCLOCK_Gpt4         19U 110U  /*!< GPT4 Clock Gate.*/
#define kCLOCK_Gpt5         20U 111U  /*!< GPT5 Clock Gate.*/
#define kCLOCK_Gpt6         21U 112U  /*!< GPT6 Clock Gate.*/
#define kCLOCK_I2c1         23U 90U  /*!< I2C1 Clock Gate.*/
#define kCLOCK_I2c2         24U 91U  /*!< I2C2 Clock Gate.*/
#define kCLOCK_I2c3         25U 92U  /*!< I2C3 Clock Gate.*/
#define kCLOCK_I2c4         26U 93U  /*!< I2C4 Clock Gate.*/
#define kCLOCK_Iomux0       27U 33U  /*!< IOMUX Clock Gate.*/
#define kCLOCK_Iomux1       28U 33U  /*!< IOMUX Clock Gate.*/
#define kCLOCK_Iomux2       29U 33U  /*!< IOMUX Clock Gate.*/
#define kCLOCK_Iomux3       30U 33U  /*!< IOMUX Clock Gate.*/
#define kCLOCK_Iomux4       31U 33U  /*!< IOMUX Clock Gate.*/
#define kCLOCK_Mu           33U 33U  /*!< MU Clock Gate.*/
#define kCLOCK_Ocram        35U 16U   /*!< OCRAM Clock Gate.*/
#define kCLOCK_OcramS       36U 32U  /*!< OCRAM S Clock Gate.*/
#define kCLOCK_Pwm1         40U 103U  /*!< PWM1 Clock Gate.*/
#define kCLOCK_Pwm2         41U 104U  /*!< PWM2 Clock Gate.*/
#define kCLOCK_Pwm3         42U 105U  /*!< PWM3 Clock Gate.*/
#define kCLOCK_Pwm4         43U 106U  /*!< PWM4 Clock Gate.*/
#define kCLOCK_Qspi         47U 87U  /*!< QSPI Clock Gate.*/
#define kCLOCK_Rdc          49U 33U  /*!< RDC Clock Gate.*/
#define kCLOCK_Sai1         51U 75U  /*!< SAI1 Clock Gate.*/
#define kCLOCK_Sai2         52U 76U  /*!< SAI2 Clock Gate.*/
#define kCLOCK_Sai3         53U 77U  /*!< SAI3 Clock Gate.*/
#define kCLOCK_Sai4         54U 78U  /*!< SAI4 Clock Gate.*/
#define kCLOCK_Sai5         55U 79U  /*!< SAI5 Clock Gate.*/
#define kCLOCK_Sai6         56U 80U  /*!< SAI6 Clock Gate.*/
#define kCLOCK_Sdma1        58U 33U  /*!< SDMA1 Clock Gate.*/
#define kCLOCK_Sdma2        59U 35U  /*!< SDMA2 Clock Gate.*/
#define kCLOCK_Sec_Debug    60U 33U  /*!< SEC_DEBUG Clock Gate.*/
#define kCLOCK_Sema42_1     61U 33U  /*!< RDC SEMA42 Clock Gate.*/
#define kCLOCK_Sema42_2     62U 33U  /*!< RDC SEMA42 Clock Gate.*/
#define kCLOCK_Sim_display  63U 16U  /*!< SIM_Display Clock Gate.*/
#define kCLOCK_Sim_m        65U 32U        /*!< SIM_M Clock Gate.*/
#define kCLOCK_Sim_main     66U 16U     /*!< SIM_MAIN Clock Gate.*/
#define kCLOCK_Sim_s        67U 32U        /*!< SIM_S Clock Gate.*/
#define kCLOCK_Sim_wakeup   68U 32U   /*!< SIM_WAKEUP Clock Gate.*/
#define kCLOCK_Uart1        73U 94U  /*!< UART1 Clock Gate.*/
#define kCLOCK_Uart2        74U 95U  /*!< UART2 Clock Gate.*/
#define kCLOCK_Uart3        75U 96U  /*!< UART3 Clock Gate.*/
#define kCLOCK_Uart4        76U 97U  /*!< UART4 Clock Gate.*/
#define kCLOCK_Usdhc1       81U 88U  /*!< USDHC1 Clock Gate.*/
#define kCLOCK_Usdhc2       82U 89U  /*!< USDHC2 Clock Gate.*/
#define kCLOCK_Wdog1        83U 114U  /*!< WDOG1 Clock Gate.*/
#define kCLOCK_Wdog2        84U 114U  /*!< WDOG2 Clock Gate.*/
#define kCLOCK_Wdog3        85U 114U  /*!< WDOG3 Clock Gate.*/
#define kCLOCK_Pdm          91U 132U     /*!< PDM Clock Gate.*/
#define kCLOCK_Usdhc3       94U 121U  /*!< USDHC3 Clock Gate.*/
#define kCLOCK_Sdma3        95U 35U    /*!< SDMA3 Clock Gate.*/
#define kCLOCK_TempSensor   98U 0xFFFF  /*!< TempSensor Clock Gate.*/

#define kCLOCK_AudioPll1Ctrl 0x9
#define kCLOCK_AudioPll2Ctrl 0x140009

#define kANALOG_PllRefOsc24M    0U      /*!< reference OSC 24M */
#define kANALOG_PllPadClk       1U      /*!< reference PAD CLK */

#endif  // _CCGR_CLOCK_H_
