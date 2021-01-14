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

#ifndef _FSL_CLOCK_H_

/*
 * Must be aligned with platform/imx8/soc/imx8mn/include/fsl_clock.h
 */
#define kCLOCK_Divider_None 0xFFFFFFFF

#define kCLOCK_Idx_RootM4 1         /*!< ARM Cortex-M4 Clock control name.*/
#define kCLOCK_Idx_RootAxi 16       /*!< AXI Clock control name.*/
#define kCLOCK_Idx_RootNoc 26       /*!< NOC Clock control name.*/
#define kCLOCK_Idx_RootAhb 32       /*!< AHB Clock control name.*/
#define kCLOCK_Idx_RootIpg 33       /*!< IPG Clock control name.*/
#define kCLOCK_Idx_RootAudioAhb 34  /*!< Audio AHB Clock control name.*/
#define kCLOCK_Idx_RootAudioIpg 35  /*!< Audio IPG Clock control name.*/
#define kCLOCK_Idx_RootDramAlt 64   /*!< DRAM ALT Clock control name.*/
#define kCLOCK_Idx_RootSai1 75  /*!< SAI1 Clock control name.*/
#define kCLOCK_Idx_RootSai2 76  /*!< SAI2 Clock control name.*/
#define kCLOCK_Idx_RootSai3 77  /*!< SAI3 Clock control name.*/
#define kCLOCK_Idx_RootSai4 78  /*!< SAI4 Clock control name.*/
#define kCLOCK_Idx_RootSai5 79  /*!< SAI5 Clock control name.*/
#define kCLOCK_Idx_RootSai6 80  /*!< SAI6 Clock control name.*/
#define kCLOCK_Idx_RootSpdif1 81  /*!< SPDIF1 Clock control name.*/
#define kCLOCK_Idx_RootSpdif2 82  /*!< SPDIF2 Clock control name.*/
#define kCLOCK_Idx_RootQspi 87  /*!< QSPI Clock control name.*/
#define kCLOCK_Idx_RootI2c1 90  /*!< I2C1 Clock control name.*/
#define kCLOCK_Idx_RootI2c2 91  /*!< I2C2 Clock control name.*/
#define kCLOCK_Idx_RootI2c3 92  /*!< I2C3 Clock control name.*/
#define kCLOCK_Idx_RootI2c4 93  /*!< I2C4 Clock control name.*/
#define kCLOCK_Idx_RootUart1 94  /*!< UART1 Clock control name.*/
#define kCLOCK_Idx_RootUart2 95  /*!< UART2 Clock control name.*/
#define kCLOCK_Idx_RootUart3 96  /*!< UART3 Clock control name.*/
#define kCLOCK_Idx_RootUart4 97  /*!< UART4 Clock control name.*/
#define kCLOCK_Idx_RootEcspi1 101  /*!< ECSPI1 Clock control name.*/
#define kCLOCK_Idx_RootEcspi2 102  /*!< ECSPI2 Clock control name.*/
#define kCLOCK_Idx_RootEcspi3 131  /*!< ECSPI3 Clock control name.*/
#define kCLOCK_Idx_RootPwm1 103  /*!< PWM1 Clock control name.*/
#define kCLOCK_Idx_RootPwm2 104  /*!< PWM2 Clock control name.*/
#define kCLOCK_Idx_RootPwm3 105  /*!< PWM3 Clock control name.*/
#define kCLOCK_Idx_RootPwm4 106  /*!< PWM4 Clock control name.*/
#define kCLOCK_Idx_RootGpt1 107  /*!< GPT1 Clock control name.*/
#define kCLOCK_Idx_RootGpt2 108  /*!< GPT2 Clock control name.*/
#define kCLOCK_Idx_RootGpt3 109  /*!< GPT3 Clock control name.*/
#define kCLOCK_Idx_RootGpt4 110  /*!< GPT4 Clock control name.*/
#define kCLOCK_Idx_RootGpt5 111  /*!< GPT5 Clock control name.*/
#define kCLOCK_Idx_RootGpt6 112  /*!< GPT6 Clock control name.*/
#define kCLOCK_Idx_RootWdog 114  /*!< WDOG Clock control name.*/
#define kCLOCK_Idx_RootPdm 132  /*!< PDM Clock control name.*/
#define kCLOCK_Idx_RootSai7 134  /*!< SAI7 Clock control name.*/
#define kCLOCK_Idx_RootNone 0xFFFFFFFF

#endif  // _FSL_CLOCK_H_
