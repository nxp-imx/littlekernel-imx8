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

#ifndef _PWM_CLOCK_H_

/*! @brief Root clock select enumeration for PWM peripheral. */
#define kCLOCK_PwmRootmuxOsc24M   0U          /*!< PWM Clock from OSC 24M.*/
#define kCLOCK_PwmRootmuxSysPll2Div10   1U    /*!< PWM Clock from SYSTEM PLL2 divided by 10.*/
#define kCLOCK_PwmRootmuxSysPll1Div5   2U     /*!< PWM Clock from SYSTEM PLL1 divided by 5.*/
#define kCLOCK_PwmRootmuxSysPll1Div20   3U    /*!< PWM Clock from SYSTEM PLL1 divided by 20.*/
#define kCLOCK_PwmRootmuxSystemPll3   4U      /*!< PWM Clock from SYSTEM PLL3.*/
#define kCLOCK_PwmRootmuxExtClk12   5U        /*!< PWM Clock from External Clock1, External Clock2.*/
#define kCLOCK_PwmRootmuxSystemPll1Div10   6U /*!< PWM Clock from SYSTEM PLL1 divided by 10.*/
#define kCLOCK_PwmRootmuxVideoPll1   7U       /*!< PWM Clock from VIDEO PLL1.*/

#endif  // _PWM_CLOCK_H_
