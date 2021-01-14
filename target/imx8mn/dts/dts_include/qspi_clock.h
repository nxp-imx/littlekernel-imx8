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

#ifndef _QSPI_CLOCK_H_

/*! @brief Root clock select enumeration for QSPI peripheral. */
#define kCLOCK_QspiRootmuxOsc24M   0U      /*!< ARM QSPI Clock from OSC 24M.*/
#define kCLOCK_QspiRootmuxSysPll1Div2   1U /*!< ARM QSPI Clock from SYSTEM PLL1 divided by 2.*/
#define kCLOCK_QspiRootmuxSysPll2Div3   2U /*!< ARM QSPI Clock from SYSTEM PLL2 divided by 3.*/
#define kCLOCK_QspiRootmuxSysPll2Div2   3U /*!< ARM QSPI Clock from SYSTEM PLL2 divided by 2.*/
#define kCLOCK_QspiRootmuxAudioPll2   4U   /*!< ARM QSPI Clock from AUDIO PLL2.*/
#define kCLOCK_QspiRootmuxSysPll1Div3   5U /*!< ARM QSPI Clock from SYSTEM PLL1 divided by 3 */
#define kCLOCK_QspiRootmuxSysPll3   6      /*!< ARM QSPI Clock from SYSTEM PLL3.*/
#define kCLOCK_QspiRootmuxSysPll1Div8   7U /*!< ARM QSPI Clock from SYSTEM PLL1 divided by 8.*/

#endif  // _QSPI_CLOCK_H_
