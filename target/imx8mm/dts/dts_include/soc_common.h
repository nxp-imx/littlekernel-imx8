/*
 * The Clear BSD License
 * Copyright 2020 NXP
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

#ifndef _SOC_COMMON_H_
#define _SOC_COMMON_H_

#define SAI_BIT_CLOCK_POLARITY_ACTIVE_HIGH              0
#define SAI_BIT_CLOCK_POLARITY_ACTIVE_LOW               1
#define SAI_BIT_CLOCK_POLARITY_DEFAULT                  2

#define SAI_MCLK_SELECT_BUS_CLOCK                       0
#define SAI_MCLK_SELECT_MCLK1                           1
#define SAI_MCLK_SELECT_MCLK2                           2
#define SAI_MCLK_SELECT_MCLK3                           3

#define SAI_FORMAT_LEFT_JUSTIFIED                       0
#define SAI_FORMAT_RIGHT_JUSTIFIED                      1
#define SAI_FORMAT_I2S                                  2
#define SAI_FORMAT_BUS_PCMA                             3
#define SAI_FORMAT_BUS_PCMB                             4
#define SAI_FORMAT_RX_BUS_I2S_AES3                      5
#define SAI_FORMAT_RX_BUS_I2S_PDM                       6

#define SAI_SYNC_ASYNC                                  0
#define SAI_SYNC_SYNC_TO_OTHER_DIRECTION                1
#define SAI_SYNC_SYNC_TO_SAME_DIRECTION_OTHER_SAI       2
#define SAI_SYNC_SYNC_TO_OTHER_DIRECTION_OTHER_SAI      3

#define SDMA_PERIPHERAL_TYPE_MEMORY                     0
#define SDMA_PERIPHERAL_TYPE_UART                       1
#define SDMA_PERIPHERAL_TYPE_UART_SPBA                  2
#define SDMA_PERIPHERAL_TYPE_SPDIF                      3
#define SDMA_PERIPHERAL_TYPE_NORMAL                     4
#define SDMA_PERIPHERAL_TYPE_NORMAL_SPBA                5
#define SDMA_PERIPHERAL_TYPE_MULTI_FIFO_PDM             6
#define SDMA_PERIPHERAL_TYPE_MULTI_FIFO_SAI_RX          7
#define SDMA_PERIPHERAL_TYPE_MULTI_FIFO_SAI_TX          8
#define SDMA_PERIPHERAL_TYPE_OPT_8_FIFO_SAI_TX          9
#define SDMA_PERIPHERAL_TYPE_OPT_4_FIFO_SAI_RX          10
#define SDMA_PERIPHERAL_TYPE_OPT_2_LANE_MIC             11
#define SDMA_PERIPHERAL_TYPE_OPT_4_LANE_MIC             12
#define SDMA_PERIPHERAL_TYPE_MULTI_SAI_TX               13
#define SDMA_PERIPHERAL_TYPE_MULTI_SAI_RX               14

#define SDMA_MODE_INTERMEDIATE_BUF                      0
#define SDMA_MODE_ZEROCOPY_UNCACHED_BUF                 1
#define SDMA_MODE_ZEROCOPY_CACHED_BUF                   2

#define SDMA_REQ_SAI1_RX                                0
#define SDMA_REQ_SAI1_TX                                1
#define SDMA_REQ_SAI2_RX                                2
#define SDMA_REQ_SAI2_TX                                3
#define SDMA_REQ_SAI3_RX                                4
#define SDMA_REQ_SAI3_TX                                5
#define SDMA_REQ_SAI4_RX                                4
#define SDMA_REQ_SAI4_TX                                5
#define SDMA_REQ_SAI5_RX                                8
#define SDMA_REQ_SAI5_TX                                9
#define SDMA_REQ_SAI6_RX                                10
#define SDMA_REQ_SAI6_TX                                11
#define SDMA_REQ_SPDIF_RX                               28
#define SDMA_REQ_SPDIF_TX                               29

#endif  // _SOC_COMMON_H_
