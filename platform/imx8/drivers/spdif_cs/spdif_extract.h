/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _SPDIF_EXTRACT_H_
#define _SPDIF_EXTRACT_H_

#include <stdint.h>

/*!
 * @addtogroup dma
 * @{
 */

/*! @file */
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * APIs
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */
/*!
 * @brief Calculate the oversample bitclock.
 * @param sampleRate target sample rate.
 * @param osr oversample ratio
 * @retval bit clock frequency should be used.
 */
int oversampleBitClock(uint32_t sampleRate, uint32_t osr);

/*!
 * @brief convert oversample data to biphase data.
 * @param oversampleBuffer
 * @param oversampleBufferSize
 * @param biphaseBuffer
 * @param biphase buffer size
 * @param oversample ratio
 * @retval recoverd biphase numbers.
 */
int oversampleToBiphase(
    uint8_t *oversampleBuffer, uint32_t oversampleSize, uint8_t *biphaseBuffer, uint32_t biphaseSize, uint32_t osr);

/*!
 * @brief biphase buffer recover to the IEC90658 format.
 * @param biphase buffer
 * @param biphase buffer size
 * @param startAddr biphase block start address
 * @retval valid biphase buffer size.
 */
int biphaseRecovery(uint8_t *biphaseBuffer, uint32_t biphaseBufferSize, uint8_t **startAddr);

/*!
 * @brief Convert biphase data to spdif data.
 * @param biphase buffer
 * @param biphase buffer size
 * @param spdif buffer
 * @param spdif buffer size
 * @retval return converted spdif data size.
 */
int biphaseToSPDIF(uint8_t *biphaseBuffer, uint32_t biphaseBufferSize, uint8_t *spdifBuffer, uint32_t spdifBufferSize);

/*!
 * @brief extract spdif frame channel status.
 * @param spdif buffer
 * @param spdif buffer size
 * @param cs buffer.
 * @param cs buffer size
 * @retval extracted spdif cs size in byte.
 */
int spdifFrameChannelStatus(uint8_t *spdifBuffer, uint32_t spdifBufferSize, uint8_t *csBuffer, uint32_t csBufferSize);

/*!
 * @brief extract spdif sub frame channel status.
 * @param spdif buffer
 * @param spdif buffer size
 * @param cs buffer.
 * @param cs buffer size
 * @retval extracted spdif cs size in byte.
 */
int spdifSubFrameChannelStatus(uint8_t *spdifBuffer,
                               uint32_t spdifBufferSize,
                               uint8_t *csBuffer,
                               uint32_t csBufferSize);

/* @} */

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/* @} */

#endif /*_SPDIF_EXTRACT_H_*/
