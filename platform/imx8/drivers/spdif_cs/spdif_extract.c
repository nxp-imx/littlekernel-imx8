/*
 * Copyright 2019-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include "assert.h"
#include "debug.h"
#include <string.h>
#include "spdif_extract.h"

#define PRINTF(...) printlk(LK_NOTICE,__VA_ARGS__)
//#define SPDIF_EXTRACT_DEBUG

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define X_PREAMBLE 0xE2
#define Y_PREAMBLE 0xE4
#define Z_PREAMBLE 0xE8

#define X_1_PREAMBLE 0x1d
#define Y_1_PREAMBLE 0x1b
#define Z_1_PREAMBLE 0x17
/*
 * 1block = 192 frame = 192 * 2 * 4
 *
 */
#define SPDIF_BLOCK_LEN_BYTES (192 * 2 * 4 * 2)
#ifdef SPDIF_EXTRACT_DEBUG
#define SPDIF_LOG(...) PRINTF(__VA_ARGS__)
#else
#define SPDIF_LOG(...)
#endif
#define SPDIF_IEC_CS_BIT 30
#define SPDIF_BIT(n) (1UL << (n))
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!@brief edge struct */
typedef struct _edge_config
{
    uint16_t edgeCount; /*!< edge counts*/
    uint8_t edgeLevel;  /*!< edge level */
} edge_config_t;

#define SPDIF_SOFTWARE_EXTRACT_IGNORE_AUDIO_DATA_ERROR

/*******************************************************************************
 * Code
 ******************************************************************************/
static void leftShiftBuffer1bit(uint8_t *buffer, uint32_t size)
{
    uint32_t i         = 0U;
    uint8_t *startAddr = buffer;
    uint8_t temp = 0U, temp1 = 0U;

    for (i = 0U; i < size; i++)
    {
        temp = startAddr[i];
        temp <<= 1U;
        temp1 = startAddr[i + 1];
        temp |= (temp1 >> 7U) & 1U;
        startAddr[i] = temp;
    }
}

static int oversampleEdgeDetect(uint32_t *buffer, uint32_t startIndex, uint32_t bitSize, uint8_t *bit)
{
    uint16_t bitCount = 0U;
    uint32_t i        = 0U;
    uint8_t currentBit, lastBit = 0U;

    for (i = startIndex; i < bitSize; i++)
    {
        currentBit = (buffer[i / 32] >> (31 - i % 32)) & 1U;

        if (bitCount == 0)
        {
            lastBit = currentBit;
        }

        if (currentBit != lastBit)
        {
            *bit = lastBit;
            break;
        }
        else
        {
            bitCount++;
        }

        lastBit = currentBit;
    }

    return bitCount;
}

static int overSampleEdgeHandle(uint32_t *buffer,
                                uint32_t startIndex,
                                uint32_t bitSize,
                                uint32_t osr,
                                edge_config_t *edge,
                                uint32_t edgeCount,
                                uint32_t *actualBitCount)
{
    uint32_t i = 0U, j = 0U, startBit = startIndex;
    int result                     = 0U;
    edge_config_t edgeContainer[2] = {0};
    uint8_t lastBitLevel           = 0U;

    /* get last bit level */
    oversampleEdgeDetect(buffer, startBit - 1, bitSize, &lastBitLevel);

    for (i = 0U; i < edgeCount; i++)
    {
        edge[i].edgeCount = oversampleEdgeDetect(buffer, startBit, bitSize, &edge[i].edgeLevel);
        if (edge[i].edgeCount == 0U)
        {
            return -2;
        }
        startBit += edge[i].edgeCount;
        if (startBit >= bitSize)
        {
            return -1;
        }
        *actualBitCount += edge[i].edgeCount;
    }

    if ((edge[0].edgeCount + edge[1].edgeCount) % osr != 0U)
    {
        // SPDIF_LOG(" first edge count %d, second edge count %d\r\n", firstEdgeCount, edge[1].edgeCount);
        if (((edge[0].edgeCount + edge[1].edgeCount + 1) % osr) == 0U)
        {
            if (osr <= 2U)
            {
                for (j = 0U; j < osr * 8; j++)
                {
                    if ((bitSize > startBit) && ((bitSize - startBit) > (osr / 2)))
                    {
                        edgeContainer[0].edgeCount =
                            oversampleEdgeDetect(buffer, startBit, bitSize, &edgeContainer[0].edgeLevel);
                        startBit += edgeContainer[0].edgeCount;

                        if (((edge[0].edgeCount % osr) == 0U) &&
                            (((edge[1].edgeCount + edgeContainer[0].edgeCount) % osr) == 0U))
                        {
                            /* drop edge1 count */
                            *actualBitCount -= edge[1].edgeCount;
                            edge[1].edgeCount = 0U;
                            break;
                        }
                        else if ((((edgeContainer[0].edgeCount - 1) % osr) == 0U))
                        {
                            if (((edgeContainer[0].edgeCount == 1) && ((edgeContainer[0].edgeLevel == 1))) ||
                                ((edgeContainer[0].edgeCount == 1) && ((edgeContainer[0].edgeLevel == 0))))
                            {
                                *actualBitCount -= 1;
                                /* for the case: 1 0 0 1 0 0  */
                                if ((edge[0].edgeCount == 1U) && (edge[0].edgeLevel == 1))
                                {
                                    SPDIF_LOG("edge0 +1, edge1 -2\r\n");
                                    edge[0].edgeCount += 1U;
                                    edge[1].edgeCount -= 2U;
                                }
                                else
                                {
                                    if ((edge[0].edgeCount % osr) != 0U)
                                    {
                                        edge[0].edgeCount -= 1;
                                    }

                                    if ((edge[1].edgeCount % osr) != 0U)
                                    {
                                        edge[1].edgeCount -= 1;
                                    }
                                }
                            }
                            else
                            {
                                *actualBitCount += 1;
                            }

                            break;
                        }
                    }
                }
            }
            else
            {
                edgeContainer[0].edgeCount =
                    oversampleEdgeDetect(buffer, startBit, bitSize, &edgeContainer[0].edgeLevel);
                startBit += edgeContainer[0].edgeCount;
                edgeContainer[1].edgeCount = 0U;

                if (((edge[0].edgeCount + edge[1].edgeCount + 1) % osr) == 0U)
                {
                    if (((edgeContainer[0].edgeCount + edgeContainer[1].edgeCount - 1) % osr) == 0U)
                    {
                        *actualBitCount += 1;
                    }
                    else if (((edgeContainer[0].edgeCount + edgeContainer[1].edgeCount) % osr) == 0U)
                    {
                        /* DO NOTHING */
                    }
                    else
                    {
                        /*assert(false);*/
                    }
                }
            }

            if ((edge[0].edgeCount % osr) != 0U)
            {
                edge[0].edgeCount += 1;
            }

            if ((edge[1].edgeCount % osr) != 0U)
            {
                edge[1].edgeCount += 1;
            }
        }
        else if (((edge[0].edgeCount + edge[1].edgeCount - 1) % osr) == 0U)
        {
            if (osr > 2)
            {
                if (((edgeContainer[0].edgeCount + edgeContainer[1].edgeCount + 1) % osr) == 0U)
                {
                    *actualBitCount -= 1;
                    // SPDIF_LOG("Given 1: Edge count 0: %d, count 1: %d, count 2: %d, count 3: %d\r\n",
                    // edge[0].edgeCount, edge[1].edgeCount, edgeContainer[0].edgeCount, edgeContainer[1].edgeCount);
                }
                else if (((edgeContainer[0].edgeCount + edgeContainer[1].edgeCount) % osr) == 0U)
                {
                    /* do nothing */
                    // SPDIF_LOG("Edge count 0: %d, count 1: %d, count 2: %d, count 3: %d\r\n", edge[0].edgeCount,
                    // edge[1].edgeCount, edgeContainer[0].edgeCount, edgeContainer[1].edgeCount);
                }
                else
                {
                    /*assert(false);*/
                }

                if ((edge[0].edgeCount % osr) != 0U)
                {
                    edge[0].edgeCount -= 1;
                }

                if ((edge[1].edgeCount % osr) != 0U)
                {
                    edge[1].edgeCount -= 1;
                }
            }
        }
        else
        {
            return -1;
        }
    }
    else
    {
        if (((edge[0].edgeCount % osr) != 0U) && ((edge[1].edgeCount % osr) != 0U))
        {
            if (osr <= 2)
            {
                if (((edge[0].edgeCount % osr) == 1) && ((edge[1].edgeCount % osr) == 1))
                {
                    if ((edge[0].edgeCount < edge[1].edgeCount))
                    {
                        edge[0].edgeCount += 1U;
                        edge[1].edgeCount -= 1U;
                    }
                    else if ((edge[1].edgeCount < edge[0].edgeCount))
                    {
                        edge[0].edgeCount -= 1U;
                        edge[1].edgeCount += 1U;
                    }
                    else
                    {
                        /* first edge got high priority to extend 1 more bit count, except when both edge count is 1  */
                        if ((edge[0].edgeCount == 1) && (edge[1].edgeCount == 1))
                        {
                            if ((edge[0].edgeLevel == 1) && (lastBitLevel != 1))
                            {
                                edge[0].edgeCount += 1U;
                                edge[1].edgeCount -= 1U;
                            }
                            else
                            {
                                edge[0].edgeCount -= 1U;
                                edge[1].edgeCount += 1U;
                            }
                        }
                        else
                        {
                            /*if((lastBitLevel == 0) && (edge[0].edgeLevel == 1))
                           {
                               edge[0].edgeCount -= 1U;
                               edge[1].edgeCount += 1U;
                           }
                           else*/
                            {
                                edge[0].edgeCount += 1U;
                                edge[1].edgeCount -= 1U;
                            }
                        }
                    }
                }
            }
            else
            {
                if (((edge[0].edgeCount % osr) == 1) && ((osr - (edge[1].edgeCount % osr) == 1)))
                {
                    edge[0].edgeCount -= 1;
                    edge[1].edgeCount += 1;
                }
                else if (((edge[1].edgeCount % osr) == 1) && ((osr - (edge[0].edgeCount % osr) == 1)))
                {
                    edge[0].edgeCount += 1;
                    edge[1].edgeCount -= 1;
                }
                else
                {
                   return -1;
                }
            }
        }
    }

    return result;
}

#ifdef SPDIF_EXTRACT_DEBUG
void printBitsField(uint32_t *buffer, uint32_t startIndex, uint32_t endIndex)
{
    uint32_t i = 0U, currentBit = 0U;

    for (i = startIndex; i < endIndex; i++)
    {
        currentBit = (buffer[i / 32] >> (31 - i % 32)) & 1U;
        SPDIF_LOG("%d ", currentBit);
    }

    SPDIF_LOG("\r\n");
}

void bufferPrintOut32bit(uint8_t *buffer, uint32_t size)
{
    uint32_t i          = 0U;
    uint32_t *srcBuffer = (uint32_t *)buffer;

    for (i = 0U; i < size / 4; i++)
    {
        if (i % 4 == 0U)
        {
            SPDIF_LOG("\r\n");
        }
        SPDIF_LOG("0x%08x,", srcBuffer[i]);
    }
}
#endif

int oversampleToBiphase(
    uint8_t *oversampleBuffer, uint32_t oversampleSize, uint8_t *biphaseBuffer, uint32_t biphaseSize, uint32_t osr)
{
    assert(oversampleSize / osr == biphaseSize);

    int bitCount       = 0U;
    uint32_t bitOffset = 0U;
    uint8_t validBit   = 0U;
    uint32_t bitSize = oversampleSize * 8U, i = 0U, j = 0U, bitIndex = 0U, biphaseByteIndex = 0U;
    uint32_t *ob       = (uint32_t *)oversampleBuffer;
    uint8_t *bb        = (uint8_t *)biphaseBuffer;
    uint32_t recovBits = 7U, recovBytes = 0U;
    int result            = 0U;
    edge_config_t edge[2] = {0};

    /* drop first edge */
    bitIndex += oversampleEdgeDetect(ob, bitIndex, bitSize, &validBit);

    do
    {
        memset(edge, 0, sizeof(edge_config_t) * 2U);

        result = overSampleEdgeHandle(ob, bitIndex, bitSize, osr, edge, 2U, &bitOffset);
        if (result == -2)
        {
            SPDIF_LOG(" unrecoverable edge is detected, start index %d, partial byte %d, bits %d\r\n", bitIndex,
                      recovBytes, recovBits);
            return -1;
        }
        else if (result == -1)
        {
            break;
        }

        for (i = 0U; i < 2U; i++)
        {
            bitCount = edge[i].edgeCount;
            validBit = edge[i].edgeLevel;
            for (j = 0U; j < (bitCount / osr); j++)
            {
                recovBytes |= validBit << recovBits;

                if (recovBits == 0)
                {
                    recovBits = 7U;
                    {
                        if (biphaseByteIndex >= biphaseSize)
                        {
                            break;
                        }
                        bb[biphaseByteIndex] = recovBytes;
                        biphaseByteIndex++;
                    }
                    recovBytes = 0U;

                    continue;
                }
                recovBits--;
            }
        }

        bitIndex += bitOffset;
        bitOffset = 0U;

    } while (bitIndex < bitSize);

    return biphaseByteIndex;
}

int oversampleBitClock(uint32_t sampleRate, uint32_t osr)
{
    return sampleRate * 32 * 2 * 2 * osr;
}

static int biphaseDataRecovery(uint8_t *biphaseBuffer, uint32_t biphaseBufferSize)
{
    uint32_t i = 0U, j = 0U;
    uint64_t frame = 0U, testPattern = 0x6000000000000000U, testResult = 0U;

    for (i = 0; i < biphaseBufferSize; i += 8U)
    {
        for (j = 1U; j < 8U; j++)
        {
            frame |= (uint64_t)biphaseBuffer[i + j] << ((8 - j) * 8U);
        }

        for (j = 0; j < 27; j++)
        {
            testResult = frame & testPattern;
#ifdef SPDIF_SOFTWARE_EXTRACT_IGNORE_AUDIO_DATA_ERROR
            if (((testResult == 0) || (testResult == testPattern)) && ((j >= 23))) /* ignore the audio data error */
#else
            if ((testResult == 0) || (testResult == testPattern))
#endif
            {
                // SPDIF_LOG("data error found in frame %x, index %d\r\n", frame, i);
                return -1;
            }
            testPattern >>= 2U;
        }
        frame       = 0U;
        testPattern = 0x6000000000000000U;
    }

    return 0;
}

/*
static int biphaseDataRecovery(uint8_t *biphaseBuffer, uint32_t biphaseBufferSize)
{
    uint32_t i = 0U, j = 0U, m = 0U;
    uint8_t frame = 0U, testPattern = 0xC0U, testResult = 0U;

    for (i = 0; i < biphaseBufferSize; i += 8U)
    {
        for (j = 1U; j < 8U; j++)
        {
            frame = biphaseBuffer[i + j] << 1;
            if(j != 7U)
            {
                frame |= (biphaseBuffer[i + j + 1] >> 7U) & 1U;
            }
        }

        for (m = 1; m < 7; m++)
        {
            if((j == 7U) && (m == 6U))
            {
                break;
            }
            testResult = frame & testPattern;

            if ((testResult == 0) || (testResult == testPattern))
            {
                SPDIF_LOG("data error found in frame %x, index %d\r\n", frame, i);
                return -1;
            }
            testPattern >>= 1U;
        }
        frame       = 0U;
        testPattern = 0xC0U;
    }

    return 0;
}*/

static int biphasePreambleRecovery(uint8_t *preamble, uint32_t size)
{
    uint32_t i            = 0U;
    uint32_t nextPreamble = 0U;

    if (((preamble[i] & Z_PREAMBLE) == preamble[i]) || ((preamble[i] & Z_PREAMBLE) == Z_PREAMBLE))
    {
        nextPreamble = Y_PREAMBLE;
    }
    else if (((preamble[i] & Z_1_PREAMBLE) == preamble[i]) || ((preamble[i] & Z_1_PREAMBLE) == Z_1_PREAMBLE))
    {
        nextPreamble = Y_1_PREAMBLE;
    }
    else
    {
        SPDIF_LOG("preamble recovery FAILED\r\n");
        return -1;
    }

    for (i = 8U; i < size; i += 8)
    {
        if (preamble[i] != nextPreamble)
        {
            SPDIF_LOG("Correct preamble found index %d, from value %x, to value %x\r\n", i, preamble[i], nextPreamble);
            preamble[i] = nextPreamble;
        }

        if (Y_1_PREAMBLE == nextPreamble)
        {
            nextPreamble = X_1_PREAMBLE;
        }
        else if (Y_PREAMBLE == nextPreamble)
        {
            nextPreamble = X_PREAMBLE;
        }
        else if (X_PREAMBLE == nextPreamble)
        {
            nextPreamble = Y_PREAMBLE;
        }
        else if (X_1_PREAMBLE == nextPreamble)
        {
            nextPreamble = Y_1_PREAMBLE;
        }
        else
        {
            return -1;
        }
    }

    return 0U;
}

static int biphaseDetectValidBlock(uint8_t *biphaseBuffer, uint32_t biphaseBufferSize)
{
    uint32_t i          = 0U;
    uint32_t nextZ      = 0U;
    uint32_t startIndex = 0U;
    bool validZDetected = false;

    for (i = 0; i < biphaseBufferSize; i++)
    {
        nextZ = 0U;

        if (biphaseBuffer[i] == Z_PREAMBLE)
        {
            nextZ = Z_PREAMBLE;
        }
        else if (biphaseBuffer[i] == Z_1_PREAMBLE)
        {
            nextZ = Z_1_PREAMBLE;
        }
        else
        {
            continue;
        }

        if (nextZ != 0U)
        {
            if ((i + SPDIF_BLOCK_LEN_BYTES) < biphaseBufferSize)
            {
                if ((biphaseBuffer[i + SPDIF_BLOCK_LEN_BYTES] == nextZ) ||
                    ((biphaseBuffer[i + SPDIF_BLOCK_LEN_BYTES] & nextZ) == biphaseBuffer[i + SPDIF_BLOCK_LEN_BYTES]) ||
                    ((biphaseBuffer[i + SPDIF_BLOCK_LEN_BYTES] & nextZ) == nextZ))
                {
                    startIndex     = i;
                    validZDetected = true;
                }
                else
                {
                    SPDIF_LOG("bad z1 block found, %d\r\n", biphaseBuffer[i + SPDIF_BLOCK_LEN_BYTES]);
                    continue;
                }
            }
            else
            {
                if ((i > SPDIF_BLOCK_LEN_BYTES) &&
                    (((biphaseBuffer[i - SPDIF_BLOCK_LEN_BYTES] & nextZ) == biphaseBuffer[i - SPDIF_BLOCK_LEN_BYTES]) ||
                     ((biphaseBuffer[i - SPDIF_BLOCK_LEN_BYTES] & nextZ) == nextZ)))
                {
                    startIndex     = i - SPDIF_BLOCK_LEN_BYTES;
                    validZDetected = true;
                }
                else
                {
                    SPDIF_LOG("bad z2 block found, %d\r\n", biphaseBuffer[i - SPDIF_BLOCK_LEN_BYTES]);
                    return -1;
                }
            }

            if (validZDetected)
            {
                validZDetected = false;

                if ((biphaseDataRecovery(&biphaseBuffer[startIndex], SPDIF_BLOCK_LEN_BYTES) != -1) &&
                    (biphasePreambleRecovery(&biphaseBuffer[startIndex], SPDIF_BLOCK_LEN_BYTES) != -1))
                {
                    return i;
                }

                continue;
            }
        }
    }

    return -1;
}

int biphaseRecovery(uint8_t *biphaseBuffer, uint32_t biphaseBufferSize, uint8_t **startAddr)
{
    uint32_t i          = 0U;
    int j               = 0U;
    uint32_t shiftIndex = 0U;

    (void) shiftIndex;
    for (i = 0U; i < 8; i++)
    {
        j = biphaseDetectValidBlock(biphaseBuffer, biphaseBufferSize);

        if (j != -1)
        {
            shiftIndex = i;
            *startAddr = &biphaseBuffer[j];

            break;
        }

        leftShiftBuffer1bit(biphaseBuffer, biphaseBufferSize);
    }

    SPDIF_LOG("biphase shift index %d,\r\n", shiftIndex);

    return i == 8 ? -1 : SPDIF_BLOCK_LEN_BYTES;
}

int biphaseToSPDIF(uint8_t *biphaseBuffer, uint32_t biphaseBufferSize, uint8_t *spdifBuffer, uint32_t spdifBufferSize)
{
    assert(spdifBufferSize >= biphaseBufferSize / 2);

    uint32_t i = 0U, j = 0U;
    uint16_t temp = 0U, b0 = 0, b1 = 0, b2 = 0, b3 = 0, rawBitIndex = 0U;
    uint16_t *rawPointer = (uint16_t *)spdifBuffer;
    uint32_t *biphase    = (uint32_t *)biphaseBuffer;

    for (i = 0U; i < biphaseBufferSize / 4; i++)
    {
        for (j = 0U; j < 32; j += 8)
        {
            b0 = (biphase[i] >> j) & 1U;
            b1 = (biphase[i] >> (j + 1)) & 1U;
            b2 = (biphase[i] >> (j + 2)) & 1U;
            b3 = (biphase[i] >> (j + 3)) & 1U;

            if (b0 != b1)
            {
                temp |= 1 << (rawBitIndex + 3);
            }

            if (b3 != b2)
            {
                temp |= 1 << (rawBitIndex + 2);
            }

            b0 = (biphase[i] >> (j + 4)) & 1U;
            b1 = (biphase[i] >> (j + 5)) & 1U;
            b2 = (biphase[i] >> (j + 6)) & 1U;
            b3 = (biphase[i] >> (j + 7)) & 1U;

            if (b0 != b1)
            {
                temp |= 1 << (rawBitIndex + 1);
            }

            if (b3 != b2)
            {
                temp |= 1 << rawBitIndex;
            }

            rawBitIndex += 4;
        }

        *rawPointer++ = temp;
        temp          = 0U;
        rawBitIndex   = 0U;
    }

    return biphaseBufferSize / 2;
}

int spdifFrameChannelStatus(uint8_t *spdifBuffer, uint32_t spdifBufferSize, uint8_t *csBuffer, uint32_t csBufferSize)
{
    assert(csBufferSize >= 48);
    assert(spdifBufferSize >= (192 * 2 * 4));

    size_t i;
    uint32_t *frames = (uint32_t *)spdifBuffer;
    uint32_t frame0;

    for (i = 0; i < 192; i++)
    {
        frame0 = frames[0];
        if (frame0 & SPDIF_BIT(SPDIF_IEC_CS_BIT))
            csBuffer[i / 8] |= SPDIF_BIT(i % 8);
        else
            csBuffer[i / 8] &= ~SPDIF_BIT(i % 8);

        frames += 1;
    }

    return 0;
}

int spdifSubFrameChannelStatus(uint8_t *spdifBuffer, uint32_t spdifBufferSize, uint8_t *csBuffer, uint32_t csBufferSize)
{
    assert(csBufferSize >= 24);
    assert(spdifBufferSize >= (192 * 2 * 4));

    size_t i;
    uint32_t *sframes = (uint32_t *)spdifBuffer;
    uint32_t sframe0;

    for (i = 0; i < 192; i++)
    {
        sframe0 = sframes[0];
        if (sframe0 & SPDIF_BIT(SPDIF_IEC_CS_BIT))
            csBuffer[i / 8] |= SPDIF_BIT(i % 8);
        else
            csBuffer[i / 8] &= ~SPDIF_BIT(i % 8);

        sframes += 2;
    }

    return 0;
}
