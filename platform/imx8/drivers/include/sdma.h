/*
 * Copyright 2016, 2019-2020 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _SDMA_H_
#define _SDMA_H_

#include "fsl_common.h"
#include <dev/dma.h>

/* ----------------------------------------------------------------------------
   -- SDMAARM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SDMAARM_Peripheral_Access_Layer SDMAARM Peripheral Access Layer
 * @{
 */
#ifndef __IO
#define __IO volatile
#endif
#ifndef __I
#define __I volatile const
#endif
#ifndef __O
#define __O volatile
#endif

/*!
 * @}
 */ /* end of group SDMAARM_Peripheral_Access_Layer */

/*!
 * @addtogroup sdma
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief SDMA transfer configuration */
typedef enum _sdma_transfer_size
{
    kSDMA_TransferSize1Bytes = 0x1U, /*!< Source/Destination data transfer size is 1 byte every time */
    kSDMA_TransferSize2Bytes = 0x2U, /*!< Source/Destination data transfer size is 2 bytes every time */
    kSDMA_TransferSize4Bytes = 0x0U, /*!< Source/Destination data transfer size is 4 bytes every time */
} sdma_transfer_size_t;


/*! @brief SDMA transfer type */
enum sdma_transfer_type
{
    kSDMA_MemoryToMemory = 0x0U, /*!< Transfer from memory to memory */
    kSDMA_PeripheralToMemory,    /*!< Transfer from peripheral to memory */
    kSDMA_MemoryToPeripheral,    /*!< Transfer from memory to peripheral */
};
typedef enum sdma_transfer_type sdma_transfer_type_t;

/*! @brief Peripheral type use SDMA */
enum sdma_peripheral
{
    kSDMA_PeripheralTypeMemory = 0x0, /*!< Peripheral DDR memory */
    kSDMA_PeripheralTypeUART,         /*!< UART use SDMA */
    kSDMA_PeripheralTypeUART_SP,      /*!< UART instance in SPBA use SDMA */
    kSDMA_PeripheralTypeSPDIF,        /*!< SPDIF use SDMA */
    kSDMA_PeripheralNormal,           /*!< Normal peripheral use SDMA */
    kSDMA_PeripheralNormal_SP,        /*!< Normal peripheral in SPBA use SDMA */
    kSDMA_PeripheralMultiFifoPDM,     /*!< multi fifo PDM */
    kSDMA_PeripheralMultiFifoSaiRX,   /*!< multi fifo sai rx use SDMA */
    kSDMA_PeripheralMultiFifoSaiTX,   /*!< multi fifo sai tx use SDMA */
    kSDMA_PeripheralOpt8FifoSaiTX,    /*!< 8-lane SHP to SAI (Primary
                                           Output) 2-Interleave
                                           64-Samples/FIFO Watermark */
    kSDMA_PeripheralOpt4FifoSaiRX,    /*!< 4-lane SAI to SHP (HDMI Input)
                                           2-Interleave 64-Samples/FIFO Watermark */
    kSDMA_PeripheralOpt2LaneMic,      /*!< 2-lane SAI to SHP (2-Mic Input)
                                           1-Interleave 64-Samples/FIFO Watermark */
    kSDMA_PeripheralOpt4LaneMic,      /*!< 4-lane SAI to SHP (4-Mic Input)
                                           1-Interleave 64-Samples/FIFO Watermark */
    kSDMA_PeripheralMultiSaiTX,       /*!< Synchronized SAI tx */
    kSDMA_PeripheralMultiSaiRX,       /*!< Synchronized SAI rx */
};

typedef enum sdma_peripheral sdma_peripheral_t;

/*! @brief SDMA multi fifo configurations.*/
typedef struct _sdma_multi_fifo_config
{
    uint8_t fifoNums;   /*!< fifo numbers */
    uint8_t fifoOffset; /*!< offset between multi fifo data register address */
    uint8_t sampleNums; /*!< number of audio samples in an audio frame */
} sdma_multi_fifo_config_t;

/*! @brief SDMA multi sai configurations.*/
typedef struct _sdma_multi_sai_config
{
    uint32_t saiIdx;	/*!< multi Sai idx/nblanes code value */
    uint8_t sampleNums; /*!< number of audio samples in an audio frame */
} sdma_multi_sai_config_t;

/*! @brief SDMA sw done configurations.*/
typedef struct _sdma_sw_done_config
{
    bool enableSwDone; /*!< fifo numbers */
    uint8_t swDoneSel; /*!< offset between multi fifo data register address */
} sdma_sw_done_config_t;

/*!
 * @brief SDMA transfer configuration
 *
 * This structure configures the source/destination transfer attribute.
 */
struct sdma_transfer_config
{
    uint32_t srcAddr;                      /*!< Source address of the transfer */
    uint32_t destAddr;                     /*!< Destination address of the transfer */
    sdma_transfer_size_t srcTransferSize;  /*!< Source data transfer size. */
    sdma_transfer_size_t destTransferSize; /*!< Destination data transfer size. */
    uint32_t bytesPerRequest;              /*!< Bytes to transfer in a minor loop*/
    uint32_t period_len;                   /*!< Buffer size for one descriptor */
    uint32_t transferSzie;                 /*!< Bytes to transfer for this descriptor */
    uint32_t scriptAddr;                   /*!< SDMA script address located in SDMA ROM. */
    uint32_t eventSource; /*!< Event source number for the channel. 0 means no event, use software trigger */
    bool isEventIgnore;   /*!< True means software trigger, false means hardware trigger */
    bool isSoftTriggerIgnore; /*!< If ignore the HE bit, 1 means use hardware events trigger, 0 means software trigger */
    sdma_transfer_type_t type;          /*!< Transfer type, transfer type used to decide the SDMA script. */
    sdma_multi_fifo_config_t multiFifo; /*!< multi fifo configurations */
    sdma_multi_sai_config_t multiSai;   /*!< multi sai configurations */
    sdma_sw_done_config_t swDone;       /*!< sw done selector */
};
typedef struct sdma_transfer_config sdma_transfer_config_t;

/*!
 * @brief SDMA buffer descriptor structure.
 *
 * This structure is a buffer descriptor, this structure describes the buffer start address and other options
 */
struct sdma_buffer_descriptor
{
    uint32_t count : 16;       /*!< Bytes of the buffer length for this buffer descriptor. */
    uint32_t status : 8;       /*!< E,R,I,C,W,D status bits stored here */
    uint32_t command : 8;      /*!< command mostly used for channel 0 */
    uint32_t bufferAddr;       /*!< Buffer start address for this descriptor. */
    uint32_t extendBufferAddr; /*!< External buffer start address, this is an optional for a transfer. */
} __attribute__ ((packed));
typedef struct sdma_buffer_descriptor sdma_buffer_descriptor_t;

/*!
 * @brief SDMA context structure for each channel. This structure can be load into SDMA core, with this structure, SDMA
 * scripts can start work.
 */
struct sdma_context_data
{
    /* Channel Status registers */
    uint32_t PC : 14;   /*!< Program counter */
    uint32_t unused1 : 1;
    uint32_t T : 1;     /*!< Test bit */
    uint32_t RPC : 14;  /*!< Return program counter */
    uint32_t unused0 : 1;
    uint32_t SF : 1;    /*!< Source fault while loading data */
    uint32_t SPC : 14;  /*!< Loop Start program counter */
    uint32_t unused2 : 1;
    uint32_t DF : 1;    /*!< Destination fault while storing data */
    uint32_t EPC : 14;  /*!< Loop end program counter */
    uint32_t LM : 2;    /*!< Loop mode */
    /* General purpose registers */
    uint32_t GeneralReg[8]; /*!< 8 general registers used for SDMA RISC core */
    /* Functional units state registers */
    uint32_t MDA;
    uint32_t MSA;
    uint32_t MS;
    uint32_t MD;
    uint32_t PDA;
    uint32_t PSA;
    uint32_t PS;
    uint32_t PD;
    uint32_t CA;
    uint32_t CS;
    uint32_t DDA;
    uint32_t DSA;
    uint32_t DS;
    uint32_t DD;
    /* Scratch RAM */
    uint32_t Scratch0;
    uint32_t Scratch1;
    uint32_t Scratch2;
    uint32_t Scratch3;
    uint32_t Scratch4;
    uint32_t Scratch5;
    uint32_t Scratch6;
    uint32_t Scratch7;
};
typedef struct sdma_context_data sdma_context_data_t;

/*! @brief SDMA channel priority.
 * enum dma_priority - DMA channel priority
 * Priority 0 is reserved by the SDMA hardware to determine when
 * there is no pending channel. Reset value is 0, which prevents the
 * channels from starting.
 */
enum dma_priority_t {
  kDMA_ChannelPriority0 = 0, /* Highest channel priority - priority 0. */
  kDMA_ChannelPriority1, /* Channel priority 1. */
  kDMA_ChannelPriority2, /* Channel priority 2. */
  kDMA_ChannelPriority3, /* Channel priority 3. */
  kDMA_ChannelPriority4, /* Channel priority 4. */
  kDMA_ChannelPriority5, /* Channel priority 5. */
  kDMA_ChannelPriority6, /* Channel priority 6. */
  kDMA_ChannelPriority7 /* Lowest channel priority - priority 7. */
};

/*!
 * @brief Dump SDMA Context.
 *
 * This function dumps a SDMA context.
 *
 * @param context context to dump.
 */
void SDMA_DumpContext(sdma_context_data_t context);

/* @} */

#if defined(__cplusplus)
}
#endif /* __cplusplus */



/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/* @} */

#endif /* _SDMA_H_ */

