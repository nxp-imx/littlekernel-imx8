/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2019-2020 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _FSL_SPDIF_H_
#define _FSL_SPDIF_H_

#include "fsl_common.h"
#include "spdif_reg_hw.h"
#include <dev/dma.h>

/*!
 * @addtogroup spdif
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
#define FSL_SPDIF_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0 */
/*@}*/

/*! @brief SPDIF return status*/
enum _spdif_status_t
{
    kStatus_SPDIF_RxDPLLLocked = MAKE_STATUS(kStatusGroup_SPDIF, 0),     /*!< SPDIF Rx PLL locked. */
    kStatus_SPDIF_TxFIFOError = MAKE_STATUS(kStatusGroup_SPDIF, 1),      /*!< SPDIF Tx FIFO error. */
    kStatus_SPDIF_TxFIFOResync = MAKE_STATUS(kStatusGroup_SPDIF, 2),     /*!< SPDIF Tx left and right FIFO resync. */
    kStatus_SPDIF_RxCnew = MAKE_STATUS(kStatusGroup_SPDIF, 3),           /*!< SPDIF Rx status channel value updated. */
    kStatus_SPDIF_ValidatyNoGood = MAKE_STATUS(kStatusGroup_SPDIF, 4),   /*!< SPDIF validaty flag not good. */
    kStatus_SPDIF_RxIllegalSymbol = MAKE_STATUS(kStatusGroup_SPDIF, 5),  /*!< SPDIF Rx receive illegal symbol. */
    kStatus_SPDIF_RxParityBitError = MAKE_STATUS(kStatusGroup_SPDIF, 6), /*!< SPDIF Rx parity bit error. */
    kStatus_SPDIF_UChannelOverrun = MAKE_STATUS(kStatusGroup_SPDIF, 7),  /*!< SPDIF receive U channel overrun. */
    kStatus_SPDIF_QChannelOverrun = MAKE_STATUS(kStatusGroup_SPDIF, 8),  /*!< SPDIF receive Q channel overrun. */
    kStatus_SPDIF_UQChannelSync = MAKE_STATUS(kStatusGroup_SPDIF, 9),    /*!< SPDIF U/Q channel sync found. */
    kStatus_SPDIF_UQChannelFrameError = MAKE_STATUS(kStatusGroup_SPDIF, 10), /*!< SPDIF U/Q channel frame error. */
    kStatus_SPDIF_RxFIFOError = MAKE_STATUS(kStatusGroup_SPDIF, 11),         /*!< SPDIF Rx FIFO error. */
    kStatus_SPDIF_RxFIFOResync = MAKE_STATUS(kStatusGroup_SPDIF, 12), /*!< SPDIF Rx left and right FIFO resync. */
    kStatus_SPDIF_LockLoss = MAKE_STATUS(kStatusGroup_SPDIF, 13),     /*!< SPDIF Rx PLL clock lock loss. */
    kStatus_SPDIF_TxIdle = MAKE_STATUS(kStatusGroup_SPDIF, 14),       /*!< SPDIF Tx is idle */
    kStatus_SPDIF_RxIdle = MAKE_STATUS(kStatusGroup_SPDIF, 15),       /*!< SPDIF Rx is idle */
    kStatus_SPDIF_QueueFull = MAKE_STATUS(kStatusGroup_SPDIF, 16)     /*!< SPDIF queue full */
};

/*! @brief SPDIF Rx FIFO full falg select, it decides when assert the rx full flag */
typedef enum _spdif_rxfull_select
{
    kSPDIF_RxFull1Sample = 0x0u, /*!< Rx full at least 1 sample in left and right FIFO */
    kSPDIF_RxFull4Samples,       /*!< Rx full at least 4 sample in left and right FIFO*/
    kSPDIF_RxFull8Samples,       /*!< Rx full at least 8 sample in left and right FIFO*/
    kSPDIF_RxFull16Samples,      /*!< Rx full at least 16 sample in left and right FIFO*/
} spdif_rxfull_select_t;

/*! @brief SPDIF tx FIFO EMPTY falg select, it decides when assert the tx empty flag */
typedef enum _spdif_txempty_select
{
    kSPDIF_TxEmpty0Sample = 0x0u, /*!< Tx empty at most 0 sample in left and right FIFO */
    kSPDIF_TxEmpty4Samples,       /*!< Tx empty at most 4 sample in left and right FIFO*/
    kSPDIF_TxEmpty8Samples,       /*!< Tx empty at most 8 sample in left and right FIFO*/
    kSPDIF_TxEmpty12Samples,      /*!< Tx empty at most 12 sample in left and right FIFO*/
} spdif_txempty_select_t;

/*! @brief SPDIF U channel source */
typedef enum _spdif_uchannel_source
{
    kSPDIF_NoUChannel = 0x0U,     /*!< No embedded U channel */
    kSPDIF_UChannelFromRx = 0x1U, /*!< U channel from receiver, it is CD mode */
    kSPDIF_UChannelFromTx = 0x3U, /*!< U channel from on chip tx */
} spdif_uchannel_source_t;

/*! @brief SPDIF clock gain*/
typedef enum _spdif_gain_select
{
    kSPDIF_GAIN_24 = 0x0U, /*!< Gain select is 24 */
    kSPDIF_GAIN_16,        /*!< Gain select is 16 */
    kSPDIF_GAIN_12,        /*!< Gain select is 12 */
    kSPDIF_GAIN_8,         /*!< Gain select is 8 */
    kSPDIF_GAIN_6,         /*!< Gain select is 6 */
    kSPDIF_GAIN_4,         /*!< Gain select is 4 */
    kSPDIF_GAIN_3,         /*!< Gain select is 3 */
} spdif_gain_select_t;

/*! @brief SPDIF tx data source */
typedef enum _spdif_tx_source
{
    kSPDIF_txFromReceiver = 0x1U, /*!< Tx data directly through SPDIF receiver */
    kSPDIF_txNormal = 0x5U,       /*!< Normal operation, data from processor */
} spdif_tx_source_t;

/*! @brief SPDIF tx data source */
typedef enum _spdif_validity_config
{
    kSPDIF_validityFlagAlwaysSet = 0x0U, /*!< Outgoing validity flags always set */
    kSPDIF_validityFlagAlwaysClear,      /*!< Outgoing validity flags always clear */
} spdif_validity_config_t;

/*! @brief The SPDIF interrupt enable flag */
enum _spdif_interrupt_enable_t
{
    kSPDIF_RxDPLLLocked = SPDIF_SIE_LOCK_MASK,                    /*!< SPDIF DPLL locked */
    kSPDIF_TxFIFOError = SPDIF_SIE_TXUNOV_MASK,                   /*!< Tx FIFO underrun or overrun */
    kSPDIF_TxFIFOResync = SPDIF_SIE_TXRESYN_MASK,                 /*!< Tx FIFO left and right channel resync */
    kSPDIF_RxControlChannelChange = SPDIF_SIE_CNEW_MASK,          /*!< SPDIF Rx control channel value changed */
    kSPDIF_ValidityFlagNoGood = SPDIF_SIE_VALNOGOOD_MASK,         /*!< SPDIF validity flag no good */
    kSPDIF_RxIllegalSymbol = SPDIF_SIE_SYMERR_MASK,               /*!< SPDIF receiver found illegal symbol */
    kSPDIF_RxParityBitError = SPDIF_SIE_BITERR_MASK,              /*!< SPDIF receiver found parity bit error */
    kSPDIF_UChannelReceiveRegisterFull = SPDIF_SIE_URXFUL_MASK,   /*!< SPDIF U channel revceive register full */
    kSPDIF_UChannelReceiveRegisterOverrun = SPDIF_SIE_URXOV_MASK, /*!< SPDIF U channel receive register overrun */
    kSPDIF_QChannelReceiveRegisterFull = SPDIF_SIE_QRXFUL_MASK,   /*!< SPDIF Q channel receive reigster full */
    kSPDIF_QChannelReceiveRegisterOverrun = SPDIF_SIE_QRXOV_MASK, /*!< SPDIF Q channel receive register overrun */
    kSPDIF_UQChannelSync = SPDIF_SIE_UQSYNC_MASK,                 /*!< SPDIF U/Q channel sync found */
    kSPDIF_UQChannelFrameError = SPDIF_SIE_UQERR_MASK,            /*!< SPDIF U/Q channel frame error */
    kSPDIF_RxFIFOError = SPDIF_SIE_RXFIFOUNOV_MASK,               /*!< SPDIF Rx FIFO underrun/overrun */
    kSPDIF_RxFIFOResync = SPDIF_SIE_RXFIFORESYN_MASK,             /*!< SPDIF Rx left and right FIFO resync */
    kSPDIF_LockLoss = SPDIF_SIE_LOCKLOSS_MASK,                    /*!< SPDIF receiver loss of lock */
    kSPDIF_TxFIFOEmpty = SPDIF_SIE_TXEM_MASK,                     /*!< SPDIF Tx FIFO empty */
    kSPDIF_RxFIFOFull = SPDIF_SIE_RXFIFOFUL_MASK                  /*!< SPDIF Rx FIFO full */
};

/*! @brief The DMA request sources */
enum _spdif_dma_enable_t
{
    kSPDIF_RxDMAEnable = SPDIF_SCR_DMA_RX_EN_MASK, /*!< Rx FIFO full */
    kSPDIF_TxDMAEnable = SPDIF_SCR_DMA_TX_EN_MASK, /*!< Tx FIFO empty */
};

/*! @brief SPDIF DMA mode */
typedef enum _dma_mode {
    kSPDIF_DMA_Private_Buffer = 0x0U, /*!< DMA fills an intermediate buffer further flushed by CPU (default) */
    kSPDIF_DMA_Zerocopy_Uncached,     /*!< Zero-copy into RX uncached circular buffer */
    kSPDIF_DMA_Zerocopy_Cached,       /*!< Zero-copy into RX cached circular buffer (cache management) */
} dma_mode_t;

/*! @brief SPDIF user configuration structure */
typedef struct _spdif_config
{
    bool isTxAutoSync;                      /*!< If auto sync mechanism open */
    bool isRxAutoSync;                      /*!< If auto sync mechanism open */
    bool isRxRawMode;                       /*!< Raw Capture Mode */
    uint8_t DPLLClkSource;                  /*!< SPDIF DPLL clock source, range from 0~15, meaning is chip-specific */
    uint8_t txClkSource;                    /*!< SPDIF tx clock source, range from 0~7, meaning is chip-specific */
    spdif_rxfull_select_t rxFullSelect;     /*!< SPDIF rx buffer full select */
    spdif_txempty_select_t txFullSelect;    /*!< SPDIF tx buffer empty select */
    spdif_uchannel_source_t uChannelSrc;    /*!< U channel source */
    spdif_tx_source_t txSource;             /*!< SPDIF tx data source */
    spdif_validity_config_t validityConfig; /*!< Validity flag config */
    spdif_gain_select_t gain;               /*!< Rx receive clock measure gain parameter. */
} spdif_config_t;

/*!@brief SPDIF transfer queue size, user can refine it according to use case. */
#define SPDIF_XFER_QUEUE_SIZE (4)

typedef struct _spdif_handle spdif_handle_t;

/*! @brief SPDIF handle structure */
struct _spdif_handle
{
    uint32_t irq_mask;                                  /*!< Interrupt mask */
    void *buf;                                          /*!< ISR buffer */
    size_t buf_size;                                    /*!< ISR buffer size */
    ssize_t xfer_remaining;                             /*!< Remaining bytes of current transfer */
#define RX_CBUF_N_FIFO_WATERMARK 8
    size_t cbuf_watermark;                              /*!< Circular buffer watermark  */

    uint8_t watermark;                                  /*!< Watermark value */
    struct dma_chan *dma_chan;                          /*!< client DMA channel */
};

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /*_cplusplus*/

/*!
 * @name Initialization and deinitialization
 * @{
 */

/*!
 * @brief Initializes the SPDIF peripheral.
 *
 * Ungates the SPDIF clock, resets the module, and configures SPDIF with a configuration structure.
 * The configuration structure can be custom filled or set with default values by
 * SPDIF_GetDefaultConfig().
 *
 * @note  This API should be called at the beginning of the application to use
 * the SPDIF driver. Otherwise, accessing the SPDIF module can cause a hard fault
 * because the clock is not enabled.
 *
 * @param base SPDIF base pointer
 * @param config SPDIF configuration structure.
*/
void SPDIF_Init(SPDIF_Type *base, const spdif_config_t *config);

/*!
 * @brief  Sets the SPDIF configuration structure to default values.
 *
 * This API initializes the configuration structure for use in SPDIF_Init.
 * The initialized structure can remain unchanged in SPDIF_Init, or it can be modified
 *  before calling SPDIF_Init.
 * This is an example.
   @code
   spdif_config_t config;
   SPDIF_GetDefaultConfig(&config);
   @endcode
 *
 * @param config pointer to master configuration structure
 */
void SPDIF_GetDefaultConfig(spdif_config_t *config);

/*!
 * @brief De-initializes the SPDIF peripheral.
 *
 * This API gates the SPDIF clock. The SPDIF module can't operate unless SPDIF_Init is called to enable the clock.
 *
 * @param base SPDIF base pointer
*/
void SPDIF_Deinit(SPDIF_Type *base);

/*!
 * @brief Resets the SPDIF Tx.
 *
 * This function makes Tx FIFO in reset mode.
 *
 * @param base SPDIF base pointer
 */
static inline void SPDIF_TxFIFOReset(SPDIF_Type *base)
{
    /* no TX FIFO reset bit available */
}

/*!
 * @brief Resets the SPDIF Rx.
 *
 * This function enables the software reset and FIFO reset of SPDIF Rx. After reset, clear the reset bit.
 *
 * @param base SPDIF base pointer
 */
static inline void SPDIF_RxFIFOReset(SPDIF_Type *base)
{
    base->SCR |= SPDIF_SCR_RXFIFO_RST_MASK;
}

/*!
 * @brief Enables/disables the SPDIF Tx.
 *
 * @param base SPDIF base pointer
 * @param enable True means enable SPDIF Tx, false means disable.
 */
void SPDIF_TxEnable(SPDIF_Type *base, bool enable);

/*!
 * @brief Enables/disables the SPDIF Rx.
 *
 * @param base SPDIF base pointer
 * @param enable True means enable SPDIF Rx, false means disable.
 */
static inline void SPDIF_RxEnable(SPDIF_Type *base, bool enable)
{
    if (enable)
    {
        /* Open Rx FIFO */
        base->SCR &= ~(SPDIF_SCR_RXFIFO_CTRL_MASK |
                       SPDIF_SCR_RXFIFO_OFF_ON_MASK |
                       SPDIF_SCR_RXFIFO_RST_MASK);
    }
    else
    {
        base->SCR |= SPDIF_SCR_RXFIFO_OFF_ON_MASK;
    }
}

/*! @} */

/*!
 * @name Status
 * @{
 */

/*!
 * @brief Gets the SPDIF status flag state.
 *
 * @param base SPDIF base pointer
 * @return SPDIF status flag value. Use the _spdif_interrupt_enable_t to get the status value needed.
 */
static inline uint32_t SPDIF_GetStatusFlag(SPDIF_Type *base)
{
    return base->SIS;
}

/*!
 * @brief Clears the SPDIF status flag state.
 *
 * @param base SPDIF base pointer
 * @param mask State mask. It can be a combination of the _spdif_interrupt_enable_t member. Notice these members
 *             cannot be included, as these flags cannot be cleared by writing 1 to these bits:
 *        @arg kSPDIF_UChannelReceiveRegisterFull
 *        @arg kSPDIF_QChannelReceiveRegisterFull
 *        @arg kSPDIF_TxFIFOEmpty
 *        @arg kSPDIF_RxFIFOFull
 */
static inline void SPDIF_ClearStatusFlags(SPDIF_Type *base, uint32_t mask)
{
    base->SIC = mask;
}

/*! @} */

/*!
 * @name Interrupts
 * @{
 */

/*!
 * @brief Enables the SPDIF Tx interrupt requests.
 *
 * @param base SPDIF base pointer
 * @param mask interrupt source
 *     The parameter can be a combination of the following sources if defined.
 *     @arg kSPDIF_WordStartInterruptEnable
 *     @arg kSPDIF_SyncErrorInterruptEnable
 *     @arg kSPDIF_FIFOWarningInterruptEnable
 *     @arg kSPDIF_FIFORequestInterruptEnable
 *     @arg kSPDIF_FIFOErrorInterruptEnable
 */
static inline void SPDIF_EnableInterrupts(SPDIF_Type *base, uint32_t mask)
{
    base->SIE |= mask;
}

/*!
 * @brief Disables the SPDIF Tx interrupt requests.
 *
 * @param base SPDIF base pointer
 * @param mask interrupt source
 *     The parameter can be a combination of the following sources if defined.
 *     @arg kSPDIF_WordStartInterruptEnable
 *     @arg kSPDIF_SyncErrorInterruptEnable
 *     @arg kSPDIF_FIFOWarningInterruptEnable
 *     @arg kSPDIF_FIFORequestInterruptEnable
 *     @arg kSPDIF_FIFOErrorInterruptEnable
 */
static inline void SPDIF_DisableInterrupts(SPDIF_Type *base, uint32_t mask)
{
    base->SIE &= ~mask;
}

/*! @} */

/*!
 * @name DMA Control
 * @{
 */

/*!
 * @brief Enables/disables the SPDIF DMA requests.
 * @param base SPDIF base pointer
 * @param mask SPDIF DMA enable mask, The parameter can be a combination of the following sources if defined
 *      @arg kSPDIF_RxDMAEnable
 *      @arg kSPDIF_TxDMAEnable
 * @param enable True means enable DMA, false means disable DMA.
 */
static inline void SPDIF_EnableDMA(SPDIF_Type *base, uint32_t mask, bool enable)
{
    if (enable)
    {
        base->SCR |= mask;
    }
    else
    {
        base->SCR &= ~mask;
    }
}

/*!
 * @brief  Gets the SPDIF Tx left data register address.
 *
 * This API is used to provide a transfer address for the SPDIF DMA transfer configuration.
 *
 * @param base SPDIF base pointer.
 * @return data register address.
 */
static inline uintptr_t SPDIF_TxGetLeftDataRegisterAddress(SPDIF_Type *base)
{
    return (uintptr_t)(&(base->STL));
}

/*!
 * @brief  Gets the SPDIF Tx right data register address.
 *
 * This API is used to provide a transfer address for the SPDIF DMA transfer configuration.
 *
 * @param base SPDIF base pointer.
 * @return data register address.
 */
static inline uintptr_t SPDIF_TxGetRightDataRegisterAddress(SPDIF_Type *base)
{
    return (uintptr_t)(&(base->STR));
}

/*!
 * @brief  Gets the SPDIF Rx left data register address.
 *
 * This API is used to provide a transfer address for the SPDIF DMA transfer configuration.
 *
 * @param base SPDIF base pointer.
 * @return data register address.
 */
static inline uintptr_t SPDIF_RxGetLeftDataRegisterAddress(SPDIF_Type *base)
{
    return (uintptr_t)(&(base->SRL));
}

/*!
 * @brief  Gets the SPDIF Rx right data register address.
 *
 * This API is used to provide a transfer address for the SPDIF DMA transfer configuration.
 *
 * @param base SPDIF base pointer.
 * @return data register address.
 */
static inline uintptr_t SPDIF_RxGetRightDataRegisterAddress(SPDIF_Type *base)
{
    return (uintptr_t)(&(base->SRR));
}

/*! @} */

/*!
 * @name Bus Operations
 * @{
 */

/*!
 * @brief Configures the SPDIF Tx sample rate.
 *
 * The audio format can be changed at run-time. This function configures the sample rate.
 *
 * @param base SPDIF base pointer.
 * @param sampleRate_Hz SPDIF sample rate frequency in Hz.
 * @param sourceClockFreq_Hz SPDIF tx clock source frequency in Hz.
*/
void SPDIF_TxSetSampleRate(SPDIF_Type *base, uint32_t sampleRate_Hz, uint32_t sourceClockFreq_Hz);

/*!
 * @brief Configures the SPDIF Rx audio format.
 *
 * The audio format can be changed at run-time. This function configures the sample rate and audio data
 * format to be transferred.
 *
 * @param base SPDIF base pointer.
 * @param clockSourceFreq_Hz SPDIF system clock frequency in hz.
 */
uint32_t SPDIF_GetRxSampleRate(SPDIF_Type *base, uint32_t clockSourceFreq_Hz);

/*!
 * @brief Sends data using a blocking method.
 *
 * @note This function blocks by polling until data is ready to be sent.
 *
 * @param base SPDIF base pointer.
 * @param buffer Pointer to the data to be written.
 * @param size Bytes to be written.
 */
void SPDIF_WriteBlocking(SPDIF_Type *base, uint8_t *buffer, uint32_t size);

/*!
 * @brief Dumps SPDIF register.
 *
 * @param base SPDIF base pointer.
 */
void SPDIF_Dump(SPDIF_Type *base);

/*!
 * @brief Writes data into SPDIF FIFO.
 *
 * @param base SPDIF base pointer.
 * @param data Data needs to be written.
 */
static inline void SPDIF_WriteLeftData(SPDIF_Type *base, uint32_t data)
{
    base->STL = data;
}

/*!
 * @brief Writes data into SPDIF FIFO.
 *
 * @param base SPDIF base pointer.
 * @param data Data needs to be written.
 */
static inline void SPDIF_WriteRightData(SPDIF_Type *base, uint32_t data)
{
    base->STR = data;
}

/*!
 * @brief Writes data into SPDIF FIFO.
 *
 * @param base SPDIF base pointer.
 * @param data Data needs to be written.
 */
static inline void SPDIF_WriteChannelStatusHigh(SPDIF_Type *base, uint32_t data)
{
    base->STCSCH = data;
}

/*!
 * @brief Writes data into SPDIF FIFO.
 *
 * @param base SPDIF base pointer.
 * @param data Data needs to be written.
 */
static inline void SPDIF_WriteChannelStatusLow(SPDIF_Type *base, uint32_t data)
{
    base->STCSCL = data;
}

/*!
 * @brief Receives data using a blocking method.
 *
 * @note This function blocks by polling until data is ready to be sent.
 *
 * @param base SPDIF base pointer.
 * @param buffer Pointer to the data to be read.
 * @param size Bytes to be read.
 */
void SPDIF_ReadBlocking(SPDIF_Type *base, uint8_t *buffer, uint32_t size);

/*!
 * @brief Reads data from the SPDIF FIFO.
 *
 * @param base SPDIF base pointer.
 * @return Data in SPDIF FIFO.
 */
static inline uint32_t SPDIF_ReadLeftData(SPDIF_Type *base)
{
    return base->SRL;
}

/*!
 * @brief Reads data from the SPDIF FIFO.
 *.

 * @param base SPDIF base pointer.
 * @return Data in SPDIF FIFO.
 */
static inline uint32_t SPDIF_ReadRightData(SPDIF_Type *base)
{
    return base->SRR;
}

/*!
 * @brief Reads data from the SPDIF FIFO.
 *
 * @param base SPDIF base pointer.
 * @return Data in SPDIF FIFO.
 */
static inline uint32_t SPDIF_ReadChannelStatusHigh(SPDIF_Type *base)
{
    return base->SRCSH;
}

/*!
 * @brief Reads data from the SPDIF FIFO.
 *.

 * @param base SPDIF base pointer.
 * @return Data in SPDIF FIFO.
 */
static inline uint32_t SPDIF_ReadChannelStatusLow(SPDIF_Type *base)
{
    return base->SRCSL;
}

/*!
 * @brief Reads data from the SPDIF FIFO.
 *
 * @param base SPDIF base pointer.
 * @return Data in SPDIF FIFO.
 */
static inline uint32_t SPDIF_ReadQChannel(SPDIF_Type *base)
{
    return base->SRQ;
}

/*!
 * @brief Reads data from the SPDIF FIFO.
 *.

 * @param base SPDIF base pointer.
 * @return Data in SPDIF FIFO.
 */
static inline uint32_t SPDIF_ReadUChannel(SPDIF_Type *base)
{
    return base->SRU;
}
/*! @} */

#if defined(__cplusplus)
}
#endif /*_cplusplus*/

/*! @} */

#endif /* _FSL_SPDIF_H_ */
