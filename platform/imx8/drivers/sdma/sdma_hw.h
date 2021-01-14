/*
 * Copyright 2016, 2019-2020 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _SDMA_HW_H_
#define _SDMA_HW_H_

#include "fsl_common.h"
#include "sdma.h"
#include "sdma_reg_hw.h"


/*!
 * @addtogroup sdma
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SDMA_COMMAND_GET_CONTEXT(ch)        (kSDMA_BDCommandGETCTX|(ch<<3))
#define IMX_DMA_SG_LOOP  1
#define SDMA_BD_MAX_CNT 0xfffc /* align with 4 bytes */

#define SDMA_FIRMWARE_MAGIC 0x414d4453
#define SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V1 34
#define SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V2 38
#define SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V3 45
#define SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V4 46
#define SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V47 50
#define SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V48 52

/*! @name Driver version */
/*@{*/
/*! @brief SDMA driver version */
#define FSL_SDMA_DRIVER_VERSION (MAKE_VERSION(2, 1, 0)) /*!< Version 2.1.0. */
/*@}*/

#ifndef _MIMX8MM6_cm4_FEATURES_H_
/* @brief SDMA availability on the SoC. */
#define FSL_FEATURE_SOC_SDMA_COUNT (3)

/* SDMA module features */

/* @brief SDMA module channel number. */
#define FSL_FEATURE_SDMA_MODULE_CHANNEL (32)
/* @brief SDMA module event number. */
#define FSL_FEATURE_SDMA_EVENT_NUM (48)

/* @brief SDMA ROM memory to memory script start address. */
#define FSL_FEATURE_SDMA_M2M_ADDR (644)
/* @brief SDMA ROM peripheral to memory script start address. */
#define FSL_FEATURE_SDMA_P2M_ADDR (685)
/* @brief SDMA ROM memory to peripheral script start address. */
#define FSL_FEATURE_SDMA_M2P_ADDR (749)
/* @brief SDMA ROM uart to memory script start address. */
#define FSL_FEATURE_SDMA_UART2M_ADDR (819)
/* @brief SDMA ROM peripheral on SPBA to memory script start address. */
#define FSL_FEATURE_SDMA_SHP2M_ADDR (893)
/* @brief SDMA ROM memory to peripheral on SPBA script start address. */
#define FSL_FEATURE_SDMA_M2SHP_ADDR (962)
/* @brief SDMA ROM UART on SPBA to memory script start address. */
#define FSL_FEATURE_SDMA_UARTSH2M_ADDR (1034)
/* @brief SDMA ROM SPDIF to memory script start address. */
#define FSL_FEATURE_SDMA_SPDIF2M_ADDR (1102)
/* @brief SDMA ROM memory to SPDIF script start address. */
#define FSL_FEATURE_SDMA_M2SPDIF_ADDR (1136)

#endif /* _MIMX8MM6_cm4_FEATURES_H_ */

/*! @brief SDMA buffer descriptor status */
typedef enum _sdma_bd_status
{
    kSDMA_BDStatusDone = 0x1U,       /*!< BD ownership, 0 means ARM core owns the BD, while 1 means SDMA owns BD. */
    kSDMA_BDStatusWrap = 0x2U,       /*!< While this BD is last one, the next BD will be the first one */
    kSDMA_BDStatusContinuous = 0x4U, /*!< Buffer is allowed to transfer/receive to/from multiple buffers */
    kSDMA_BDStatusInterrupt = 0x8U,  /*!< While this BD finished, send an interrupt. */
    kSDMA_BDStatusError = 0x10U,     /*!< Error occurred on buffer descriptor command. */
    kSDMA_BDStatusLast =
        0x20U, /*!< This BD is the last BD in this array. It means the transfer ended after this buffer */
    kSDMA_BDStatusExtend = 0x80, /*!< Buffer descriptor extend status for SDMA scripts */
} sdma_bd_status_t;

/*! @brief SDMA buffer descriptor command */
typedef enum _sdma_bd_command
{
    kSDMA_BDCommandSETDM = 0x1U,  /*!< Load SDMA data memory from ARM core memory buffer. */
    kSDMA_BDCommandGETDM = 0x2U,  /*!< Copy SDMA data memory to ARM core memory buffer. */
    kSDMA_BDCommandSETPM = 0x4U,  /*!< Load SDMA program memory from ARM core memory buffer. */
    kSDMA_BDCommandGETPM = 0x6U,  /*!< Copy SDMA program memory to ARM core memory buffer. */
    kSDMA_BDCommandSETCTX = 0x7U, /*!< Load context for one channel into SDMA RAM from ARM platform memory buffer. */
    kSDMA_BDCommandGETCTX = 0x3U, /*!< Copy context for one channel from SDMA RAM to ARM platform memory buffer. */
} sdma_bd_command_t;

/*! @brief SDMA context switch mode */
typedef enum _sdma_context_switch_mode
{
    kSDMA_ContextSwitchModeStatic = 0x0U,     /*!< SDMA context switch mode static */
    kSDMA_ContextSwitchModeDynamicLowPower,   /*!< SDMA context switch mode dynamic with low power */
    kSDMA_ContextSwitchModeDynamicWithNoLoop, /*!< SDMA context switch mode dynamic with no loop */
    kSDMA_ContextSwitchModeDynamic,           /*!< SDMA context switch mode dynamic */
} sdma_context_switch_mode_t;

/*! @brief SDMA core clock frequency ratio to the ARM DMA interface. */
typedef enum _sdma_clock_ratio
{
    kSDMA_HalfARMClockFreq = 0x0U, /*!< SDMA core clock frequency half of ARM platform */
    kSDMA_ARMClockFreq,            /*!< SDMA core clock frequency equals to ARM platform */
} sdma_clock_ratio_t;



/*! @brief SDMA transfer status */
enum _sdma_transfer_status
{
    kStatus_SDMA_ERROR = MAKE_STATUS(kStatusGroup_SDMA, 0), /*!< SDMA context error. */
    kStatus_SDMA_Busy = MAKE_STATUS(kStatusGroup_SDMA, 1),  /*!< Channel is busy and can't handle the
                                                                 transfer request. */
};

/*! @brief SDMA multi fifo mask */
enum _sdma_multi_fifo_mask
{
    kSDMA_MultiFifoWatermarkLevelMask = 0xFFFU,/*!< multi fifo watermark level mask */
    kSDMA_MultiFifoNumsMask = 0xFU,            /*!< multi fifo nums mask */
    kSDMA_MultiFifoOffsetMask = 0xFU,          /*!< multi fifo offset mask */
    kSDMA_MultiFifoSwDoneMask = 0x1U,          /*!< multi fifo sw done mask */
    kSDMA_MultiFifoSwDoneSelectorMask = 0xFU,  /*!< multi fifo sw done selector mask */
    kSDMA_MultiFifoNumsSampleMask = 0xFU,  /*!< multi fifo nums audio words per audio frame */
};

/*! @brief SDMA multi fifo shift */
enum _sdma_multi_fifo_shift
{
    kSDMA_MultiFifoWatermarkLevelShift = 0U,  /*!< multi fifo watermark level shift */
    kSDMA_MultiFifoNumsShift = 12U,           /*!< multi fifo nums shift */
    kSDMA_MultiFifoOffsetShift = 16U,         /*!< multi fifo offset shift */
    kSDMA_MultiFifoSwDoneShift = 23U,         /*!< multi fifo sw done shift */
    kSDMA_MultiFifoSwDoneSelectorShift = 24U, /*!< multi fifo sw done selector shift */
    kSDMA_MultiFifoNumsSampleShift = 28U,     /*!< multi fifo nums audio words per audio frame */
};

/*! @brief SDMA multi SAI mask */
enum _sdma_multi_sai_mask
{
    kSDMA_MultiSaiWatermarkLevelMask = 0xFFFU,/*!< multi fifo watermark level mask */
    kSDMA_MultiSaiNumsSampleMask = 0xFU,   /*!< multi SAI nums audio words per audio frame */
};

/*! @brief SDMA multi SAI shift */
enum _sdma_multi_sai_shift
{
    kSDMA_MultiSaiWatermarkLevelShift = 0U,  /*!< multi SAI watermark level shift */
    kSDMA_MultiSaiNumsSampleShift = 28U,     /*!< multi SAI nums audio words per audio frame */
};

/*! @brief SDMA global configuration structure.*/
struct sdma_config
{
    bool enableRealTimeDebugPin;   /*!< If enable real-time debug pin, default is closed to reduce power consumption.*/
    bool isSoftwareResetClearLock; /*!< If software reset clears the LOCK bit which prevent writing SDMA scripts into
                                      SDMA.*/
    sdma_clock_ratio_t ratio;      /*!< SDMA core clock ratio to ARM platform DMA interface */
};
typedef struct sdma_config sdma_config_t;



/*!
 * @brief SDMA channel control descriptor structure.
 */
struct sdma_channel_control
{
    uint32_t currentBDAddr; /*!< Address of current buffer descriptor processed  */
    uint32_t baseBDAddr;    /*!< The start address of the buffer descriptor array */
    uint32_t channelDesc;   /*!< Optional for transfer */
    uint32_t status;        /*!< Channel status */
} __attribute__ ((packed));
typedef struct sdma_channel_control sdma_channel_control_t;

/*! @brief Callback for SDMA */
struct sdma_handle;

/*! @brief Define callback function for SDMA. */
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
typedef void (*sdma_callback)(struct sdma_handle *handle, void *userData, bool transferDone, uint32_t bdIndex);
#else
typedef void (*sdma_callback)(void *userData);
#endif /* FSL_SDK_DISABLE_IRQ */

/*! @brief Callback Data for SDMA */
struct sdma_callback_data {
    struct sdma_handle *handle;
    void *userData;
    bool transferDone;
    uint32_t bdIndex;
};
typedef struct sdma_callback_data sdma_callback_data_t;

/*! @brief SDMA transfer handle structure */
/* FIXME: sdma_handle to be renamed if only use for one channel? like sdma_channel, ... */
struct sdma_handle
{
    sdma_callback callback;           /*!< Callback function for major count exhausted. */
    void *userData;                   /*!< Callback function parameter. */
    SDMAARM_Type *base;               /*!< SDMA peripheral base address. */
    sdma_buffer_descriptor_t *BDPool; /*!< Pointer to memory stored BD arrays. */
    uint32_t bdCount;                 /*!< How many buffer descriptor   */
    uint32_t bdIndex;                 /*!< How many buffer descriptor   */
    uint32_t eventSource;             /*!< Event source count for the channel */
    sdma_context_data_t *context;     /*!< Channel context to exectute in SDMA */
    paddr_t context_phys;             /*!< Channel context physical address */
    uint8_t channel;                  /*!< SDMA channel number. */
    uint8_t channelNums;              /*!< total transfer channel numbers, used for multififo */
    uint8_t fifoOffset;               /*!< fifo address offset between FIFOs */
    uint8_t count;                    /*!< The transfer data count in a DMA request */
    uint8_t priority;                 /*!< SDMA channel priority */
    uint8_t flags;                    /*!< Flags for the current channel. */
    uint8_t status;                   /*!< The status of the current channel. */
    /* base address of the channel 0 buffer descriptor */
    sdma_buffer_descriptor_t *bd0;    /*!< SDMA channel 0 buffer descriptor */
    paddr_t bd0_phys;                 /*!< SDMA channel 0 buffer descriptor physical address */
    /* base address of this channel's buffer descriptor */
    sdma_buffer_descriptor_t *bd;     /*!< SDMA channel buffer descriptor */
    paddr_t bd_phys;                  /*!< SDMA channel buffer descriptor physical address */
    uint32_t chn_real_count;          /*!< transferred bytes */
    unsigned int period_len;          /*!< buffer descriptor size in cyclic mode */
    struct sdma_transfer_config cfg; /*!< DMA transfer configuration */
};
typedef struct sdma_handle sdma_handle_t;

/**
 * struct sdma_fw_script_start_addrs - SDMA firmware script start pointers
 *
 * start addresses of the different functions in the physical
 * address space of the SDMA.
 */
struct sdma_fw_script_start_addrs {
	int32_t ap_2_ap_addr;
	int32_t ap_2_bp_addr;
	int32_t ap_2_ap_fixed_addr;
	int32_t bp_2_ap_addr;
	int32_t loopback_on_dsp_side_addr;
	int32_t mcu_interrupt_only_addr;
	int32_t firi_2_per_addr;
	int32_t firi_2_mcu_addr;
	int32_t per_2_firi_addr;
	int32_t mcu_2_firi_addr;
	int32_t uart_2_per_addr;
	int32_t uart_2_mcu_fixed_addr;
	int32_t per_2_app_addr;
	int32_t mcu_2_app_addr;
	int32_t per_2_per_addr;
	int32_t uartsh_2_per_addr;
	int32_t uartsh_2_mcu_fixed_addr;
	int32_t per_2_shp_addr;
	int32_t mcu_2_shp_addr;
	int32_t ata_2_mcu_addr;
	int32_t mcu_2_ata_addr;
	int32_t app_2_per_addr;
	int32_t app_2_mcu_addr;
	int32_t shp_2_per_addr;
	int32_t shp_2_mcu_addr;
	int32_t mshc_2_mcu_addr;
	int32_t mcu_2_mshc_addr;
	int32_t spdif_2_mcu_addr;
	int32_t mcu_2_spdif_addr;
	int32_t asrc_2_mcu_addr;
	int32_t ext_mem_2_ipu_addr;
	int32_t descrambler_addr;
	int32_t dptc_dvfs_addr;
	int32_t utra_addr;
	int32_t ram_code_start_addr;
	/* End of v1 array */
	int32_t mcu_2_ssish_addr;
	int32_t ssish_2_mcu_addr;
	int32_t hdmi_dma_addr;
	/* End of v2 array */
	int32_t zcanfd_2_mcu_addr;
	int32_t zqspi_2_mcu_addr;
	int32_t mcu_2_ecspi_addr;
	int32_t mcu_2_sai_addr;
	int32_t sai_2_mcu_addr;
	/* End of v3 array */
	int32_t uart_2_mcu_addr;
	int32_t uartsh_2_mcu_addr;
	int32_t mcu_2_zqspi_addr;
	/* End of v4 array */
	int32_t zz4l_2_mcu;
	int32_t zz8l_2_sai;
	int32_t zzmic2_2_mcu;
	int32_t zzmic4_2_mcu;
	/* End of v4.7 array */
	int32_t mcu_2_multi_sai;
	int32_t multi_sai_2_mcu;
	/* End of v4.8 array */
};

/**
 * struct sdma_firmware_header - Layout of the firmware image
 *
 * @magic		"SDMA"
 * @version_major	increased whenever layout of struct sdma_script_start_addrs
 *			changes.
 * @version_minor	firmware minor version (for binary compatible changes)
 * @script_addrs_start	offset of struct sdma_script_start_addrs in this image
 * @num_script_addrs	Number of script addresses in this image
 * @ram_code_start	offset of SDMA ram image in this firmware image
 * @ram_code_size	size of SDMA ram image
 * @script_addrs	Stores the start address of the SDMA scripts
 *			(in SDMA memory space)
 */
struct sdma_firmware_header {
    uint32_t magic;
    uint32_t version_major;
    uint32_t version_minor;
    uint32_t script_addrs_start;
    uint32_t num_script_addrs;
    uint32_t ram_code_start;
    uint32_t ram_code_size;
};

/*******************************************************************************
 * APIs
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*!
 * @name SDMA initialization and de-initialization
 * @{
 */

/*!
 * @brief Initializes the SDMA peripheral.
 *
 * This function ungates the SDMA clock and configures the SDMA peripheral according
 * to the configuration structure.
 *
 * @param base SDMA peripheral base address.
 * @param config A pointer to the configuration structure, see "sdma_config_t".
 * @note This function enables the minor loop map feature.
 */
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
void SDMA_Init(SDMAARM_Type *base, const sdma_config_t *config);
#else
void SDMA_Init(SDMAARM_Type *base, const sdma_config_t *config, sdma_channel_control_t *sdma_ccb, paddr_t sdma_ccb0_phys, sdma_buffer_descriptor_t *sdma_bd, paddr_t sdma_bd0_phys);
#endif

/*!
 * @brief Deinitializes the SDMA peripheral.
 *
 * This function gates the SDMA clock.
 *
 * @param base SDMA peripheral base address.
 */
void SDMA_Deinit(SDMAARM_Type *base);

/*!
 * @brief Gets the SDMA default configuration structure.
 *
 * This function sets the configuration structure to default values.
 * The default configuration is set to the following values.
 * @code
 *   config.enableRealTimeDebugPin = false;
 *   config.isSoftwareResetClearLock = true;
 *   config.ratio = kSDMA_HalfARMClockFreq;
 * @endcode
 *
 * @param config A pointer to the SDMA configuration structure.
 */
void SDMA_GetDefaultConfig(sdma_config_t *config);

/*!
 * @brief Sets all SDMA core register to reset status.
 *
 * If only reset ARM core, SDMA register cannot return to reset value, shall call this function to reset all SDMA
 * register to reset vaule. But the internal status cannot be reset.
 *
 * @param base SDMA peripheral base address.
 */
void SDMA_ResetModule(SDMAARM_Type *base);

/* @} */
/*!
 * @name SDMA Channel Operation
 * @{
 */
/*!
 * @brief Enables the interrupt source for the SDMA error.
 *
 * Enable this will trigger an interrupt while SDMA occurs error while executing scripts.
 *
 * @param base SDMA peripheral base address.
 * @param channel SDMA channel number.
 */
static inline void SDMA_EnableChannelErrorInterrupts(SDMAARM_Type *base, uint32_t channel)
{
    base->INTRMASK |= (1U << channel);
}

/*!
 * @brief Disables the interrupt source for the SDMA error.
 *
 * @param base SDMA peripheral base address.
 * @param channel SDMA channel number.
 */
static inline void SDMA_DisableChannelErrorInterrupts(SDMAARM_Type *base, uint32_t channel)
{
    base->INTRMASK &= ~(1U << channel);
}

/* @} */
/*!
 * @name SDMA Buffer Descriptor Operation
 * @{
 */

/*!
 * @brief Sets buffer descriptor contents.
 *
 * This function sets the descriptor contents such as source, dest address and status bits.
 *
 * @param bd Pointer to the buffer descriptor structure.
 * @param srcAddr Source address for the buffer descriptor.
 * @param destAddr Destination address for the buffer descriptor.
 * @param busWidth The transfer width, it only can be a member of sdma_transfer_size_t.
 * @param bufferSize Buffer size for this descriptor, this number shall less than 0xFFFF. If need to transfer a big
 * size,
 * shall divide into several buffer descriptors.
 * @param isLast Is the buffer descriptor the last one for the channel to transfer. If only one descriptor used for
 * the channel, this bit shall set to TRUE.
 * @param enableInterrupt If trigger an interrupt while this buffer descriptor transfer finished.
 * @param isWrap Is the buffer descriptor need to be wrapped. While this bit set to true, it will automatically wrap
 * to the first buffer descrtiptor to do transfer.
 * @param type Transfer type, memory to memory, peripheral to memory or memory to peripheral.
 */
void SDMA_ConfigBufferDescriptor(sdma_buffer_descriptor_t *bd,
                                 uint32_t srcAddr,
                                 uint32_t destAddr,
                                 sdma_transfer_size_t busWidth,
                                 size_t bufferSize,
                                 bool isLast,
                                 bool enableInterrupt,
                                 bool isWrap,
                                 sdma_transfer_type_t type);

/*! @} */
/*!
 * @name SDMA Channel Transfer Operation
 * @{
 */

/*!
 * @brief Set SDMA channel priority.
 *
 * This function sets the channel priority. The default value is 0 for all channels, priority 0 will prevents
 * channel from starting, so the priority must be set before start a channel.
 *
 * @param base SDMA peripheral base address.
 * @param channel SDMA channel number.
 * @param priority SDMA channel priority.
 */
static inline void SDMA_SetChannelPriority(SDMAARM_Type *base, uint32_t channel, uint8_t priority)
{
    base->SDMA_CHNPRI[channel] = priority;
}

/*!
 * @brief Set SDMA request source mapping channel.
 *
 * This function sets which channel will be triggered by the dma request source.
 *
 * @param base SDMA peripheral base address.
 * @param source SDMA dma request source number.
 * @param channelMask SDMA channel mask. 1 means channel 0, 2 means channel 1, 4 means channel 3. SDMA supports
 * an event trigger multi-channel. A channel can also be triggered by several source events.
 */
static inline void SDMA_SetSourceChannel(SDMAARM_Type *base, uint32_t source, uint32_t channelMask)
{
    base->CHNENBL[source] = channelMask;
}

/*!
 * @brief Start a SDMA channel by software trigger.
 *
 * This function start a channel.
 *
 * @param base SDMA peripheral base address.
 * @param channel SDMA channel number.
 */
static inline void SDMA_StartChannelSoftware(SDMAARM_Type *base, uint32_t channel)
{
    base->HSTART = (1U << channel);
}

/*!
 * @brief Start a SDMA channel by hardware events.
 *
 * This function start a channel.
 *
 * @param base SDMA peripheral base address.
 * @param channel SDMA channel number.
 */
static inline void SDMA_StartChannelEvents(SDMAARM_Type *base, uint32_t channel)
{
    base->EVTPEND = (1U << channel);
}

/*!
 * @brief Stop a SDMA channel.
 *
 * This function stops a channel.
 *
 * @param base SDMA peripheral base address.
 * @param channel SDMA channel number.
 */
static inline void SDMA_StopChannel(SDMAARM_Type *base, uint32_t channel)
{
    base->STOP_STAT = (1U << channel);
}

/*!
 * @brief Set the SDMA context switch mode.
 *
 * @param base SDMA peripheral base address.
 * @param mode SDMA context switch mode.
 */
void SDMA_SetContextSwitchMode(SDMAARM_Type *base, sdma_context_switch_mode_t mode);

/*! @} */

/*!
 * @name SDMA Channel Status Operation
 * @{
 */

/*!
 * @brief Gets the SDMA interrupt status of all channels.
 *
 * @param base SDMA peripheral base address.
 * @return The interrupt status for all channels. Check the relevant bits for specific channel.
 */
static inline uint32_t SDMA_GetChannelInterruptStatus(SDMAARM_Type *base)
{
    return base->INTR;
}

/*!
 * @brief Clear the SDMA channel interrupt status of specific channels.
 *
 * @param base SDMA peripheral base address.
 * @param mask The interrupt status need to be cleared.
 */
static inline void SDMA_ClearChannelInterruptStatus(SDMAARM_Type *base, uint32_t mask)
{
    base->INTR = mask;
}

/*!
 * @brief Gets the SDMA stop status of all channels.
 *
 * @param base SDMA peripheral base address.
 * @return The stop status for all channels. Check the relevant bits for specific channel.
 */
static inline uint32_t SDMA_GetChannelStopStatus(SDMAARM_Type *base)
{
    return base->STOP_STAT;
}

/*!
 * @brief Clear the SDMA channel stop status of specific channels.
 *
 * @param base SDMA peripheral base address.
 * @param mask The stop status need to be cleared.
 */
static inline void SDMA_ClearChannelStopStatus(SDMAARM_Type *base, uint32_t mask)
{
    base->STOP_STAT = mask;
}

/*!
 * @brief Gets the SDMA channel pending status of all channels.
 *
 * @param base SDMA peripheral base address.
 * @return The pending status for all channels. Check the relevant bits for specific channel.
 */
static inline uint32_t SDMA_GetChannelPendStatus(SDMAARM_Type *base)
{
    return base->EVTPEND;
}

/*!
 * @brief Clear the SDMA channel pending status of specific channels.
 *
 * @param base SDMA peripheral base address.
 * @param mask The pending status need to be cleared.
 */
static inline void SDMA_ClearChannelPendStatus(SDMAARM_Type *base, uint32_t mask)
{
    base->EVTPEND = mask;
}

/*!
 * @brief Gets the SDMA channel error status.
 *
 * SDMA channel error flag is asserted while an incoming DMA request was detected and it triggers a channel
 * that is already pending or being serviced. This probably means there is an overflow of data for that channel.
 *
 * @param base SDMA peripheral base address.
 * @return The error status for all channels. Check the relevant bits for specific channel.
 */
static inline uint32_t SDMA_GetErrorStatus(SDMAARM_Type *base)
{
    return base->EVTERR;
}

/*!
 * @brief Gets the SDMA request source pending status.
 *
 * @param base SDMA peripheral base address.
 * @param source DMA request source number.
 * @return True means the request source is pending, otherwise not pending.
 */
bool SDMA_GetRequestSourceStatus(SDMAARM_Type *base, uint32_t source);

/*! @} */
/*!
 * @name SDMA Transactional Operation
 */

/*!
 * @brief Creates the SDMA handle.
 *
 * This function is called if using the transactional API for SDMA. This function
 * initializes the internal state of the SDMA handle.
 *
 * @param handle SDMA handle pointer. The SDMA handle stores callback function and parameters.
 * @param base SDMA peripheral base address.
 * @param channel SDMA channel number.
 * @param context Context structure for the channel to download into SDMA. Users shall make sure the context located
 * in a non-cacheable memory, or it will cause SDMA run fail. Users shall not touch the context contents, it only be
 * filled by SDMA driver in SDMA_SubmitTransfer function.
 */
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
void SDMA_CreateHandle(sdma_handle_t *handle, SDMAARM_Type *base, uint32_t channel, sdma_context_data_t *context);
#else
void SDMA_CreateHandle(sdma_handle_t *handle, SDMAARM_Type *base, uint32_t channel, sdma_context_data_t *context, sdma_channel_control_t *sdma_ch_ccb, uint32_t sdma_ch_bd_phys);
#endif

/*!
 * @brief Installs the BDs memory pool into the SDMA handle.
 *
 * This function is called after the SDMA_CreateHandle to use multi-buffer feature.
 *
 * @param handle SDMA handle pointer.
 * @param BDPool A memory pool to store BDs. It must be located in non-cacheable address.
 * @param BDCount The number of BD slots.
 */
void SDMA_InstallBDMemory(sdma_handle_t *handle, sdma_buffer_descriptor_t *BDPool, uint32_t BDCount);

/*!
 * @brief Installs a callback function for the SDMA transfer.
 *
 * This callback is called in the SDMA IRQ handler. Use the callback to do something after
 * the current major loop transfer completes.
 *
 * @param handle SDMA handle pointer.
 * @param callback SDMA callback function pointer.
 * @param userData A parameter for the callback function.
 */
void SDMA_SetCallback(sdma_handle_t *handle, sdma_callback callback, void *userData);

/*!
 * @brief multi fifo configurations.
 *
 * This api is used to support multi fifo for SDMA, if user want to get multi fifo data, then this api
 * should be called before submit transfer.
 *
 * @param config transfer configurations.
 * @param fifoNums fifo numbers that multi fifo operation perform.
 * @param fifoOffset offset between multififo address.
 * @param sampleNums number of audio channels in an audio frame.
 */
void SDMA_SetMultiFifoConfig(sdma_transfer_config_t *config, uint32_t fifoNums, uint32_t fifoOffset, uint32_t sampleNums);

/*!
 * @brief multi sai configurations.
 *
 * This api is used to support multiple SAI for SDMA
 *
 * @param sampleNums number of audio channels in an audio frame.
 */
void SDMA_SetMultiSaiConfig(sdma_transfer_config_t *config, uint32_t sampleNums);

/*!
 * @brief enable sdma sw done feature.
 *
 * @param base SDMA base.
 * @param config transfer configurations.
 * @param sel sw done selector.
 * @param type peripheral type is used to determine the corresponding peripheral sw done selector bit.
 */
void SDMA_EnableSwDone(SDMAARM_Type *base, sdma_transfer_config_t *config, uint8_t sel, sdma_peripheral_t type);

/*!
 * @brief load script to sdma program memory.
 *
 * @param base SDMA base.
 * @param destAddr dest script address, should be SDMA program memory address.
 * @param srcAddr source address of target script.
 * @param bufferSizeBytes bytes size of script.
 */
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
void SDMA_LoadScript(SDMAARM_Type *base, uint32_t destAddr, void *srcAddr, size_t bufferSizeBytes);
#else
void SDMA_LoadScript(SDMAARM_Type *base, uint32_t destAddr, void *srcAddr, size_t bufferSizeBytes, struct sdma_buffer_descriptor *bd_ch);
#endif /* FSL_SDK_DISABLE_IRQ */

/*!
 * @brief dump script from sdma program memory.
 *
 * @param base SDMA base.
 * @param srcAddr should be SDMA program memory address.
 * @param destAddr address to store scripts.
 * @param bufferSizeBytes bytes size of script.
 */
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
void SDMA_DumpScript(SDMAARM_Type *base, uint32_t srcAddr, void *destAddr, size_t bufferSizeBytes);
#else
void SDMA_DumpScript(SDMAARM_Type *base, uint32_t srcAddr, void *destAddr, size_t bufferSizeBytes, struct sdma_buffer_descriptor *bd_ch);
#endif /* FSL_SDK_DISABLE_IRQ */

/*!
 * @brief Prepares the SDMA transfer structure.
 *
 * This function prepares the transfer configuration structure according to the user input.
 *
 * @param config The user configuration structure of type sdma_transfer_t.
 * @param srcAddr SDMA transfer source address.
 * @param destAddr SDMA transfer destination address.
 * @param srcWidth SDMA transfer source address width(bytes).
 * @param destWidth SDMA transfer destination address width(bytes).
 * @param bytesEachRequest SDMA transfer bytes per channel request.
 * @param transferSize SDMA transfer bytes to be transferred.
 * @param eventSource Event source number for the transfer, if use software trigger, just write 0.
 * @param peripheral Peripheral type, used to decide if need to use some special scripts.
 * @param type SDMA transfer type. Used to decide the correct SDMA script address in SDMA ROM.
 * @note The data address and the data width must be consistent. For example, if the SRC
 *       is 4 bytes, the source address must be 4 bytes aligned, or it results in
 *       source address error.
 */
void SDMA_PrepareTransfer(sdma_transfer_config_t *config,
                          uint32_t srcAddr,
                          uint32_t destAddr,
                          uint32_t srcWidth,
                          uint32_t destWidth,
                          uint32_t bytesEachRequest,
                          uint32_t period_len,
                          uint32_t transferSize,
                          uint32_t eventSource,
                          enum sdma_peripheral peripheral,
                          enum sdma_transfer_type type,
                          struct sdma_fw_script_start_addrs *script_addrs);

/*!
 * @brief Set SDMA Runnable Channel Selection Control.
 *
 * This function selects which trigger conditions that must occur for the channel to start.
 *
 * @param handle SDMA handle pointer.
 * @param config Pointer to SDMA transfer configuration structure.
 */
void SDMA_Config_Ownership(sdma_handle_t *handle, const sdma_transfer_config_t *config);

/*!
 * @brief Set SDMA Channel Enable Registers.
 *
 * This function selects which DMA request is activated for a DMA channel.
 *
 * @param handle SDMA handle pointer.
 * @param eventSource DMA request to enable for the handle's channel.
 */
void SDMA_Event_Enable(sdma_handle_t *handle, uint32_t eventSource);
void SDMA_Event_Disable(sdma_handle_t *handle, uint32_t eventSource);

/*!
 * @brief Submits the SDMA transfer request.
 *
 * This function submits the SDMA transfer request according to the transfer configuration structure.
 *
 * @param handle SDMA handle pointer.
 * @param config Pointer to SDMA transfer configuration structure.
 */
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
void SDMA_SubmitTransfer(sdma_handle_t *handle, const sdma_transfer_config_t *config);
#else
void SDMA_SubmitTransfer(sdma_handle_t *handle, const sdma_transfer_config_t *config, sdma_buffer_descriptor_t *bd_ch, sdma_buffer_descriptor_t *bd0, paddr_t context_phys);
#endif /* FSL_SDK_DISABLE_IRQ */

/*!
 * @brief Setup the buffer descriptors for a memory-to-memory transfer.
 *
 * This function select creates buffer descriptors for a memcpy command.
 *
 * @param handle SDMA handle pointer.
 * @param config Pointer to SDMA transfer configuration structure.
 * @param bd_ch array of Buffer Descriptors.
 */
void SDMA_PrepMemcpy(sdma_handle_t *handle, const sdma_transfer_config_t *config, sdma_buffer_descriptor_t *bd_ch);

/*!
 * @brief Setup the buffer descriptors for a peripheral to/from memory transfer.
 *
 * This function select creates buffer descriptors for a peripheral to/from memory command.
 *
 * @param handle SDMA handle pointer.
 * @param config Pointer to SDMA transfer configuration structure.
 * @param bd_ch array of Buffer Descriptors.
 */
void SDMA_PrepDmaCyclic(sdma_handle_t *handle, const sdma_transfer_config_t *config, struct sdma_buffer_descriptor *bd_ch);


/*!
 * @brief SDMA enables a channel.
 *
 * This function enables the channel request. Users can call this function after submitting the transfer request
 * or before submitting the transfer request.
 *
 * @param handle SDMA handle pointer.
 */
void SDMA_EnableChannel(sdma_handle_t *handle);

/*!
 * @brief SDMA disables a channel.
 *
 * This function disables the channel request to pause the transfer. Users can call SDMA_StartTransfer()
 * again to resume the transfer.
 *
 * @param handle SDMA handle pointer.
 */
void SDMA_DisableChannel(sdma_handle_t *handle);

/*!
 * @brief Get transferred bytes while not using BD pools.
 *
 * This function returns the buffer descriptor count value if not using buffer descriptor.
 * While do a simple transfer, which only uses one descriptor, the SDMA driver inside handle the
 * buffer descriptor. In uart receive case, it can tell users how many data already received, also
 * it can tells users how many data transfferd while error occurred.
 * Notice, the count would not change while transfer is on-going using default SDMA script.
 *
 * @param handle DMA handle pointer.
 * @return Transferred bytes.
 */
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
uint32_t SDMA_GetTransferredBytes(sdma_handle_t *handle);
#else
uint32_t SDMA_GetTransferredBytes(sdma_handle_t *handle, sdma_buffer_descriptor_t *sdma_ch_bd);
#endif /* FSL_SDK_DISABLE_IRQ */

#if defined FSL_FEATURE_SOC_SPBA_COUNT && (FSL_FEATURE_SOC_SPBA_COUNT > 0)
/*!
 * @brief Judge if address located in SPBA.
 *
 * @param addr Address which need to judge.
 * @param return True means located in SPBA, false means not.
 */
bool SDMA_IsPeripheralInSPBA(uint32_t addr);
#endif /* FSL_FEATURE_SOC_SPBA_COUNT */

/*!
 * @brief SDMA IRQ handler for complete a buffer descriptor transfer.
 *
 * This function clears the interrupt flags and also handle the CCB for the channel.
 *
 * @param handle SDMA handle pointer.
 */
void SDMA_HandleIRQ(sdma_handle_t *handle);

/*!
 * @brief Load the SDMA context from ARM memory into SDMA RAM region.
 *
 * @param base SDMA peripheral base address.
 * @param channel SDMA channel number.
 * @param tcd Point to TCD structure.
 */
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
void SDMA_LoadContext(sdma_handle_t *handle, const sdma_transfer_config_t *config);
#else
void SDMA_LoadContext(sdma_handle_t *handle, const sdma_transfer_config_t *config, sdma_buffer_descriptor_t *bd0, paddr_t context_phys);
#endif /* FSL_SDK_DISABLE_IRQ */

/*!
 * @brief Get channel SDMA Context from SDMA's RAM.
 *
 * This function copies a SDMA context from SDMA RAM to a buffer
 * at <context_phys>.
 *
 * @param handle SDMA handle pointer.
 * @param context context retrieved from SDMA's RAM
 */
void SDMA_GetContext(sdma_handle_t *handle, sdma_context_data_t *context);

/* @} */

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/* @} */

#endif /*_SDMA_HW_H_*/
