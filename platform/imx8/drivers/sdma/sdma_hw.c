/*
 * Copyright 2017, 2019-2020 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <debug.h>
#include "sdma_hw.h"
#include "delay.h"

#if (defined(FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET) && FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET)
#include "fsl_memory.h"
#endif
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define min(a, b)   (a) < (b) ? a : b


/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.sdma"
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief Get instance number for SDMA.
 *
 * @param base SDMA peripheral base address.
 */
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
static uint32_t SDMA_GetInstance(SDMAARM_Type *base);
#endif /* FSL_SDK_DISABLE_IRQ */

/*!
 * @brief Run scripts for channel0.
 *
 * Channel0 is by default used as the boot channel for SDMA, also the scripts for channel0 will download scripts
 * for other channels from ARM platform to SDMA RAM context.
 *
 * @param base SDMA peripheral base address.
 */
static void SDMA_RunChannel0(SDMAARM_Type *base);

/*******************************************************************************
 * Variables
 ******************************************************************************/
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
/*! @brief Array to map SDMA instance number to base pointer. */
static SDMAARM_Type *const s_sdmaBases[] = SDMAARM_BASE_PTRS;
#endif

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/*! @brief Array to map SDMA instance number to clock name. */
static const clock_ip_name_t s_sdmaClockName[] = SDMA_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
/*! @brief Array to map SDMA instance number to IRQ number. */
static const IRQn_Type s_sdmaIRQNumber[FSL_FEATURE_SOC_SDMA_COUNT] = SDMAARM_IRQS;
#endif /* FSL_SDK_DISABLE_IRQ */

#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
/*! @brief Pointers to transfer handle for each SDMA channel. */
static sdma_handle_t *s_SDMAHandle[FSL_FEATURE_SOC_SDMA_COUNT][FSL_FEATURE_SDMA_MODULE_CHANNEL];

/*! @brief channel 0 Channel control block */
/* TODO: this one was 'AT_NONCACHEABLE_SECTION_ALIGN' */
static sdma_channel_control_t s_SDMACCB[FSL_FEATURE_SOC_SDMA_COUNT][FSL_FEATURE_SDMA_MODULE_CHANNEL] __attribute__((aligned(4));

/*! @brief channel 0 buffer descriptor */
/* TODO: this one was 'AT_NONCACHEABLE_SECTION_ALIGN' */
static sdma_buffer_descriptor_t s_SDMABD[FSL_FEATURE_SOC_SDMA_COUNT][FSL_FEATURE_SDMA_MODULE_CHANNEL] __attribute__((aligned(4));
#endif /* FSL_SDK_DISABLE_IRQ */

/*******************************************************************************
 * Code
 ******************************************************************************/
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
static uint32_t SDMA_GetInstance(SDMAARM_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < ARRAY_SIZE(s_sdmaBases); instance++)
    {
        if (s_sdmaBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < ARRAY_SIZE(s_sdmaBases));

    return instance;
}
#endif /* FSL_SDK_DISABLE_IRQ */

static void SDMA_RunChannel0(SDMAARM_Type *base)
{
    /* Start channel 0 */
    SDMA_StartChannelSoftware(base, 0U);

    /* Waiting for channel 0 finished */
    while ((base->STOP_STAT & 0x1) == 1U)
    {
    }

    /* Clear the channel interrupt status */
    SDMA_ClearChannelInterruptStatus(base, 0x1U);

    /* Set SDMA context switch to dynamic switching */
    SDMA_SetContextSwitchMode(base, kSDMA_ContextSwitchModeDynamic);
}

static uint32_t SDMA_GetScriptAddr(enum sdma_peripheral peripheral, enum sdma_transfer_type type, struct sdma_fw_script_start_addrs *script_addrs)
{
    uint32_t val = 0;

    if (type == kSDMA_MemoryToMemory)
    {
        val = FSL_FEATURE_SDMA_M2M_ADDR;
    }
    else if (type == kSDMA_MemoryToPeripheral)
    {
        switch (peripheral)
        {
            case kSDMA_PeripheralTypeUART:
            case kSDMA_PeripheralNormal:
                val = script_addrs->mcu_2_app_addr;
                break;
            case kSDMA_PeripheralTypeUART_SP:
            case kSDMA_PeripheralNormal_SP:
                val = script_addrs->mcu_2_shp_addr;
                break;
            case kSDMA_PeripheralTypeSPDIF:
                val = script_addrs->mcu_2_spdif_addr;
                break;
            case kSDMA_PeripheralMultiFifoSaiTX:
                val = script_addrs->mcu_2_sai_addr;
                break;
            case kSDMA_PeripheralOpt8FifoSaiTX:
                val = script_addrs->zz8l_2_sai;
                break;
            case kSDMA_PeripheralMultiSaiTX:
                val = script_addrs->mcu_2_multi_sai;
                break;
            default:
                break;
        }
    }
    else
    {
        switch (peripheral)
        {
            case kSDMA_PeripheralTypeUART:
                val = script_addrs->uart_2_mcu_addr;
                break;
            case kSDMA_PeripheralNormal:
                val = script_addrs->app_2_mcu_addr;
                break;
            case kSDMA_PeripheralTypeUART_SP:
                val = script_addrs->uartsh_2_mcu_addr;
                break;
            case kSDMA_PeripheralNormal_SP:
                val = script_addrs->shp_2_mcu_addr;
                break;
            case kSDMA_PeripheralTypeSPDIF:
                val = script_addrs->spdif_2_mcu_addr;
                break;
            case kSDMA_PeripheralMultiFifoPDM:
            case kSDMA_PeripheralMultiFifoSaiRX:
                val = script_addrs->sai_2_mcu_addr;
                break;
            case kSDMA_PeripheralOpt4FifoSaiRX:
                val = script_addrs->zz4l_2_mcu;
                break;
            case kSDMA_PeripheralOpt2LaneMic:
                val = script_addrs->zzmic2_2_mcu;
                break;
            case kSDMA_PeripheralOpt4LaneMic:
                val = script_addrs->zzmic4_2_mcu;
                break;
            case kSDMA_PeripheralMultiSaiRX:
                val = script_addrs->multi_sai_2_mcu;
                break;
            default:
                break;
        }
    }

    return val;
}

#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
void SDMA_LoadContext(sdma_handle_t *handle, const sdma_transfer_config_t *config)
#else
void SDMA_LoadContext(sdma_handle_t *handle, const sdma_transfer_config_t *config, struct sdma_buffer_descriptor *bd0, paddr_t context_phys)
#endif /* FSL_SDK_DISABLE_IRQ */
{
    struct sdma_buffer_descriptor *l_bd0;
    uint32_t l_context_paddr;

#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
    uint32_t instance = SDMA_GetInstance(handle->base);
    l_bd0 = &s_SDMABD[instance][0];
    l_context_paddr = (uint32_t)context;
#else
    assert(bd0 != NULL);
    l_bd0 = bd0;
    /* context physical address */
    l_context_paddr = (uint32_t)((uintptr_t)context_phys);
#endif /* FSL_SDK_DISABLE_IRQ */

    sdma_context_data_t *context = handle->context;
    assert(context != NULL);

    memset(context, 0, sizeof(sdma_context_data_t));

    /* Set SDMA core's PC to the channel script address */
    context->PC = config->scriptAddr;
    printlk(LK_DEBUG, "PC=0x%08x\n", context->PC);

    /* Set the request source into context */
    if (config->eventSource >= 32)
    {
        context->GeneralReg[0] = (1U << (config->eventSource - 32));
    }
    else
    {
        context->GeneralReg[1] = (1U << config->eventSource);
    }
    printlk(LK_DEBUG, "GR[0]=0x%08x\n", context->GeneralReg[0]);
    printlk(LK_DEBUG, "GR[1]=0x%08x\n", context->GeneralReg[1]);

    /* Set source address and dest address for p2p, m2p and p2m */
    if (config->type == kSDMA_MemoryToPeripheral)
    {
        context->GeneralReg[2] = (uint32_t)config->destAddr;
        context->GeneralReg[6] = (uint32_t)config->destAddr;
    }
    else
    {
        context->GeneralReg[2] = (uint32_t)config->srcAddr;
        context->GeneralReg[6] = (uint32_t)config->srcAddr;
    }
    ASSERT(context->GeneralReg[2] != 0);
    printlk(LK_DEBUG, "GR[2]=0x%08x\n", context->GeneralReg[2]);
    printlk(LK_DEBUG, "config->bytesPerRequest=0x%x\n", config->bytesPerRequest);

    /* Set watermark, multi fifo for p2p, m2p and p2m into context */
    context->GeneralReg[7] =
        ((config->bytesPerRequest & kSDMA_MultiFifoWatermarkLevelMask) << kSDMA_MultiFifoWatermarkLevelShift) |
        ((config->multiFifo.fifoNums & kSDMA_MultiFifoNumsMask) << kSDMA_MultiFifoNumsShift) |
        ((config->multiFifo.fifoOffset & kSDMA_MultiFifoOffsetMask) << kSDMA_MultiFifoOffsetShift) |
        ((config->swDone.enableSwDone & kSDMA_MultiFifoSwDoneMask) << kSDMA_MultiFifoSwDoneShift) |
        ((config->swDone.swDoneSel & kSDMA_MultiFifoSwDoneSelectorMask) << kSDMA_MultiFifoSwDoneSelectorShift);

    if (config->multiFifo.sampleNums) {
        /* SAI multiFIFO mode */
        context->GeneralReg[7] |= (
            ((config->multiFifo.sampleNums - 1) & kSDMA_MultiFifoNumsSampleMask)
            << kSDMA_MultiFifoNumsSampleShift);
    } else if (config->multiSai.sampleNums) {
        /* SAI multi SAI mode, reconfigure GeneralReg 7 */
        context->GeneralReg[7] =
            ((config->bytesPerRequest & kSDMA_MultiSaiWatermarkLevelMask) << kSDMA_MultiSaiWatermarkLevelShift) |
            (((config->multiSai.sampleNums - 1) & kSDMA_MultiSaiNumsSampleMask)
             << kSDMA_MultiSaiNumsSampleShift);
    }
    printlk(LK_DEBUG, "GR[7]=0x%08x\n", context->GeneralReg[7]);

    /* Load SDMA context data from platform memory buffer */
    l_bd0->command = kSDMA_BDCommandSETDM;
    l_bd0->status = kSDMA_BDStatusDone | kSDMA_BDStatusWrap | kSDMA_BDStatusInterrupt;
    l_bd0->count = sizeof(*context) / 4U;
    /* context physical address */
#if (defined(FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET) && FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET)
    l_bd0->bufferAddr = MEMORY_ConvertMemoryMapAddress((uint32_t)context, kMEMORY_Local2DMA);
#else
    l_bd0->bufferAddr = (uint32_t)l_context_paddr;
#endif
    /* SDMA memory destination address for this context */
    l_bd0->extendBufferAddr = 2048 + (sizeof(*context) / 4) * handle->channel;

    /* Run channel0 scripts after context prepared */
    SDMA_RunChannel0(handle->base);
}

/*!
 * @brief Get SDMA Context for an handle.
 *
 * This function copies a SDMA context from SDMA RAM to a buffer
 * at <context_phys>.
 *
 * @param handle SDMA handle pointer.
 * @param context pointer to a context retrieved from SDMA's RAM
 */
void SDMA_GetContext(sdma_handle_t *handle, sdma_context_data_t *context)
{
    assert(context != NULL);
    /* Channel 0 Buffer Descriptor */
    struct sdma_buffer_descriptor *bd0 = handle->bd0;
    assert(bd0 != NULL);

    /* context physical address */
    //l_context_paddr = (uint32_t)((uintptr_t)context_phys);

    sdma_context_data_t *ch_context = handle->context;
    assert(ch_context != NULL);

    memset(ch_context, 0, sizeof(sdma_context_data_t));

    /* Get SDMA context data from platform memory buffer */
#if 0
    bd0->command = SDMA_COMMAND_GET_CONTEXT(handle->channel);
    bd0->status = kSDMA_BDStatusDone | kSDMA_BDStatusInterrupt;
#endif
#if 1
    bd0->command = kSDMA_BDCommandGETDM;
    bd0->status = kSDMA_BDStatusDone | kSDMA_BDStatusWrap | kSDMA_BDStatusInterrupt;
#endif
    bd0->count = sizeof(sdma_context_data_t) / 4U;
    /* context physical address */
    bd0->bufferAddr = handle->context_phys; /*(uint32_t)((uintptr_t)context_phys);*/
#if 1
    /* SDMA memory destination address for this context */
    bd0->extendBufferAddr = 2048 + (sizeof(sdma_context_data_t) / 4) * handle->channel;
#endif

    /* Run channel0 scripts after context prepared */
    SDMA_RunChannel0(handle->base);

    /* copy context to users's context variable */
    memcpy(context, handle->context, sizeof(sdma_context_data_t));
}

/*!
 * @brief Dump SDMA Context.
 *
 * This function dumps a SDMA context.
 *
 * @param context to dump.
 */
void SDMA_DumpContext(sdma_context_data_t context)
{
    printf("sdma_context_data_t @%p:\n", &context);
    printf("  PC   0x%08x T    0x%08x RPC  0x%08x SF   0x%08x\n", context.PC, context.T, context.RPC, context.SF);
    printf("  SPC  0x%08x DF   0x%08x EPC  0x%08x LM   0x%08x\n", context.SPC, context.DF, context.EPC, context.LM);
    unsigned int i;
    for (i = 0; i < 8; i++) {
        printf("  GR_%d 0x%08x\n", i, context.GeneralReg[i]);
    }
    printf("  MDA  0x%08x MSA  0x%08x\n", context.MDA, context.MSA);
    printf("  MS   0x%08x MD   0x%08x\n", context.MS,  context.MD);
    printf("  PDA  0x%08x PSA  0x%08x\n", context.PDA, context.PSA);
    printf("  PS   0x%08x PD   0x%08x\n", context.PS,  context.PD);
    printf("  CA   0x%08x CS   0x%08x\n", context.CA,  context.CS);
    printf("  DDA  0x%08x DSA  0x%08x\n", context.DDA, context.DSA);
    printf("  DS   0x%08x DD   0x%08x\n", context.DS,  context.DD);
}

#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
void SDMA_LoadScript(SDMAARM_Type *base, uint32_t destAddr, void *srcAddr, size_t bufferSizeBytes)
#else
void SDMA_LoadScript(SDMAARM_Type *base, uint32_t destAddr, void *srcAddr, size_t bufferSizeBytes, struct sdma_buffer_descriptor *bd_ch)
#endif /* FSL_SDK_DISABLE_IRQ */
{
    ASSERT(base != NULL);
    ASSERT(destAddr != 0);
    ASSERT(srcAddr != NULL);
    ASSERT(bufferSizeBytes != 0);
    struct sdma_buffer_descriptor *bd;
    /* Set the descriptor to 0 */
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
    uint32_t instance = SDMA_GetInstance(base);
    bd = &s_SDMABD[instance][0];
#else
    ASSERT(bd_ch != NULL);
    bd = &bd_ch[0];
#endif /* FSL_SDK_DISABLE_IRQ */
    uint32_t bufferAddr = 0U;

#if (defined(FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET) && FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET)
    bufferAddr = MEMORY_ConvertMemoryMapAddress((uint32_t)srcAddr, kMEMORY_Local2DMA);
#else
    bufferAddr = (uint32_t)((uintptr_t)srcAddr);
#endif

    bd->command = kSDMA_BDCommandSETPM;
    bd->status = kSDMA_BDStatusDone | kSDMA_BDStatusWrap | kSDMA_BDStatusExtend;
    bd->count = bufferSizeBytes;
    bd->bufferAddr = bufferAddr;
    bd->extendBufferAddr = destAddr;

    /* Run channel0 scripts */
    SDMA_RunChannel0(base);
}

#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
void SDMA_DumpScript(SDMAARM_Type *base, uint32_t srcAddr, void *destAddr, size_t bufferSizeBytes)
#else
void SDMA_DumpScript(SDMAARM_Type *base, uint32_t srcAddr, void *destAddr, size_t bufferSizeBytes, struct sdma_buffer_descriptor *bd_ch)
#endif /* FSL_SDK_DISABLE_IRQ */
{
    struct sdma_buffer_descriptor *bd;
    /* Set the descriptor to 0 */
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
    uint32_t instance = SDMA_GetInstance(base);
    bd = &s_SDMABD[instance][0];
#else
    bd = &bd_ch[0];
#endif /* FSL_SDK_DISABLE_IRQ */
    uint32_t bufferAddr = 0U;

#if (defined(FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET) && FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET)
    bufferAddr = MEMORY_ConvertMemoryMapAddress((uint32_t)destAddr, kMEMORY_Local2DMA);
#else
    bufferAddr = (uint32_t)((uintptr_t)destAddr);
#endif

    bd->command = kSDMA_BDCommandGETPM;
    bd->status = kSDMA_BDStatusDone | kSDMA_BDStatusWrap | kSDMA_BDStatusExtend;
    bd->count = bufferSizeBytes;
    bd->bufferAddr = bufferAddr;
    bd->extendBufferAddr = srcAddr;

    /* Run channel0 scripts */
    SDMA_RunChannel0(base);
}

#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
#if defined FSL_FEATURE_SOC_SPBA_COUNT && (FSL_FEATURE_SOC_SPBA_COUNT > 0)
bool SDMA_IsPeripheralInSPBA(uint32_t addr)
{
    uint32_t spbaNum = FSL_FEATURE_SOC_SPBA_COUNT;
    uint32_t i = 0;
    SPBA_Type *spbaBase;
    SPBA_Type *spbaArray[FSL_FEATURE_SOC_SPBA_COUNT] = SPBA_BASE_PTRS;

    for (i = 0; i < spbaNum; i++)
    {
        spbaBase = spbaArray[i];
        if ((addr >= FSL_FEATURE_SPBA_STARTn(spbaBase)) && (addr <= FSL_FEATURE_SPBA_ENDn(spbaBase)))
        {
            return true;
        }
    }

    return false;
}
#endif /* FSL_FEATURE_SOC_SPBA_COUNT */
#endif /* FSL_SDK_DISABLE_IRQ */

#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
void SDMA_Init(SDMAARM_Type *base, const sdma_config_t *config)
#else
void SDMA_Init(SDMAARM_Type *base, const sdma_config_t *config, sdma_channel_control_t *sdma_ccb, paddr_t sdma_ccb0_phys, struct sdma_buffer_descriptor *sdma_bd, paddr_t sdma_bd0_phys)
#endif
{
    assert(config != NULL);

    uint32_t tmpreg;
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
    uint32_t instance = SDMA_GetInstance(base);

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Ungate SDMA peripheral clock */
    CLOCK_EnableClock(s_sdmaClockName[instance]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

    /* Clear the channel CCB */
    memset(&s_SDMACCB[instance][0], 0, sizeof(sdma_channel_control_t) * FSL_FEATURE_SDMA_MODULE_CHANNEL);
#else
    /* Clear the channel CCB */
    memset(sdma_ccb, 0, sizeof(sdma_channel_control_t) * FSL_FEATURE_SDMA_MODULE_CHANNEL);
#endif /* FSL_SDK_DISABLE_IRQ */

    /* Reset all SDMA registers */
    SDMA_ResetModule(base);

    /* Init the CCB for channel 0 */
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
    memset(&s_SDMABD[instance][0], 0, sizeof(sdma_buffer_descriptor_t));
#if (defined(FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET) && FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET)
    s_SDMACCB[instance][0].currentBDAddr =
        MEMORY_ConvertMemoryMapAddress((uint32_t)(&s_SDMABD[instance][0]), kMEMORY_Local2DMA);
    s_SDMACCB[instance][0].baseBDAddr =
        MEMORY_ConvertMemoryMapAddress((uint32_t)(&s_SDMABD[instance][0]), kMEMORY_Local2DMA);
#else
    s_SDMACCB[instance][0].currentBDAddr = (uint32_t)(&s_SDMABD[instance][0]);
    s_SDMACCB[instance][0].baseBDAddr = (uint32_t)(&s_SDMABD[instance][0]);
#endif
#else
    memset(sdma_bd, 0, sizeof(struct sdma_buffer_descriptor));
    sdma_ccb[0].currentBDAddr = (uint32_t)((uintptr_t)sdma_bd0_phys);
    sdma_ccb[0].baseBDAddr    = (uint32_t)((uintptr_t)sdma_bd0_phys);
#endif /* FSL_SDK_DISABLE_IRQ */

    /* Set channel 0 priority */
    SDMA_SetChannelPriority(base, 0, 7U);

    /* Set channel 0 ownership */
    base->HOSTOVR = 0U;
    base->EVTOVR = 1U;

    /* Configure SDMA peripheral according to the configuration structure. */
    tmpreg = base->CONFIG;
    tmpreg &= ~(SDMAARM_CONFIG_ACR_MASK | SDMAARM_CONFIG_RTDOBS_MASK);
    /* Channel 0 shall use static context switch method */
    tmpreg |= (SDMAARM_CONFIG_ACR(config->ratio) | SDMAARM_CONFIG_RTDOBS(config->enableRealTimeDebugPin) |
               SDMAARM_CONFIG_CSM(0U));
    base->CONFIG = tmpreg;

    tmpreg = base->SDMA_LOCK;
    tmpreg &= ~SDMAARM_SDMA_LOCK_SRESET_LOCK_CLR_MASK;
    tmpreg |= SDMAARM_SDMA_LOCK_SRESET_LOCK_CLR(config->isSoftwareResetClearLock);
    base->SDMA_LOCK = tmpreg;

    /* Set the context size to 32 bytes */
    base->CHN0ADDR = 0x4050U;

    /* Set channel 0 CCB address */
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
#if (defined(FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET) && FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET)
    base->MC0PTR = MEMORY_ConvertMemoryMapAddress((uint32_t)(&s_SDMACCB[instance][0]), kMEMORY_Local2DMA);
#else
    base->MC0PTR = (uint32_t)(&s_SDMACCB[instance][0]);
#endif
#else
    base->MC0PTR = (uint32_t)((uintptr_t)sdma_ccb0_phys);
#endif /* FSL_SDK_DISABLE_IRQ */
}

void SDMA_Deinit(SDMAARM_Type *base)
{
    /* Clear the MC0PTR register */
    base->MC0PTR = 0U;
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Gate SDMA periphral clock */
    CLOCK_DisableClock(s_sdmaClockName[SDMA_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

void SDMA_GetDefaultConfig(sdma_config_t *config)
{
    assert(config != NULL);

    config->enableRealTimeDebugPin = false;
    config->isSoftwareResetClearLock = true;
    config->ratio = kSDMA_HalfARMClockFreq;
}

void SDMA_ResetModule(SDMAARM_Type *base)
{
    uint32_t i = 0, status;

    base->RESET = 1;
    while (base->RESET) {
        udelay(1);
    }
    base->MC0PTR = 0;
    status = base->INTR;
    SDMA_ClearChannelInterruptStatus(base, status);
    status = base->STOP_STAT;
    SDMA_ClearChannelStopStatus(base, status);
    base->EVTOVR = 0;
    base->DSPOVR = 0xFFFFFFFFU;
    base->HOSTOVR = 0;
    status = base->EVTPEND;
    SDMA_ClearChannelPendStatus(base, status);
    base->INTRMASK = 0;

    /* Disable all events */
    for (i = 0; i < FSL_FEATURE_SDMA_EVENT_NUM; i++)
    {
        SDMA_SetSourceChannel(base, i, 0);
    }

    /* Clear all channel priority */
    for (i = 0; i < FSL_FEATURE_SDMA_MODULE_CHANNEL; i++)
    {
        SDMA_SetChannelPriority(base, i, 0);
    }
}

void SDMA_ConfigBufferDescriptor(struct sdma_buffer_descriptor *bd,
                                 uint32_t srcAddr,
                                 uint32_t destAddr,
                                 sdma_transfer_size_t busWidth,
                                 size_t bufferSize,
                                 bool isLast,
                                 bool enableInterrupt,
                                 bool isWrap,
                                 enum sdma_transfer_type type)
{
    printlk(LK_NOTICE, "%s: entry\n", __PRETTY_FUNCTION__);
    printlk(LK_NOTICE, "%s:%d: destAddr=0x%x, bufferSize=0x%lx, isLast=%d, isWrap=%d.\n",
            __PRETTY_FUNCTION__, __LINE__, destAddr, bufferSize,
            isLast, isWrap);
    /* Set the descriptor to 0 */
    memset(bd, 0, sizeof(struct sdma_buffer_descriptor));
    if (type == kSDMA_PeripheralToMemory)
    {
#if (defined(FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET) && FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET)
        bd->bufferAddr = MEMORY_ConvertMemoryMapAddress(destAddr, kMEMORY_Local2DMA);
#else
        bd->bufferAddr = destAddr;
#endif
    }
    else
    {
#if (defined(FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET) && FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET)
        bd->bufferAddr = MEMORY_ConvertMemoryMapAddress(srcAddr, kMEMORY_Local2DMA);
#else
        bd->bufferAddr = srcAddr;
#endif
    }
#if (defined(FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET) && FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET)
    bd->extendBufferAddr = MEMORY_ConvertMemoryMapAddress(destAddr, kMEMORY_Local2DMA);
#else
    bd->extendBufferAddr = destAddr;
#endif
    if (isLast)
    {
        bd->status |= kSDMA_BDStatusLast;
    }
    else
    {
        bd->status |= kSDMA_BDStatusContinuous;
    }

    /* Set interrupt and wrap feature */
    if (enableInterrupt)
    {
        bd->status |= kSDMA_BDStatusInterrupt;
    }
    if (isWrap)
    {
        bd->status |= kSDMA_BDStatusWrap;
    }

    bd->status |= kSDMA_BDStatusDone;

    /* Configure the command according to bus width */
    bd->command = busWidth;
    bd->count = bufferSize;
    printlk(LK_NOTICE, "%s: exit\n", __PRETTY_FUNCTION__);
}

void SDMA_SetContextSwitchMode(SDMAARM_Type *base, sdma_context_switch_mode_t mode)
{
    uint32_t val = base->CONFIG & (~SDMAARM_CONFIG_CSM_MASK);
    val |= mode;
    base->CONFIG = val;
}

bool SDMA_GetRequestSourceStatus(SDMAARM_Type *base, uint32_t source)
{
    if (source < 32U)
    {
        return ((base->EVT_MIRROR & (1U << source)) >> source);
    }
    else
    {
        source -= 32U;
        return ((base->EVT_MIRROR2 & (1U << source)) >> source);
    }
}

#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
void SDMA_CreateHandle(sdma_handle_t *handle, SDMAARM_Type *base, uint32_t channel, sdma_context_data_t *context)
#else
/* deprecated function: need fields added to sdma_handle_t */
void SDMA_CreateHandle(sdma_handle_t *handle, SDMAARM_Type *base, uint32_t channel, sdma_context_data_t *context, sdma_channel_control_t *sdma_ch_ccb, uint32_t sdma_ch_bd_phys)
#endif
{
    assert(handle != NULL);
    assert(channel < FSL_FEATURE_SDMA_MODULE_CHANNEL);

#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
    /* Get the DMA instance number */
    uint32_t sdmaInstance;
    sdmaInstance = SDMA_GetInstance(base);
#endif /* FSL_SDK_DISABLE_IRQ */

    /* Zero the handle */
    memset(handle, 0, sizeof(*handle));

    handle->base = base;
    handle->channel = channel;
    handle->bdCount = 1U;
    handle->context = context;
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
    s_SDMAHandle[sdmaInstance][channel] = handle;
#endif /* FSL_SDK_DISABLE_IRQ */

/* Set channel CCB, default is the static buffer descriptor if not use EDMA_InstallBDMemory */
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
#if (defined(FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET) && FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET)
    s_SDMACCB[sdmaInstance][channel].currentBDAddr =
        MEMORY_ConvertMemoryMapAddress((uint32_t)(&s_SDMABD[sdmaInstance][channel]), kMEMORY_Local2DMA);
    s_SDMACCB[sdmaInstance][channel].baseBDAddr =
        MEMORY_ConvertMemoryMapAddress((uint32_t)(&s_SDMABD[sdmaInstance][channel]), kMEMORY_Local2DMA);
#else
    s_SDMACCB[sdmaInstance][channel].baseBDAddr = (uint32_t)(&s_SDMABD[sdmaInstance][channel]);
    s_SDMACCB[sdmaInstance][channel].currentBDAddr = (uint32_t)(&s_SDMABD[sdmaInstance][channel]);
#endif
#else
    sdma_ch_ccb->baseBDAddr    = sdma_ch_bd_phys;
    sdma_ch_ccb->currentBDAddr = sdma_ch_bd_phys;
#endif /* FSL_SDK_DISABLE_IRQ */

    /* Enable interrupt */
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
    EnableIRQ(s_sdmaIRQNumber[sdmaInstance]);
#else
#endif /* FSL_SDK_DISABLE_IRQ */
}

#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
void SDMA_InstallBDMemory(sdma_handle_t *handle, sdma_buffer_descriptor_t *BDPool, uint32_t BDCount)
{
    assert(handle && BDPool && BDCount);

    uint32_t sdmaInstance = SDMA_GetInstance(handle->base);

    /* Send user defined buffer descriptor pool to handle */
    handle->BDPool = BDPool;

    handle->bdCount = BDCount;

/* Update the CCB contents */
#if (defined(FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET) && FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET)
    s_SDMACCB[sdmaInstance][handle->channel].baseBDAddr =
        MEMORY_ConvertMemoryMapAddress((uint32_t)(handle->BDPool), kMEMORY_Local2DMA);
    s_SDMACCB[sdmaInstance][handle->channel].currentBDAddr =
        MEMORY_ConvertMemoryMapAddress((uint32_t)(handle->BDPool), kMEMORY_Local2DMA);
#else
    s_SDMACCB[sdmaInstance][handle->channel].baseBDAddr = (uint32_t)(handle->BDPool);
    s_SDMACCB[sdmaInstance][handle->channel].currentBDAddr = (uint32_t)(handle->BDPool);
#endif
}
#endif /* FSL_SDK_DISABLE_IRQ */

void SDMA_SetCallback(sdma_handle_t *handle, sdma_callback callback, void *userData)
{
    assert(handle != NULL);

    handle->callback = callback;
    handle->userData = userData;
}

void SDMA_SetMultiFifoConfig(sdma_transfer_config_t *config, uint32_t fifoNums, uint32_t fifoOffset, uint32_t sampleNums)
{
    assert(config != NULL);

    config->multiFifo.fifoNums = fifoNums;
    config->multiFifo.fifoOffset = fifoOffset;
    config->multiFifo.sampleNums = sampleNums;
}

void SDMA_SetMultiSaiConfig(sdma_transfer_config_t *config, uint32_t sampleNums)
{
    assert(config != NULL);

    config->multiSai.sampleNums = sampleNums;
}

/* TODO: register DONE_CONF is undocumented but seems to be used for PDM */
void SDMA_EnableSwDone(SDMAARM_Type *base, sdma_transfer_config_t *config, uint8_t sel, enum sdma_peripheral type)
{
    assert(config != NULL);

    config->swDone.swDoneSel = sel;
    config->swDone.enableSwDone = true;

    /* enable sw done function */
    if (type == kSDMA_PeripheralMultiFifoPDM)
    {
        base->DONE_CONF |= SDMAARM_DONE_CONF_DONE_CONF_MASK;
    }
}

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
                          struct sdma_fw_script_start_addrs *script_addrs)
{
    assert(config != NULL);
    assert((srcWidth == 1U) || (srcWidth == 2U) || (srcWidth == 4U));
    assert((destWidth == 1U) || (destWidth == 2U) || (destWidth == 4U));

#if (defined(FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET) && FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET)
    config->srcAddr = MEMORY_ConvertMemoryMapAddress(srcAddr, kMEMORY_Local2DMA);
    config->destAddr = MEMORY_ConvertMemoryMapAddress(destAddr, kMEMORY_Local2DMA);
#else
    config->srcAddr = srcAddr;
    config->destAddr = destAddr;
#endif
    config->bytesPerRequest = bytesEachRequest;
    config->transferSzie = transferSize;
    config->period_len = period_len;
    config->type = type;
    switch (srcWidth)
    {
        case 1U:
            config->srcTransferSize = kSDMA_TransferSize1Bytes;
            break;
        case 2U:
            config->srcTransferSize = kSDMA_TransferSize2Bytes;
            break;
        case 4U:
            config->srcTransferSize = kSDMA_TransferSize4Bytes;
            break;
        default:
            break;
    }
    switch (destWidth)
    {
        case 1U:
            config->destTransferSize = kSDMA_TransferSize1Bytes;
            break;
        case 2U:
            config->destTransferSize = kSDMA_TransferSize2Bytes;
            break;
        case 4U:
            config->destTransferSize = kSDMA_TransferSize4Bytes;
            break;
        default:
            break;
    }
    switch (type)
    {
        case kSDMA_MemoryToMemory:
            config->scriptAddr = FSL_FEATURE_SDMA_M2M_ADDR;
            config->isEventIgnore = true;
            config->isSoftTriggerIgnore = false;
            config->eventSource = 0;
            break;
        case kSDMA_MemoryToPeripheral:
            config->scriptAddr = SDMA_GetScriptAddr(peripheral,
                            kSDMA_MemoryToPeripheral,
                            script_addrs);
            config->isEventIgnore = false;
            config->isSoftTriggerIgnore = true;
            config->eventSource = eventSource;
            break;
        case kSDMA_PeripheralToMemory:
            config->scriptAddr = SDMA_GetScriptAddr(peripheral,
                            kSDMA_PeripheralToMemory,
                            script_addrs);
            config->isEventIgnore = false;
            config->isSoftTriggerIgnore = true;
            config->eventSource = eventSource;
            break;
        default:
            break;
    }
}

/*!
 * @brief Set SDMA Runnable Channel Selection Control.
 *
 * This function selects which trigger conditions that must occur for the channel to start.
 *
 * @param handle SDMA handle pointer.
 * @param config Pointer to SDMA transfer configuration structure.
 */
void SDMA_Config_Ownership(sdma_handle_t *handle, const sdma_transfer_config_t *config)
{
    /* DO register shall always set */
    handle->base->DSPOVR |= (1U << handle->channel);

    /* Configure EO bit */
    if (config->isEventIgnore)
    {
        handle->base->EVTOVR |= (1U << handle->channel);
    }
    else
    {
        handle->base->EVTOVR &= ~(1U << handle->channel);
    }

    /* Configure HO bits */
    if (config->isSoftTriggerIgnore)
    {
        handle->base->HOSTOVR |= (1U << handle->channel);
    }
    else
    {
        handle->base->HOSTOVR &= ~(1U << handle->channel);
    }
}

/*!
 * @brief Set SDMA Channel Enable Registers.
 *
 * This function selects which DMA request is activated for a DMA channel.
 *
 * @param handle SDMA handle pointer.
 * @param eventSource DMA request to enable for the handle's channel.
 */
void SDMA_Event_Enable(sdma_handle_t *handle, uint32_t eventSource)
{
    uint32_t val = 0U;

    /* Set event source channel */
    val = handle->base->CHNENBL[eventSource];
    val |= (1U << (handle->channel));
    SDMA_SetSourceChannel(handle->base, eventSource, val);
}

void SDMA_Event_Disable(sdma_handle_t *handle, uint32_t eventSource)
{
    uint32_t val = 0U;

    /* Clear the event map. */
    val = handle->base->CHNENBL[eventSource];
    val &= ~(1U << (handle->channel));
    SDMA_SetSourceChannel(handle->base, eventSource, val);
}


#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
void SDMA_SubmitTransfer(sdma_handle_t *handle, const sdma_transfer_config_t *config)
#else
void SDMA_SubmitTransfer(sdma_handle_t *handle, const sdma_transfer_config_t *config, struct sdma_buffer_descriptor *bd_ch, struct sdma_buffer_descriptor *bd0, paddr_t context_phys)
#endif /* FSL_SDK_DISABLE_IRQ */
{
    assert(handle != NULL);
    assert(config != NULL);

#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
    uint32_t instance = SDMA_GetInstance(handle->base);

#else
    assert(bd_ch != NULL);
    assert(bd0 != NULL);
#endif /* FSL_SDK_DISABLE_IRQ */

    handle->eventSource = config->eventSource;

    /* Set event source channel */
    if (config->type != kSDMA_MemoryToMemory)
    {
        SDMA_Event_Enable(handle, config->eventSource);
    }

    SDMA_Config_Ownership(handle, config);

    /* If use default buffer descriptor, configure the buffer descriptor */
    if (handle->BDPool == NULL)
    {
        size_t transfer_size = config->transferSzie;
        uint32_t src_addr = config->srcAddr;
        uint32_t dest_addr = config->destAddr;
        bool is_last = true;
        bool is_wrap = false;
        struct sdma_buffer_descriptor *bd;
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
        bd = &s_SDMABD[instance][handle->channel];
#else
        bd = &bd_ch[0];
#endif /* FSL_SDK_DISABLE_IRQ */

        size_t len = config->transferSzie; /* remaining data to transfer */
        unsigned int loop_count = (config->transferSzie / SDMA_BD_MAX_CNT) + 1;
        unsigned int i = 0;

        if (loop_count >= sizeof(bd_ch)) {
            printf("Error: array BD for channel %d is too small.\n", handle->channel);
            return;
        }
        memset(bd, 0, loop_count * sizeof(struct sdma_buffer_descriptor));

        /* create <loop_count> number of buffer descriptors */
        for (i = 0; i < loop_count; i++) {
            transfer_size = min(len, SDMA_BD_MAX_CNT);
            is_last = (i >= (loop_count - 1));
            is_wrap = false;

            if (config->type == kSDMA_MemoryToPeripheral)
            {
                SDMA_ConfigBufferDescriptor(
                                        bd,
                                        src_addr /*config->srcAddr*/,
                                        dest_addr /*config->destAddr*/,
                                        config->destTransferSize,
                                        transfer_size /*config->transferSzie*/,
                                        is_last, /* isLast */
                                        true, /* enableInt */
                                        is_wrap, /* isWrap */
                                        config->type);
            }
            else
            {
                SDMA_ConfigBufferDescriptor(
                                        bd,
                                        src_addr /*config->srcAddr*/,
                                        dest_addr /*config->destAddr*/,
                                        config->srcTransferSize,
                                        transfer_size /*config->transferSzie*/,
                                        is_last, /* isLast */
                                        true, /* enableInt */
                                        is_wrap, /* isWrap */
                                        config->type);
            }
            len -= transfer_size;
            src_addr += transfer_size;
            dest_addr += transfer_size;
            bd++;
        } /* end for loop_count */
    }

    /* Load the context */
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
    SDMA_LoadContext(handle, config);
#else
    SDMA_LoadContext(handle, config, bd0, context_phys);
#endif /* FSL_SDK_DISABLE_IRQ */
}

/*!
 * @brief Setup the buffer descriptors for a memory-to-memory transfer.
 *
 * This function select creates buffer descriptors for a memcpy command.
 *
 * @param handle SDMA handle pointer.
 * @param config Pointer to SDMA transfer configuration structure.
 * @param bd_ch array of Buffer Descriptors.
 */
void SDMA_PrepMemcpy(sdma_handle_t *handle, const sdma_transfer_config_t *config, struct sdma_buffer_descriptor *bd_ch)
{
    size_t len = config->transferSzie; /* remaining data to transfer */
    size_t count;
    int channel = handle->channel;
    unsigned int i = 0;
    unsigned int num_bds = 0;
    int param;
    uint32_t dma_src = config->srcAddr;
    uint32_t dma_dst = config->destAddr;
    struct sdma_buffer_descriptor *bd;

    printlk(LK_NOTICE, "%s:%d: memcpy: %pad->%pad, len=%zu, channel=%d.\n",
            __PRETTY_FUNCTION__, __LINE__, &dma_src, &dma_dst, len, channel);

    num_bds = (len / SDMA_BD_MAX_CNT) + 1;
    /* transfer init */
    handle->bdCount = num_bds;
    handle->flags = 0;
    handle->chn_real_count = 0;

    /* buffer descriptors are already allocated */
    if (num_bds > sizeof(bd_ch)) {
        printf("Error: array BD for channel %d is too small.\n", channel);
        return;
    }
    bd = &bd_ch[0];
    memset(bd, 0, num_bds * sizeof(struct sdma_buffer_descriptor));

    /* configure buffer descriptors */
    do {
        count = min(len, SDMA_BD_MAX_CNT);
        bd = &bd_ch[i];
        bd->bufferAddr = dma_src;
        bd->extendBufferAddr = dma_dst;
        bd->count = count;
        bd->command = kSDMA_TransferSize4Bytes;
        //sdmac->chn_count += count;

        //if (check_bd_buswidth(bd, sdmac, count, dma_dst, dma_src))
        //    goto err_bd_out;

        dma_src += count;
        dma_dst += count;
        len -= count;
        i++;

        param = kSDMA_BDStatusDone | kSDMA_BDStatusExtend | kSDMA_BDStatusContinuous;
        /* last bd */
        if (!len) {
            param |= kSDMA_BDStatusInterrupt;
            param |= kSDMA_BDStatusLast;
            param &= ~kSDMA_BDStatusContinuous;
        }

        printlk(LK_NOTICE, "%s:%d: entry %d: command: %d count: %zd dma: 0x%x %s%s\n",
                __PRETTY_FUNCTION__, __LINE__,
                i, bd->command, count, bd->bufferAddr,
                param & kSDMA_BDStatusWrap ? "wrap" : "",
                param & kSDMA_BDStatusInterrupt ? " intr" : "");

        bd->status = param;
    } while (len);
}

/*!
 * @brief Setup the buffer descriptors for a peripheral to/from memory transfer.
 *
 * This function select creates buffer descriptors for a peripheral to/from memory command.
 *
 * @param handle SDMA handle pointer.
 * @param config Pointer to SDMA transfer configuration structure.
 * @param bd_ch array of Buffer Descriptors.
 */
void SDMA_PrepDmaCyclic(sdma_handle_t *handle, const sdma_transfer_config_t *config, struct sdma_buffer_descriptor *bd_ch)
{
    int channel = handle->channel;
    unsigned int i = 0;
    size_t buf = 0;
    unsigned int num_periods = 0;
    int param;
    struct sdma_buffer_descriptor *bd;
    uint32_t dma_addr;
    size_t word_size;
    size_t period_len;
    uint32_t buf_len;

    printlk(LK_INFO, "%s channel: %d\n", __func__, channel);

    period_len = config->period_len;
    assert(period_len != 0);
    buf_len = config->transferSzie;
    num_periods = buf_len / period_len;
    /* transfer init */
    handle->bdCount = num_periods;
    handle->bdIndex = 0;
    handle->period_len = config->period_len;
    handle->flags |= IMX_DMA_SG_LOOP;
    handle->chn_real_count = 0;

    /* buffer descriptors are already allocated */
    if (num_periods > sizeof(bd_ch)) {
        printf("Error: array BD for channel %d is too small.\n", handle->channel);
        return;
    }
    if (period_len > SDMA_BD_MAX_CNT) {
        printf("Error: SDMA channel %d: maximum period size exceeded: %zu > %d\n",
            channel, period_len, SDMA_BD_MAX_CNT);
        return;
    }

    bd = &bd_ch[0];
    memset(bd, 0, num_periods * sizeof(struct sdma_buffer_descriptor));

    if (config->type == kSDMA_PeripheralToMemory) {
        dma_addr  = config->destAddr;
        word_size = config->srcTransferSize;
    } else {
        dma_addr  = config->srcAddr;
        word_size = config->destTransferSize;
    }
    ASSERT(dma_addr != 0);

    /* configure buffer descriptors */
    while (buf < buf_len) {
        bd = &bd_ch[i];
        bd->bufferAddr = dma_addr;
        bd->count = period_len;
        bd->command = word_size;

        param = kSDMA_BDStatusDone | kSDMA_BDStatusExtend | kSDMA_BDStatusContinuous | kSDMA_BDStatusInterrupt;

        if ((i + 1) == num_periods)
            param |= kSDMA_BDStatusWrap;

        printlk(LK_INFO, "entry %d: command: %d count: %zu dma: 0x%x %s%s\n",
                i, bd->command, period_len, bd->bufferAddr,
                param & kSDMA_BDStatusWrap ? "wrap" : "",
                param & kSDMA_BDStatusInterrupt ? " intr" : "");

        bd->status = param;

        dma_addr += period_len;
        buf += period_len;

        i++;
    }
}

void SDMA_EnableChannel(sdma_handle_t *handle)
{
    assert(handle != NULL);

    /* Set the channel priority */
    if (handle->priority == 0)
    {
        handle->priority = handle->base->SDMA_CHNPRI[handle->channel];
    }

    /* Set priority if regsiter bit is 0*/
    if (handle->base->SDMA_CHNPRI[handle->channel] == 0)
    {
        SDMA_SetChannelPriority(handle->base, handle->channel, handle->priority);
    }

    if (handle->eventSource != 0)
    {
        SDMA_StartChannelEvents(handle->base, handle->channel);
    }
    else
    {
        SDMA_StartChannelSoftware(handle->base, handle->channel);
    }
}


void SDMA_DisableChannel(sdma_handle_t *handle)
{
    assert(handle != NULL);

    /* Stop channel */
    SDMA_StopChannel(handle->base, handle->channel);
    /* Clear the event map */
    SDMA_Event_Disable(handle, handle->eventSource);
    /* Clear the channel priority */
    SDMA_SetChannelPriority(handle->base, handle->channel, kDMA_ChannelPriority0);
}


#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
uint32_t SDMA_GetTransferredBytes(sdma_handle_t *handle)
#else
uint32_t SDMA_GetTransferredBytes(sdma_handle_t *handle, struct sdma_buffer_descriptor *sdma_ch_bd)
#endif /* FSL_SDK_DISABLE_IRQ */
{
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
    uint32_t instance = SDMA_GetInstance(handle->base);
#endif /* FSL_SDK_DISABLE_IRQ */
    uint32_t val = 0;

    if (handle->BDPool == NULL)
    {
#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
        val = s_SDMABD[instance][handle->channel].count - 1U;
#else
#if 0
        struct sdma_buffer_descriptor *bd = sdma_ch_bd;
        bool is_last;
        /* iterate thru the array of buffer descriptors */
        is_last = false;
        do {
            val += (bd->count);
            is_last = (bd->status & kSDMA_BDStatusLast);
            bd++;
        } while (!is_last);
#endif /* 0 */
        val = handle->chn_real_count;
#endif /* FSL_SDK_DISABLE_IRQ */
    }
    else
    {
        val = 0;
    }

    return val;
}

#if !(defined(FSL_SDK_DISABLE_IRQ) && FSL_SDK_DISABLE_IRQ)
void SDMA_HandleIRQ(sdma_handle_t *handle)
{
    assert(handle != NULL);
    uint32_t status = 0;

    /* Clear the status for all channels */
    status = (SDMA_GetChannelInterruptStatus(handle->base) & 0xFFFFFFFEU);
    SDMA_ClearChannelInterruptStatus(handle->base, status);

    /* Set the current BD address to the CCB */
    if (handle->BDPool)
    {
        /* Set the DONE bits */
        handle->bdIndex = (handle->bdIndex + 1U) % handle->bdCount;
/* TODO: why s_SDMACCB[0]? most likely a bug */
        s_SDMACCB[0][handle->channel].currentBDAddr = (uint32_t)(&handle->BDPool[handle->bdIndex]);
    }
    else
    {
        s_SDMACCB[0][handle->channel].currentBDAddr = s_SDMACCB[0][handle->channel].baseBDAddr;
    }

    if (handle->callback != NULL)
    {
        (handle->callback)(handle, handle->userData, true, handle->bdIndex);
    }
}

#if defined(SDMAARM)
void SDMA_DriverIRQHandler(void)
{
    uint32_t i = 1U, val;

    /* Clear channel 0 */
    SDMA_ClearChannelInterruptStatus(SDMAARM, 1U);
    /* Ignore channel0, as channel0 is only used for download */
    val = (SDMAARM->INTR) >> 1U;
    while (val)
    {
        if (val & 0x1U)
        {
            SDMA_HandleIRQ(s_SDMAHandle[0][i]);
        }
        i++;
        val >>= 1U;
    }
}
#endif
#if defined(SDMAARM1)
void SDMA1_DriverIRQHandler(void)
{
    uint32_t i = 1U, val;

    /* Clear channel 0 */
    SDMA_ClearChannelInterruptStatus(SDMAARM1, 1U);
    /* Ignore channel0, as channel0 is only used for download */
    val = (SDMAARM1->INTR) >> 1U;
    while (val)
    {
        if (val & 0x1U)
        {
            SDMA_HandleIRQ(s_SDMAHandle[0][i]);
        }
        i++;
        val >>= 1U;
    }
}
#endif
#if defined(SDMAARM2)
void SDMA2_DriverIRQHandler(void)
{
    uint32_t i = 1U, val;

    /* Clear channel 0 */
    SDMA_ClearChannelInterruptStatus(SDMAARM2, 1U);
    /* Ignore channel0, as channel0 is only used for download */
    val = (SDMAARM2->INTR) >> 1U;
    while (val)
    {
        if (val & 0x1U)
        {
            SDMA_HandleIRQ(s_SDMAHandle[1][i]);
        }
        i++;
        val >>= 1U;
    }
}
#endif

#if defined(SDMAARM3)
void SDMA3_DriverIRQHandler(void)
{
    uint32_t i = 1U, val;

    /* Clear channel 0 */
    SDMA_ClearChannelInterruptStatus(SDMAARM3, 1U);
    /* Ignore channel0, as channel0 is only used for download */
    val = (SDMAARM3->INTR) >> 1U;
    while (val)
    {
        if (val & 0x1U)
        {
            SDMA_HandleIRQ(s_SDMAHandle[2][i]);
        }
        i++;
        val >>= 1U;
    }
}
#endif
#endif /* FSL_SDK_DISABLE_IRQ */
