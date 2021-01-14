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
#include <sys/types.h>
#include <stdlib.h>
#include <lib/appargs.h>
#include "platform/interrupts.h"
#include <dev/class/sdma.h>

#include <kernel/spinlock.h>
#include <kernel/semaphore.h>
#include <kernel/event.h>
#include <kernel/vm.h>
#include "lib/dpc.h"

#include <assert.h>
#include <err.h>
#include <debug.h>

#include <dev/dma.h>

#include "sdma_hw.h"
#include "platform.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

//#define SDMA_DBG_TIME_MONITOR

#define NUM_BD 8 /*< number of buffer descriptors per channel */
#define SDMA_RAM_SIZE 8192

#define SDMA_BD_IS_OWNED_BY_CPU(x) ( 0 == (x->status & kSDMA_BDStatusDone) )
#define SDMA_BD_HAS_ERRORS(x)      ( x->status & kSDMA_BDStatusError )

struct imx_sdma_state {
    int bus_id;
    SDMAARM_Type *io_base;
    spin_lock_t lock;
    mutex_t mutex;
    event_t wait;
    unsigned irq;
    struct device *device;
    struct list_node node;
    struct sdma_handle sdma_handle[FSL_FEATURE_SDMA_MODULE_CHANNEL];
    struct dma_chan channels[FSL_FEATURE_SDMA_MODULE_CHANNEL];
    /* base address of the channel 0 control block (CCB0) */
    struct sdma_channel_control *sdma_ccb;
    paddr_t sdma_ccb_phys;
    /* base address of the channel 0 buffer descriptor */
    struct sdma_buffer_descriptor *sdma_bd0;
    paddr_t sdma_bd0_phys;
    /* base address of the channel [i] buffer descriptor */
    struct sdma_buffer_descriptor *sdma_bd[FSL_FEATURE_SDMA_MODULE_CHANNEL];
    paddr_t sdma_bd_phys[FSL_FEATURE_SDMA_MODULE_CHANNEL];
    /* context area */
    sdma_context_data_t *sdma_context;
    paddr_t sdma_context_phys;
    /* clock ratio for AHB:SDMA core. 1:1 is 1, 2:1 is 0 */
    bool clk_ratio;
    /* Firmware */
    struct sdma_fw_script_start_addrs *script_addrs;
    bool fw_loaded;
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
#ifdef IMX_SDMA_USE_OCRAM
static uint8_t sdma_idx_lut[IMX_SDMA_NUM];
static uint8_t cur_idx = 0;
static __SECTION(".ocram.data") sdma_channel_control_t sdmaccb[IMX_SDMA_NUM][FSL_FEATURE_SDMA_MODULE_CHANNEL];
static __SECTION(".ocram.data") struct {
        sdma_buffer_descriptor_t bd[NUM_BD];
    } __attribute__((aligned(128))) sdmabd[IMX_SDMA_NUM][FSL_FEATURE_SDMA_MODULE_CHANNEL];
static __SECTION(".ocram.data") sdma_context_data_t sdmacontext[IMX_SDMA_NUM];

static int sdma_get_index_from_bus_id(int bus_id)
{
    int i;

    for (i = 0; i < IMX_SDMA_NUM; i++) {
        if (sdma_idx_lut[i] == bus_id)
            return i;
    }

    return -1;
}
#endif

static struct list_node imx_sdma_list = LIST_INITIAL_VALUE(imx_sdma_list);
static spin_lock_t imx_sdma_list_lock = SPIN_LOCK_INITIAL_VALUE;


static inline struct imx_sdma_state * imx_sdma_dev_to_state(struct device *dev)
{
    return dev->state;
}

/* SDMA ROM table for script addresses */
static struct sdma_fw_script_start_addrs sdma_fw_rom_scripts_imx8m = {
    .ap_2_ap_addr      = FSL_FEATURE_SDMA_M2M_ADDR,
    .uart_2_mcu_addr   = FSL_FEATURE_SDMA_UART2M_ADDR,
    .mcu_2_app_addr    = FSL_FEATURE_SDMA_M2P_ADDR,
    .uartsh_2_mcu_addr = FSL_FEATURE_SDMA_UARTSH2M_ADDR,
    .mcu_2_shp_addr    = FSL_FEATURE_SDMA_M2SHP_ADDR,
    .app_2_mcu_addr    = FSL_FEATURE_SDMA_P2M_ADDR,
    .shp_2_mcu_addr    = FSL_FEATURE_SDMA_SHP2M_ADDR,
    .spdif_2_mcu_addr  = FSL_FEATURE_SDMA_SPDIF2M_ADDR,
    .mcu_2_spdif_addr  = FSL_FEATURE_SDMA_M2SPDIF_ADDR,
};

/*******************************************************************************
 * Code
 ******************************************************************************/
/** SDMA interrupt subroutine for cyclic transfers.
 */

#ifdef SDMA_DBG_TIME_MONITOR

#define SDMA_DBG_CHANS     32
#define SDMA_DBG_INSTANCES 3
#define SDMA_DBG_DISPLAY_PERIOD 4096
#define SDMA_DBG_LONG_TIME 0xFFFFFFFF

struct sdma_dbg_chan {
    uint64_t time_prev;
    uint64_t span_max;
    uint64_t span_min;
};

static struct sdma_dbg_info {
    struct sdma_dbg_chan chans[SDMA_DBG_INSTANCES][SDMA_DBG_CHANS];
    uint64_t duration_min;
    uint64_t duration_max;
    uint64_t time;
    uint32_t period;
} s_sdma_dbg_info;


static void sdma_dbg_entry(void)
{
    struct sdma_dbg_info *dbg = &s_sdma_dbg_info;
    dbg->time = current_time_hires();
}


static void sdma_dbg_chan_complete(size_t dma_id, size_t chan_id)
{
    struct sdma_dbg_info *dbg = &s_sdma_dbg_info;
    struct sdma_dbg_chan *chan;
    uint64_t span;

    if (dma_id >= SDMA_DBG_INSTANCES)
        return;
    if (chan_id > SDMA_DBG_CHANS)
        return;

    chan = &dbg->chans[dma_id][chan_id];
    if (chan->time_prev) {
        span = (dbg->time - chan->time_prev);
        if (span > chan->span_max)
            chan->span_max = span;
        if (span < chan->span_min)
            chan->span_min = span;
    }
    chan->time_prev = dbg->time;
}

static void sdma_dbg_reset(void)
{
    struct sdma_dbg_info *dbg = &s_sdma_dbg_info;
    struct sdma_dbg_chan chan_reset =
                { .time_prev = 0, .span_max = 0, .span_min = SDMA_DBG_LONG_TIME };
    size_t i, j;

    for (i = 0; i < SDMA_DBG_INSTANCES; i++) {
        for (j = 0; j < SDMA_DBG_CHANS; j++) {
            dbg->chans[i][j] = chan_reset; //struct copy
        }
    }

    dbg->duration_max = 0;
    dbg->duration_min = SDMA_DBG_LONG_TIME;
    dbg->time = 0;
}

static void sdma_dbg_exit(void)
{
    struct sdma_dbg_info *dbg = &s_sdma_dbg_info;
    uint64_t duration;
    uint32_t active = 0;
    size_t i, j;

    duration = (current_time_hires() - dbg->time);
    if (duration > dbg->duration_max)
        dbg->duration_max = duration;
    if (duration < dbg->duration_min)
        dbg->duration_min = duration;

    dbg->period = (dbg->period + 1) % SDMA_DBG_DISPLAY_PERIOD;
    if (dbg->period != (SDMA_DBG_DISPLAY_PERIOD - 1))
        return;

    printlk(LK_NOTICE, "%s:%d: smda isr (%lld/%lld)\n",
            __PRETTY_FUNCTION__, __LINE__, dbg->duration_min, dbg->duration_max);

    for (i = 0; i < SDMA_DBG_INSTANCES; i++) {
        for (j = 0; j < SDMA_DBG_CHANS; j++) {
            if (dbg->chans[i][j].span_min == SDMA_DBG_LONG_TIME)
                dbg->chans[i][j].span_min = 0;
            if (dbg->chans[i][j].span_max != 0)
                active |= (1 << i);
        }
    }

    for (i = 0; i < SDMA_DBG_INSTANCES; i++) {
        struct sdma_dbg_chan *dma_chans;
        dma_chans = &dbg->chans[i][0];
        if (!(active & (1 << i)))
            continue;
        printlk(LK_NOTICE, "%s:%d: SDMA%d "
               "#0(%lld/%lld) #1(%lld/%lld) #2(%lld/%lld) #3(%lld/%lld) #4(%lld/%lld) #5(%lld/%lld) #6(%lld/%lld) #7(%lld/%lld) "
               "#8(%lld/%lld) #9(%lld/%lld) #10(%lld/%lld) #11(%lld/%lld) #12(%lld/%lld) #13(%lld/%lld) #14(%lld/%lld) #15(%lld/%lld)\n",
               __PRETTY_FUNCTION__, __LINE__,
               (int)(i + 1),
               dma_chans[0].span_min, dma_chans[0].span_max, dma_chans[1].span_min, dma_chans[1].span_max,
               dma_chans[2].span_min, dma_chans[2].span_max, dma_chans[3].span_min, dma_chans[3].span_max,
               dma_chans[4].span_min, dma_chans[4].span_max, dma_chans[5].span_min, dma_chans[5].span_max,
               dma_chans[6].span_min, dma_chans[6].span_max, dma_chans[7].span_min, dma_chans[7].span_max,
               dma_chans[8].span_min, dma_chans[8].span_max, dma_chans[9].span_min, dma_chans[9].span_max,
               dma_chans[10].span_min, dma_chans[10].span_max, dma_chans[11].span_min, dma_chans[11].span_max,
               dma_chans[12].span_min, dma_chans[12].span_max, dma_chans[13].span_min, dma_chans[13].span_max,
               dma_chans[14].span_min, dma_chans[14].span_max, dma_chans[15].span_min, dma_chans[15].span_max
               );
    }

    sdma_dbg_reset();
}

#else

static void sdma_dbg_entry(void) {}
static void sdma_dbg_chan_complete(size_t dma_id, size_t chan_id) {}
static void sdma_dbg_reset(void) {}
static void sdma_dbg_exit(void) {}

#endif


static void sdma_update_channel_loop(struct sdma_handle *handle, spin_lock_t *lock)
{
    struct sdma_buffer_descriptor *bd;
    status_t error = NO_ERROR;
    uint8_t old_status = handle->status;

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);
    /*
     * loop mode. Iterate over descriptors, re-setup them and
     * call callback function.
     */
    while (handle) {
        printlk(LK_INFO, "%s:%d: bdIndex=%d\n", __PRETTY_FUNCTION__, __LINE__,
                handle->bdIndex);
        bd = &handle->bd[handle->bdIndex];

        if (bd->status & kSDMA_BDStatusDone)
            break;

        if (bd->status & kSDMA_BDStatusError) {
            bd->status &= ~kSDMA_BDStatusError;
            // FIXME handle->status = DMA_ERROR;
            error = ERR_IO;
        }

        /*
         * We use bd->mode.count to calculate the residue, since contains
         * the number of bytes present in the current buffer descriptor.
         */
        handle->chn_real_count = bd->count;
        bd->status |= kSDMA_BDStatusDone;
        bd->count = handle->period_len;
        //desc->buf_ptail = desc->buf_tail;
        handle->bdIndex = (handle->bdIndex + 1) % handle->bdCount;

        /* FIXME: did not understand this line */
        if (error)
            handle->status = old_status;
    }
    printlk(LK_VERBOSE, "%s: exit\n", __PRETTY_FUNCTION__);
}

/** SDMA interrupt handler
 *  Code from SDMA3_DriverIRQHandler() and SDMA_HandleIRQ()
 */
static enum handler_return imx_sdma_isr(void *arg)
{
    struct device *dev = (struct device *)arg;
    ASSERT(dev);
    struct imx_sdma_state *state = dev->state;
    ASSERT(state);
    bool reschedule = false;
    uint32_t status = 0;

    SDMAARM_Type *base = state->io_base;

    ASSERT(base);
    uint32_t i = 1U;

    sdma_dbg_entry();
#ifdef SDMA_ENABLE_EVT_ERROR
    uint32_t evterr = SDMA_GetErrorStatus(base);
    if (evterr)
        printlk(LK_WARNING, "SDMA %d: error %x\n", state->bus_id, evterr);
#endif
    /* Clear the status for all channels */
    status = SDMA_GetChannelInterruptStatus(base);
    SDMA_ClearChannelInterruptStatus(base, status);

    /* Ignore channel0, as channel0 is only used for download */
    status >>= 1U;
    i = 1U; /* start with channel #1 */

    while (status) {
        if (status & 0x1U) {
            spin_lock(&state->lock);

            sdma_handle_t *handle = &state->sdma_handle[i];
            assert(handle != NULL);
            struct dma_chan *chan = &state->channels[i];
            ASSERT(chan);
            struct dma_descriptor *desc = chan->desc;
//            ASSERT(desc);
            struct sdma_buffer_descriptor *bd = &handle->bd[handle->bdIndex];
            uint32_t n_bd_handled_by_cpu = 0;


            unsigned loop = 0;
                /* Prevent further processing on already-released channels */
            while (!(chan->chan_available) && desc) {
                printlk(LK_VERBOSE, "SDMA (%d) channel (%d) bdIndex (%d), status (%x)\n",
                            state->bus_id, i, handle->bdIndex, bd->status);

                desc->status = DMA_XFER_RUNNING;

                /* Check if buffer descriptor is owned by SDMA IP */
                if (!(SDMA_BD_IS_OWNED_BY_CPU(bd)))
                    break;

                loop++;

                desc->status = DMA_XFER_COMPLETE;

                if ( SDMA_BD_HAS_ERRORS(bd) ) {
                    printlk(LK_DEBUG,
                            "Buffer descriptor error (sdma=%d, channel=%d, index=%d)\n\n",
                        state->bus_id, i, handle->bdIndex);
                    bd->status &= ~kSDMA_BDStatusError;
                    desc->status = DMA_XFER_ERROR;
                }

                if (desc->status != DMA_XFER_ERROR) {
                    desc->bytes_transferred += handle->period_len;
                    desc->period_elapsed++;
                    sdma_dbg_chan_complete(state->bus_id - 1, i);
                } else {
                    printlk(LK_WARNING, "SDMA%d ch%d error\n", state->bus_id, i);
                }

                /*
                 * We use bd->mode.count to calculate the residue, since contains
                 * the number of bytes present in the current buffer descriptor.
                 */
                handle->chn_real_count = bd->count;

                bd->count = handle->period_len;
                handle->bdIndex = (handle->bdIndex + 1) % handle->bdCount;

                /* Check buffer descriptors 'loopback' */
                if (++n_bd_handled_by_cpu == handle->bdCount) {
                    printlk(LK_WARNING,
                        "SDMA %d ahead of CPU by a whole bd chain"
                        " - starvation suspected\n", state->bus_id);
                    /* TODO - take appropriate action */
                }

                /* Notify client (if registered) */
                bool release_descriptor = true;
                if (desc->cb) {
                    reschedule = true;
                    if (desc->cb(desc) == false) {
                        release_descriptor = false;
                    }
                }

                /* Notify zero-copy callback to update destination/size (if needed) */
                if (desc->dma_mode && desc->zerocopy_cb) {
                    uint32_t len = bd->count;
                    if (desc->zerocopy_cb( desc, &bd->bufferAddr, &len ) == false) {
                        release_descriptor = false;
                    }
                    /* Current SDMA supports up to 16-bits count values only */
                    ASSERT(len < (1 << 16));
                    bd->count = len;
                }

                /* Give back buffer descriptor to SDMA */
                smp_wmb();
                if (release_descriptor) {
                    bd->status |= kSDMA_BDStatusDone;
                } else {
                    /* mark descriptor as error to prevent further re-processing */
                    bd->status |= kSDMA_BDStatusError;
                    break;
                }

                /* Move to next buffer descriptor */
                bd = &handle->bd[handle->bdIndex];
            }

            if (loop > 1)
                printlk(LK_DEBUG, "SDMA%d ch%d, %d descriptor processed\n",
                    state->bus_id, i, loop);

            spin_unlock(&state->lock);
            /* ... otherwise, move to next channel */
        }
        i++;
        status >>= 1U;
    }

    sdma_dbg_exit();

    if (reschedule) {
        return INT_RESCHEDULE;
    } else {
        return INT_NO_RESCHEDULE;
    }
}

static struct sdma_handle *to_sdma_handle(struct dma_chan *chan)
{
    return chan->private;
}

/** Update FW script table
 *  Copies lookup table from <src> to <dest>
 * @param dest SDMA script lookup table to update
 * @param src lookup table to copy to <dest>
 * @param num_scripts number of script addresses to copy
 */
static void sdma_add_scripts(
    struct sdma_fw_script_start_addrs *dest,
    const struct sdma_fw_script_start_addrs *src,
    unsigned int num_scripts)
{
    int32_t *addr_arr = (int32_t *)src;
    int32_t *saddr_arr = (int32_t *)dest;
    unsigned int i;

    ASSERT(dest != NULL);
    ASSERT(src != NULL);
    ASSERT(num_scripts <=
        (sizeof(struct sdma_fw_script_start_addrs) / sizeof(uint32_t)));

    for (i = 0; i < num_scripts; i++)
        if (addr_arr[i] > 0)
            saddr_arr[i] = addr_arr[i];
}

/** Load a firmware to SDMA RAM.
 * @param base SDMA base address.
 * @param fw_addr firmware location.
 * @param buffer descriptor to use when loading the SDMA firmware.
 * @param sdma_script_addrs SDMA lookup table to update.
 */
static status_t sdma_load_firmware(struct imx_sdma_state *state,
    struct sdma_buffer_descriptor *sdma_bd0)
{
    SDMAARM_Type *base = state->io_base;
    struct sdma_firmware_header *header;
    status_t ret = NO_ERROR;

    header = malloc(sizeof(struct sdma_firmware_header));
    if (!header)
        return ERR_NO_MEMORY;

    unsigned int count = of_device_get_int32_array(state->device, "firmware",
       (uint32_t *)header, sizeof(struct sdma_firmware_header) / sizeof(uint32_t));

    if (count != (sizeof(struct sdma_firmware_header) / sizeof(uint32_t))) {
        printlk(LK_ERR, "%s:%d: error while getting header: %d.\n",
                __PRETTY_FUNCTION__, __LINE__, count);
        ret = count;
        goto free_header;
    }

    if (header->magic != SDMA_FIRMWARE_MAGIC) {
        ret = ERR_NOT_VALID;
        goto free_header;
    }

    printlk(LK_NOTICE, "fw version %d.%d\n",
            header->version_major, header->version_minor);

    unsigned int num_scripts = header->num_script_addrs;
    printlk(LK_DEBUG, "Num scripts %d\n", num_scripts);

    unsigned int num_scripts_max =
        (sizeof(struct sdma_fw_script_start_addrs) / sizeof(uint32_t));
    if (num_scripts > num_scripts_max) {
        printlk(LK_WARNING, "warning: firmware contains more scripts than expected."
           " Dropping last %d scripts.\n",
           (num_scripts - num_scripts_max));
        num_scripts = num_scripts_max;
    }

    uint32_t fw_size, *fw_addr;

    fw_size = sizeof(struct sdma_firmware_header) +
        num_scripts * sizeof(uint32_t) +
        header->ram_code_size;
    printlk(LK_DEBUG, "fw size %u\n", fw_size);
    fw_size = ROUNDUP(fw_size, sizeof(uint32_t));

    fw_addr = malloc(fw_size);
    if (!fw_addr) {
        ret = ERR_NO_MEMORY;
        goto free_header;
    }

    count = of_device_get_int32_array(state->device, "firmware",
       fw_addr, fw_size / sizeof(uint32_t));

    if (count != (fw_size / sizeof(uint32_t))) {
        printlk(LK_ERR, "%s:%d: error while getting fw: %d.\n",
                __PRETTY_FUNCTION__, __LINE__, count);
        ret = count;
        goto free_fw;
    }

    uint16_t *ram_code = (void *)fw_addr + header->ram_code_start;

    /* flush buffer for the firmware */
    paddr_t sdma_fw_src_paddr;
    arch_clean_invalidate_cache_range((vaddr_t)ram_code, header->ram_code_size);
    sdma_fw_src_paddr = phys_to_dma(vaddr_to_paddr((void *)ram_code));

    const struct sdma_fw_script_start_addrs *fw_scripts =
        (void *)fw_addr + header->script_addrs_start;

    SDMA_LoadScript(base,
        fw_scripts->ram_code_start_addr, /* dest addr */
        (void *)sdma_fw_src_paddr, /* srcAddr */
        header->ram_code_size / 2, /* bufferSizeWords */
        sdma_bd0 /* buffer descriptor */
    );


    sdma_add_scripts(state->script_addrs, fw_scripts, num_scripts);

free_fw:
    free(fw_addr);
free_header:
    free(header);

    return ret;
}

/** Initialize SDMA driver and select clock source
 *  This function wraps: SDMA_GetDefaultConfig(), SDMA_Init()
 */
static status_t imx_sdma_init(struct device *dev)
{
    const struct device_config_data *imx_config = dev->config;
    struct imx_sdma_state *state;
    status_t ret = NO_ERROR;

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    /* sanity checks */
    ASSERT(imx_config);

    state = malloc(sizeof(struct imx_sdma_state));
    ASSERT(state);
    memset(state, 0, sizeof(*state));
    dev->state = state;
    state->device = dev;

    state->bus_id = imx_config->bus_id;
    printlk(LK_INFO, "%s:%d: bus_id=%d\n.", __PRETTY_FUNCTION__, __LINE__,
            state->bus_id);

    struct device_cfg_reg *reg =
        device_config_get_register_by_name(imx_config, "core");
    ASSERT(reg);
    state->io_base = (SDMAARM_Type *) reg->vbase;

    struct device_cfg_irq *irq =
        device_config_get_irq_by_name(imx_config, "core");
    ASSERT(irq);

    bool ratio = false;
    ratio = of_device_get_bool(dev, "ratio-1-1");
    printlk(LK_INFO, "%s:%d: ratio-1-1 is %d\n", __PRETTY_FUNCTION__, __LINE__,
            ratio);
    state->clk_ratio = ratio;


    /* allocate an array of Channel Control Blocks (uncached contiguous memory block) */
    void *sdma_ccb_vaddr;
    paddr_t sdma_ccb_paddr;
#ifdef IMX_SDMA_USE_OCRAM
    int sdma_idx = cur_idx++;
    DEBUG_ASSERT(sdma_idx < IMX_SDMA_NUM);
    sdma_idx_lut[sdma_idx] = state->bus_id;

    printlk(LK_NOTICE, "%s:%d: SDMA%d uses on-chip RAM for descriptors - offset %d\n",
       __PRETTY_FUNCTION__, __LINE__, state->bus_id, sdma_idx);
    sdma_ccb_vaddr = &sdmaccb[sdma_idx];
    sdma_ccb_paddr = vaddr_to_paddr((void *)&sdmaccb[sdma_idx]);
#else
    ret = vmm_alloc_contiguous(vmm_get_kernel_aspace(), "sdmaccb",
            sizeof(sdma_channel_control_t) * FSL_FEATURE_SDMA_MODULE_CHANNEL,
            &sdma_ccb_vaddr, 0 /* log2_align */, 0 /* vmm flags */,
            ARCH_MMU_FLAG_UNCACHED);
    printlk(LK_INFO, "%s:%d: sdmaccb: vmm_alloc_contig returns %d, ptr %p\n",
            __PRETTY_FUNCTION__, __LINE__, ret, sdma_ccb_vaddr);
    if (ret < 0) {
        goto err_init_sdmaccb;
    }
    if (!sdma_ccb_vaddr) {
        printlk(LK_ERR, "%s:%d: error allocating sdma_ccb_vaddr.\n",
                __PRETTY_FUNCTION__, __LINE__);
        ret = ERR_NO_MEMORY;
        goto err_init_sdmaccb;
    }
    sdma_ccb_paddr = phys_to_dma(vaddr_to_paddr((void *)sdma_ccb_vaddr));
#endif /* IMX_SDMA_USE_OCRAM */

    state->sdma_ccb = (void *)sdma_ccb_vaddr;
    state->sdma_ccb_phys = sdma_ccb_paddr;
    /* zero-out the CCB structures array just allocated */
    memset(sdma_ccb_vaddr, 0,
        sizeof(sdma_channel_control_t) * FSL_FEATURE_SDMA_MODULE_CHANNEL);
    printlk(LK_NOTICE, "%s:%d: sdma ccb: va: %p pa: %lx \n",
            __PRETTY_FUNCTION__, __LINE__, state->sdma_ccb, state->sdma_ccb_phys);

    /* allocate an array of Buffer Descriptors for channel 0 (boot channel) */
    void *sdma_bd0_vaddr;
    paddr_t sdma_bd0_paddr;
#ifdef IMX_SDMA_USE_OCRAM
    sdma_bd0_vaddr = &sdmabd[sdma_idx][0];
    sdma_bd0_paddr = vaddr_to_paddr((void *)&sdmabd[sdma_idx][0]);
    unsigned z;
    for (z = 0; z < FSL_FEATURE_SDMA_MODULE_CHANNEL; z++) {
        printlk(LK_INFO, "%s:%d: sdma bd[%d]: va: %p pa: %lx \n",
                __PRETTY_FUNCTION__, __LINE__,  z, &sdmabd[sdma_idx][z],
                vaddr_to_paddr((void *)&sdmabd[sdma_idx][z]));
            memset(&sdmabd[sdma_idx][z], 0, sizeof(sdmabd[sdma_idx][z]));
    }
#else
    ret = vmm_alloc_contiguous(vmm_get_kernel_aspace(), "sdmabd0",
            sizeof(sdma_buffer_descriptor_t) * NUM_BD,
            &sdma_bd0_vaddr, 0, 0, ARCH_MMU_FLAG_UNCACHED);
    printlk(LK_INFO, "%s:%d: sdmabd: vmm_alloc_contig returns %d, ptr %p\n",
            __PRETTY_FUNCTION__, __LINE__, ret, sdma_bd0_vaddr);
    if (ret < 0) {
        goto err_init_sdmabd0;
    }
    if (!sdma_bd0_vaddr) {
        printlk(LK_ERR, "%s:%d: error allocating sdma_bd_vaddr.\n",
                __PRETTY_FUNCTION__, __LINE__);
        ret = ERR_NO_MEMORY;
        goto err_init_sdmabd0;
    }
    sdma_bd0_paddr = phys_to_dma(vaddr_to_paddr((void *)sdma_bd0_vaddr));
#endif /* IMX_SDMA_USE_OCRAM */

    state->sdma_bd0 = (void *)sdma_bd0_vaddr;
    state->sdma_bd0_phys = sdma_bd0_paddr;
    /* zero-out the BD array just allocated */
    memset(state->sdma_bd0, 0,
        sizeof(sdma_buffer_descriptor_t) * NUM_BD);
    printlk(LK_NOTICE, "%s:%d: sdma bd0: va: %p pa: %lx \n",
            __PRETTY_FUNCTION__, __LINE__, state->sdma_bd0, state->sdma_bd0_phys);

    /* allocate a buffer for the Context area */
    void *sdma_context_vaddr;
    paddr_t sdma_context_paddr;
#ifdef IMX_SDMA_USE_OCRAM
    sdma_context_vaddr = &sdmacontext[sdma_idx];
    sdma_context_paddr = vaddr_to_paddr((void *)&sdmacontext[sdma_idx]);
#else
    ret = vmm_alloc_contiguous(vmm_get_kernel_aspace(), "sdmacontext",
            sizeof(sdma_context_data_t),
            &sdma_context_vaddr, 0 /* log2_align */, 0 /* vmm flags */,
            ARCH_MMU_FLAG_UNCACHED);
    printlk(LK_INFO, "%s:%d: sdmacontext: vmm_alloc_contig returns %d, ptr %p\n",
            __PRETTY_FUNCTION__, __LINE__, ret, sdma_context_vaddr);
    if (ret < 0) {
        goto err_init;
    }
    if (!sdma_context_vaddr) {
        printlk(LK_ERR, "%s:%d: error allocating sdma_context_vaddr.\n",
                __PRETTY_FUNCTION__, __LINE__);
        ret = ERR_NO_MEMORY;
        goto err_init;
    }
    sdma_context_paddr = phys_to_dma(vaddr_to_paddr((void *)sdma_context_vaddr));
#endif /* IMX_SDMA_USE_OCRAM */

    state->sdma_context = (void *)sdma_context_vaddr;
    state->sdma_context_phys = sdma_context_paddr;
    /* zero-out the context structure just allocated */
    memset(sdma_context_vaddr, 0, sizeof(sdma_context_data_t));
    printlk(LK_NOTICE, "%s:%d: sdma context: va: %p pa: %lx \n",
            __PRETTY_FUNCTION__, __LINE__, state->sdma_context, state->sdma_context_phys);

    spin_lock_init(&state->lock);
    mutex_init(&state->mutex);
    event_init(&state->wait, false, 0);

    /* set CLK */
    struct device_cfg_clk *clk_audio_ipg =
        device_config_get_clk_by_name(imx_config, "audio_ipg");
    ASSERT(clk_audio_ipg);

    devcfg_set_clock(clk_audio_ipg);

    /* Configure SDMA */
    struct sdma_config config;
    //SDMA_GetDefaultConfig(&config);
    config.enableRealTimeDebugPin = false;
    config.isSoftwareResetClearLock = true;
    if (state->clk_ratio) {
        config.ratio = kSDMA_ARMClockFreq;
    } else {
        config.ratio = kSDMA_HalfARMClockFreq;
    }

    /* Initialize SDMA */
    SDMA_Init(state->io_base, &config,
        &state->sdma_ccb[0], state->sdma_ccb_phys,
        &state->sdma_bd0[0], state->sdma_bd0_phys);

    /* initialize dma channel flag */
    /* skip channel#0 (used internally for SDMA) */
    unsigned int i;
    for (i = 1; i < FSL_FEATURE_SDMA_MODULE_CHANNEL; i++) {
        state->channels[i].chan_available = true;
        state->channels[i].dma_device = dev;
    }

    /* load SDMA scripts */
    state->fw_loaded = false;
    state->script_addrs = malloc(sizeof(struct sdma_fw_script_start_addrs));
    ASSERT(state->script_addrs);

    /* initialize script table (0xffff=no script available) */
    memset(state->script_addrs, 0xff, sizeof(struct sdma_fw_script_start_addrs));

    /* initialize script table with ROM content */
    sdma_add_scripts(state->script_addrs, &sdma_fw_rom_scripts_imx8m,
        SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V48);

    /* load firmware to SDMA RAM if available */
    ret = sdma_load_firmware(state, state->sdma_bd0);
    if (ret == NO_ERROR) {
        printlk(LK_NOTICE, "%s:%d: Loaded firmware to SDMA %d RAM\n",
                __PRETTY_FUNCTION__, __LINE__, state->bus_id);
        state->fw_loaded = true;
    } else {
        printlk(LK_ERR, "%s:%d: Error: failed to load firmware to SDMA RAM.\n",
                __PRETTY_FUNCTION__, __LINE__);
        goto err_load;
    }

    register_int_handler(irq->irq, imx_sdma_isr, dev);
    state->irq = irq->irq;

    /* Enable interrupt */
    unmask_interrupt(state->irq);

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&imx_sdma_list_lock, lock_state);

    list_add_tail(&imx_sdma_list, &state->node);

    spin_unlock_irqrestore(&imx_sdma_list_lock, lock_state);

    printlk(LK_INFO, "%s:%d: dev: %p state: %p state->dev: %p dev->state: %p\n",
            __PRETTY_FUNCTION__, __LINE__, dev, state, state->device,
            dev->state);

    printlk(LK_VERBOSE, "%s: exit\n", __PRETTY_FUNCTION__);
    return NO_ERROR;

err_load:
#ifndef IMX_SDMA_USE_OCRAM
err_init:
    vmm_free_region(vmm_get_kernel_aspace(), (vaddr_t)sdma_context_vaddr);
err_init_sdmabd0:
    vmm_free_region(vmm_get_kernel_aspace(), (vaddr_t)sdma_bd0_vaddr);
err_init_sdmaccb:
    vmm_free_region(vmm_get_kernel_aspace(), (vaddr_t)sdma_ccb_vaddr);
#endif
    return ret;
}

/** Initialize one SDMA channel.
 *  This function wraps: SDMA_CreateHandle()
 * @param dev device
 * @param channel DMA channel to open/initialize
 */
static status_t imx_sdma_alloc_channel(struct device *dev, int channel)
{
    status_t ret = NO_ERROR;
    struct imx_sdma_state *state;
    state = dev->state;
    ASSERT(state);
    SDMAARM_Type *base = state->io_base;
    ASSERT(base);

    assert(channel < FSL_FEATURE_SDMA_MODULE_CHANNEL);

    struct sdma_handle *ch_handle = &state->sdma_handle[channel];
    ASSERT(ch_handle);
    /* get CCB buffer for this channel */
    struct sdma_channel_control *ch_ccb = &state->sdma_ccb[channel];
    ASSERT(ch_ccb);

    /* allocate array of Buffer Descriptors for this channel */
    void *sdma_bd_vaddr;
    paddr_t sdma_bd_paddr;

#ifdef IMX_SDMA_USE_OCRAM
    int sdma_idx = sdma_get_index_from_bus_id(state->bus_id);
    DEBUG_ASSERT(sdma_idx >= 0 && sdma_idx < IMX_SDMA_NUM);
    sdma_bd_vaddr = &sdmabd[sdma_idx][channel];
    sdma_bd_paddr = vaddr_to_paddr((void *)&sdmabd[sdma_idx][channel]);
#else
    ret = vmm_alloc_contiguous(vmm_get_kernel_aspace(), "sdmabd",
            sizeof(sdma_buffer_descriptor_t) * NUM_BD,
            &sdma_bd_vaddr, 0, 0, ARCH_MMU_FLAG_UNCACHED);
    printlk(LK_INFO, "%s:%d: sdmabd: vmm_alloc_contig returns %d, ptr %p\n",
            __PRETTY_FUNCTION__, __LINE__, ret, sdma_bd_vaddr);
    if (ret < 0) {
        printlk(LK_ERR, "%s:%d: error allocating sdma_bd_vaddr.\n",
                __PRETTY_FUNCTION__, __LINE__);
        goto out;
    }
    if (!sdma_bd_vaddr) {
        printlk(LK_ERR, "%s:%d: error allocating sdma_bd_vaddr.\n",
                __PRETTY_FUNCTION__, __LINE__);
        ret = ERR_NO_MEMORY;
        goto out;
    }
    sdma_bd_paddr = phys_to_dma(vaddr_to_paddr((void *)sdma_bd_vaddr));
#endif
    state->sdma_bd[channel] = (void *)sdma_bd_vaddr;
    state->sdma_bd_phys[channel] = sdma_bd_paddr;
    /* zero-out the BD array just allocated */
    memset(sdma_bd_vaddr, 0, sizeof(sdma_buffer_descriptor_t) * NUM_BD);
    printlk(LK_INFO, "%s:%d: sdma ccb[%d]: va: %p pa: %lx \n",
            __PRETTY_FUNCTION__, __LINE__, channel, state->sdma_bd[channel],
            state->sdma_bd_phys[channel]);
    /* Set channel CCB */
    ch_ccb->baseBDAddr    = sdma_bd_paddr;
    ch_ccb->currentBDAddr = sdma_bd_paddr;

    /* Zero the handle */
    memset(ch_handle, 0, sizeof(*ch_handle));

    ch_handle->base    = base;
    ch_handle->channel = channel;
    ch_handle->bdCount = 1U;
    ch_handle->context      = state->sdma_context;
    ch_handle->context_phys = state->sdma_context_phys;
    ch_handle->bd0      = state->sdma_bd0;
    ch_handle->bd0_phys = state->sdma_bd0_phys;
    ch_handle->bd       = state->sdma_bd[channel];
    ch_handle->bd_phys  = state->sdma_bd_phys[channel];

    goto out;

#ifndef IMX_SDMA_USE_OCRAM
free_buffers:
    vmm_free_region(vmm_get_kernel_aspace(), (vaddr_t)sdma_bd_vaddr);
#endif
out:
    return ret;
}

static status_t imx_sdma_dealloc_channel(struct device *dev, int channel)
{

#ifndef IMX_SDMA_USE_OCRAM
    struct imx_sdma_state *state;

    state = dev->state;
    vmm_free_region(vmm_get_kernel_aspace(), (vaddr_t) state->sdma_bd[channel]);
#endif
    return NO_ERROR;
}

static bool is_channel_available(struct dma_chan* chan)
{
    /* FIXME need a flag for channel status (assigned/available) */
    return chan->chan_available;
}

static status_t imx_sdma_request_channel(struct device *dev, struct dma_chan **chan)
{
    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);
    struct imx_sdma_state *state = dev->state;
    ASSERT(state);

    struct dma_chan *cur_chan = NULL;
    unsigned int i;
    for (i = 1; i < FSL_FEATURE_SDMA_MODULE_CHANNEL; i++) {
        cur_chan = &state->channels[i];
        printlk(LK_INFO, "%s:%d: dma_chan %p %s\n", __PRETTY_FUNCTION__,
                __LINE__, cur_chan, cur_chan->name);
        spin_lock_saved_state_t lock_state;
        spin_lock_irqsave(&state->lock, lock_state);
        if (is_channel_available(cur_chan)) {
            unsigned int channel = i;
            cur_chan->chan_available = false; /* channel is now assigned */
            spin_unlock_irqrestore(&state->lock, lock_state);
            printlk(LK_INFO, "%s:%d: using dma_chan number %d\n",
                    __PRETTY_FUNCTION__, __LINE__, channel);
            struct sdma_handle *sdma_handle = &state->sdma_handle[i];
            cur_chan->private = (void *) sdma_handle;
            status_t ret = NO_ERROR;
            /* allocate channel resources */
            ret = imx_sdma_alloc_channel(dev, channel);
            if (ret) {
                printlk(LK_ERR, "%s:%d: error: could not allocate channel %d.\n",
                        __PRETTY_FUNCTION__, __LINE__, channel);
                return ret;
            }
            /* set channel priority */
            SDMA_SetChannelPriority(state->io_base, channel, kDMA_ChannelPriority0);
#ifdef SDMA_ENABLE_EVT_ERROR
            /* Enable error interrupt */
            SDMA_EnableChannelErrorInterrupts(state->io_base, channel);
#endif
            break;
        } else {
            spin_unlock_irqrestore(&state->lock, lock_state);
        }
    }
    *chan = cur_chan;
    printlk(LK_VERBOSE, "%s: exit\n", __PRETTY_FUNCTION__);

    return NO_ERROR;
}

static status_t imx_sdma_prepare_dma_cyclic(struct device *dev,
   struct dma_chan *chan, paddr_t buf_addr, size_t buf_len,
   size_t period_len, enum dma_data_direction direction,
   struct dma_descriptor ** descriptor)
{
    struct imx_sdma_state *state;
    state = dev->state;
    ASSERT(state);
    SDMAARM_Type *base = state->io_base;
    ASSERT(base);

    struct sdma_handle *sdma_chan = to_sdma_handle(chan);
    ASSERT(sdma_chan);
    unsigned int channel = sdma_chan->channel;
    struct sdma_transfer_config *sdma_config = &sdma_chan->cfg;
    ASSERT(sdma_config);

    struct sdma_buffer_descriptor *sdma_bd = &state->sdma_bd[channel][0];
    ASSERT(sdma_bd);
    uint32_t dst_addr, src_addr;
    sdma_transfer_type_t sdma_direction;

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    /* allocate dma_descriptor. Must be free by client driver */
    *descriptor = malloc(sizeof(struct dma_descriptor));
    (*descriptor)->chan = chan;

    printlk(LK_VERBOSE, "%s: looking for dma channel %s\n", __PRETTY_FUNCTION__, chan->name);

    struct device_cfg_dma_channel *dma_cfg =
        device_config_get_dma_channel_by_name(chan->client_device->config, chan->name);
    ASSERT(dma_cfg);
    if (direction == DMA_DEV_TO_MEM) {
        dst_addr = buf_addr;
        src_addr = chan->slave_config.src_addr;
        sdma_direction = kSDMA_PeripheralToMemory;
    } else {
        src_addr = buf_addr;
        dst_addr = chan->slave_config.dst_addr;
        sdma_direction = kSDMA_MemoryToPeripheral;
    }
    ASSERT(src_addr != 0);
    ASSERT(dst_addr != 0);

    uint32_t eventSource = dma_cfg->dma_spec.args[0];
    enum sdma_peripheral peripheral = dma_cfg->dma_spec.args[1];

    /* handling for SAI multiFIFO */
    if ((peripheral == kSDMA_PeripheralMultiFifoSaiRX)
        || (peripheral == kSDMA_PeripheralMultiFifoSaiTX)) {
        uint32_t fifo_offset = 0; /* FIFOs start at dataline #0 */

        /* check input parameters */
        if (chan->slave_config.src_maxburst
                > kSDMA_MultiFifoWatermarkLevelMask) {
                return ERR_INVALID_ARGS;
        }

        /* set multi fifo configurations */
        SDMA_SetMultiFifoConfig(sdma_config,
            chan->slave_config.src_fifo_num, fifo_offset,
            chan->slave_config.src_sample_num);
        printlk(LK_DEBUG, "chan->slave_config.src_fifo_num=%d\n",
            chan->slave_config.src_fifo_num);
        printlk(LK_DEBUG, "chan->slave_config.src_maxburst=%d\n",
            chan->slave_config.src_maxburst);
    }

    /* handling for multiSAI */
    if ((peripheral == kSDMA_PeripheralMultiSaiRX)
        || (peripheral == kSDMA_PeripheralMultiSaiTX)) {
        /* set multi fifo configurations */
        SDMA_SetMultiSaiConfig(sdma_config,
            chan->slave_config.src_sample_num);
    }

    printlk(LK_DEBUG, "SDMA_PrepareTransfer src_addr=%x, dst_addr=%x,"
        " period_len=%ld src_addr_width=%d buf_len=%ld\n",
        src_addr, dst_addr, period_len,
        chan->slave_config.src_addr_width, buf_len);
    SDMA_PrepareTransfer(
        sdma_config,
        src_addr,
        dst_addr,
        chan->slave_config.src_addr_width,
        chan->slave_config.dst_addr_width,
        chan->slave_config.src_maxburst, /* bytesEachRequest */
        period_len, /* period_len */
        buf_len, /* transferSize */
        eventSource, /* eventSource */
        peripheral, /* sdma_peripheral_t peripheral */
        sdma_direction, /* sdma_transfer_type_t type */
        state->script_addrs);
    printlk(LK_DEBUG, "DMA%d script addr = %x/%u, req %d, peripheral %d\n",
                  state->bus_id, sdma_config->scriptAddr,
                  sdma_config->scriptAddr,
                  eventSource, peripheral);

    /* Set event source channel */
    if (sdma_direction != kSDMA_MemoryToMemory) {
        SDMA_Event_Enable(sdma_chan, dma_cfg->dma_spec.args[0]);
    }
    /* Set SDMA Runnable Channel Selection Control */
    SDMA_Config_Ownership(sdma_chan, sdma_config);

    switch (sdma_config->type) {
    case kSDMA_MemoryToMemory:
        break;
    case kSDMA_MemoryToPeripheral:
    case kSDMA_PeripheralToMemory:
        SDMA_PrepDmaCyclic(
            sdma_chan,
            sdma_config,
            sdma_bd);
        break;
    default:
        break;
    }

    /* Load the context */
    printlk(LK_INFO, "%s:%d: SDMA_LoadContext\n", __PRETTY_FUNCTION__, __LINE__);
    SDMA_LoadContext(sdma_chan, sdma_config, &state->sdma_bd0[0], state->sdma_context_phys);

    /* set channel priority */
    SDMA_SetChannelPriority(state->io_base, channel, dma_cfg->dma_spec.args[2]);

    printlk(LK_VERBOSE, "%s: exit\n", __PRETTY_FUNCTION__);

    return NO_ERROR;
}

static status_t imx_sdma_slave_config(struct device *dev, struct dma_chan *ch,
                                        struct dma_slave_config *cfg)
{
    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    ch->slave_config = *cfg;

    printlk(LK_VERBOSE, "%s: exit\n", __PRETTY_FUNCTION__);

    return NO_ERROR;
}

/** code from SDMA_SubmitTransfer
 *  This function wraps: SDMA_submitTransfer()
 *  - sets channel's BD (buffer descriptor).
 *  - sets channel's DMA request source.
 *  - sets and loads channel's context.
 * @param dev device
 * @param channel DMA channel
 * @param cfg channel's transfer configuration
 */
static status_t imx_sdma_submit(struct device *dev, struct dma_descriptor *desc)
{

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);
    struct imx_sdma_state *state;
    state = dev->state;
    ASSERT(state);
    SDMAARM_Type *base = state->io_base;
    ASSERT(base);
    struct dma_chan *chan = desc->chan;

    struct sdma_handle *handle = to_sdma_handle(chan);
    ASSERT(handle);

    chan->desc = desc;

    //SDMA_EnableChannel(handle);

    printlk(LK_VERBOSE, "%s: exit\n", __PRETTY_FUNCTION__);
    return NO_ERROR;
}

static status_t imx_sdma_terminate(struct device *dev, struct dma_chan *chan)
{
    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    status_t ret = NO_ERROR;
    struct imx_sdma_state *state = dev->state;
    ASSERT(state);

    struct sdma_handle *sdma_handle = to_sdma_handle(chan);
    unsigned int channel = sdma_handle->channel;

    printlk(LK_DEBUG, "Terminate DMA Channel %d\n", channel);

    SDMA_DisableChannel(sdma_handle);
    /* deallocate channel resources */
    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&state->lock, lock_state);
#ifdef SDMA_ENABLE_EVT_ERROR
    SDMA_DisableChannelErrorInterrupts(state->io_base, channel);
#endif
    ret = imx_sdma_dealloc_channel(dev, channel);
    chan->chan_available = true; /* channel is now free */
    chan->desc = NULL;
    spin_unlock_irqrestore(&state->lock, lock_state);

    printlk(LK_VERBOSE, "%s: exit\n", __PRETTY_FUNCTION__);

    return ret;
}

static status_t imx_sdma_resume_channel(struct device *dev, struct dma_chan *chan)
{
    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);
    struct sdma_handle *sdma_chan = to_sdma_handle(chan);
    status_t ret = NO_ERROR;

    SDMA_EnableChannel(sdma_chan);

    printlk(LK_VERBOSE, "%s: exit\n", __PRETTY_FUNCTION__);

    return ret;
}

static status_t imx_sdma_pause_channel(struct device *dev, struct dma_chan *chan)
{
    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);
    struct sdma_handle *sdma_chan = to_sdma_handle(chan);
    status_t ret = NO_ERROR;

    SDMA_DisableChannel(sdma_chan);

    printlk(LK_VERBOSE, "%s: exit\n", __PRETTY_FUNCTION__);

    return ret;
}

static struct device_class sdma_device_class = {
    .name = "sdma",
};

static struct sdma_ops the_ops = {
    .std = {
        .device_class = &sdma_device_class,
        .init = imx_sdma_init,
    },
    .request_channel = imx_sdma_request_channel,
    .prepare_dma_cyclic = imx_sdma_prepare_dma_cyclic,
    .slave_config = imx_sdma_slave_config,
    .submit = imx_sdma_submit,
    .terminate = imx_sdma_terminate,
    .pause_channel = imx_sdma_pause_channel,
    .resume_channel = imx_sdma_resume_channel,
};

/* DMA driver must be initialized before its clients:
 * Raised level from DRIVER_INIT_PLATFORM to DRIVER_INIT_CORE
 */
DRIVER_EXPORT_WITH_LVL(sdma, &the_ops.std, DRIVER_INIT_CORE);

struct device *class_sdma_get_device_by_id(int id)
{
    struct device *dev = NULL;
    struct imx_sdma_state *state = NULL;

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&imx_sdma_list_lock, lock_state);

    list_for_every_entry(&imx_sdma_list, state, struct imx_sdma_state, node) {
        if (state->bus_id == id) {
            dev = state->device;
            break;
        }
    }

    spin_unlock_irqrestore(&imx_sdma_list_lock, lock_state);

    return dev;
}
