/*
 * Copyright 2018-2020 NXP
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <dev/class/i2c.h>
#include <platform/i2c.h>
#include <dev/interrupt.h>

#include <kernel/spinlock.h>
#include <kernel/mutex.h>
#include <kernel/thread.h>
#include <kernel/event.h>
#include <list.h>
#include <err.h>
#include <malloc.h>
#include <assert.h>
#include <string.h>

#include <lib/appargs.h>

#include "delay.h"


/*
 *
 * I2C HW block related definitions
 *
 */

/** I2C - Register Layout Typedef */
#define __IO volatile
typedef struct {
    __IO uint16_t IADR;                              /**< I2C Address Register, offset: 0x0 */
    uint8_t RESERVED_0[2];
    __IO uint16_t IFDR;                              /**< I2C Frequency Divider Register, offset: 0x4 */
    uint8_t RESERVED_1[2];
    __IO uint16_t I2CR;                              /**< I2C Control Register, offset: 0x8 */
    uint8_t RESERVED_2[2];
    __IO uint16_t I2SR;                              /**< I2C Status Register, offset: 0xC */
    uint8_t RESERVED_3[2];
    __IO uint16_t I2DR;                              /**< I2C Data I/O Register, offset: 0x10 */
} I2C_Type;

/* ----------------------------------------------------------------------------
   -- I2C Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Register_Masks I2C Register Masks
 * @{
 */

/*! @name IADR - I2C Address Register */
#define I2C_IADR_ADR_MASK                        (0xFEU)
#define I2C_IADR_ADR_SHIFT                       (1U)
#define I2C_IADR_ADR(x)                          (((uint16_t)(((uint16_t)(x)) << I2C_IADR_ADR_SHIFT)) & I2C_IADR_ADR_MASK)

/*! @name IFDR - I2C Frequency Divider Register */
#define I2C_IFDR_IC_MASK                         (0x3FU)
#define I2C_IFDR_IC_SHIFT                        (0U)
#define I2C_IFDR_IC(x)                           (((uint16_t)(((uint16_t)(x)) << I2C_IFDR_IC_SHIFT)) & I2C_IFDR_IC_MASK)

/*! @name I2CR - I2C Control Register */
#define I2C_I2CR_RSTA_MASK                       (0x4U)
#define I2C_I2CR_RSTA_SHIFT                      (2U)
#define I2C_I2CR_RSTA(x)                         (((uint16_t)(((uint16_t)(x)) << I2C_I2CR_RSTA_SHIFT)) & I2C_I2CR_RSTA_MASK)
#define I2C_I2CR_TXAK_MASK                       (0x8U)
#define I2C_I2CR_TXAK_SHIFT                      (3U)
#define I2C_I2CR_TXAK(x)                         (((uint16_t)(((uint16_t)(x)) << I2C_I2CR_TXAK_SHIFT)) & I2C_I2CR_TXAK_MASK)
#define I2C_I2CR_MTX_MASK                        (0x10U)
#define I2C_I2CR_MTX_SHIFT                       (4U)
#define I2C_I2CR_MTX(x)                          (((uint16_t)(((uint16_t)(x)) << I2C_I2CR_MTX_SHIFT)) & I2C_I2CR_MTX_MASK)
#define I2C_I2CR_MSTA_MASK                       (0x20U)
#define I2C_I2CR_MSTA_SHIFT                      (5U)
#define I2C_I2CR_MSTA(x)                         (((uint16_t)(((uint16_t)(x)) << I2C_I2CR_MSTA_SHIFT)) & I2C_I2CR_MSTA_MASK)
#define I2C_I2CR_IIEN_MASK                       (0x40U)
#define I2C_I2CR_IIEN_SHIFT                      (6U)
#define I2C_I2CR_IIEN(x)                         (((uint16_t)(((uint16_t)(x)) << I2C_I2CR_IIEN_SHIFT)) & I2C_I2CR_IIEN_MASK)
#define I2C_I2CR_IEN_MASK                        (0x80U)
#define I2C_I2CR_IEN_SHIFT                       (7U)
#define I2C_I2CR_IEN(x)                          (((uint16_t)(((uint16_t)(x)) << I2C_I2CR_IEN_SHIFT)) & I2C_I2CR_IEN_MASK)

/*! @name I2SR - I2C Status Register */
#define I2C_I2SR_RXAK_MASK                       (0x1U)
#define I2C_I2SR_RXAK_SHIFT                      (0U)
#define I2C_I2SR_RXAK(x)                         (((uint16_t)(((uint16_t)(x)) << I2C_I2SR_RXAK_SHIFT)) & I2C_I2SR_RXAK_MASK)
#define I2C_I2SR_IIF_MASK                        (0x2U)
#define I2C_I2SR_IIF_SHIFT                       (1U)
#define I2C_I2SR_IIF(x)                          (((uint16_t)(((uint16_t)(x)) << I2C_I2SR_IIF_SHIFT)) & I2C_I2SR_IIF_MASK)
#define I2C_I2SR_SRW_MASK                        (0x4U)
#define I2C_I2SR_SRW_SHIFT                       (2U)
#define I2C_I2SR_SRW(x)                          (((uint16_t)(((uint16_t)(x)) << I2C_I2SR_SRW_SHIFT)) & I2C_I2SR_SRW_MASK)
#define I2C_I2SR_IAL_MASK                        (0x10U)
#define I2C_I2SR_IAL_SHIFT                       (4U)
#define I2C_I2SR_IAL(x)                          (((uint16_t)(((uint16_t)(x)) << I2C_I2SR_IAL_SHIFT)) & I2C_I2SR_IAL_MASK)
#define I2C_I2SR_IBB_MASK                        (0x20U)
#define I2C_I2SR_IBB_SHIFT                       (5U)
#define I2C_I2SR_IBB(x)                          (((uint16_t)(((uint16_t)(x)) << I2C_I2SR_IBB_SHIFT)) & I2C_I2SR_IBB_MASK)
#define I2C_I2SR_IAAS_MASK                       (0x40U)
#define I2C_I2SR_IAAS_SHIFT                      (6U)
#define I2C_I2SR_IAAS(x)                         (((uint16_t)(((uint16_t)(x)) << I2C_I2SR_IAAS_SHIFT)) & I2C_I2SR_IAAS_MASK)
#define I2C_I2SR_ICF_MASK                        (0x80U)
#define I2C_I2SR_ICF_SHIFT                       (7U)
#define I2C_I2SR_ICF(x)                          (((uint16_t)(((uint16_t)(x)) << I2C_I2SR_ICF_SHIFT)) & I2C_I2SR_ICF_MASK)

/*! @name I2DR - I2C Data I/O Register */
#define I2C_I2DR_DATA_MASK                       (0xFFU)
#define I2C_I2DR_DATA_SHIFT                      (0U)
#define I2C_I2DR_DATA(x)                         (((uint16_t)(((uint16_t)(x)) << I2C_I2DR_DATA_SHIFT)) & I2C_I2DR_DATA_MASK)


static inline int imx_i2c_bus_is_busy(I2C_Type *base)
{
    return !!(base->I2SR & I2C_I2SR_IBB_MASK);
}

static inline int imx_i2c_bus_is_free(I2C_Type *base)
{
    return !(base->I2SR & I2C_I2SR_IBB_MASK);
}

static inline int imx_i2c_arbitration_is_lost(I2C_Type *base)
{
    return !!(base->I2SR & I2C_I2SR_IAL_MASK);
}

static inline void imx_i2c_disable_interrupt(I2C_Type *base)
{
    base->I2CR &= ~I2C_I2CR_IIEN_MASK;
}

static inline void imx_i2c_enable_interrupt(I2C_Type *base)
{
    base->I2CR |= I2C_I2CR_IIEN_MASK;
}

static void _imx_i2c_start(I2C_Type *base, uint8_t address, unsigned is_read, unsigned repeat)
{
    unsigned idr;
    unsigned icr = I2C_I2CR_MTX_MASK;
    idr = (((uint16_t)address) << 1U);
    if (is_read)
        idr |= 1;

    if (repeat)
        icr |= I2C_I2CR_RSTA_MASK;
    else
        icr |= I2C_I2CR_MSTA_MASK;

    base->I2CR |= icr;
    base->I2DR = idr;
}

static inline void imx_i2c_start(I2C_Type *base, uint8_t addr, unsigned is_read)
{
    _imx_i2c_start(base, addr, is_read, 0);
}

static inline void imx_i2c_restart(I2C_Type *base, uint8_t addr, unsigned is_read)
{
    _imx_i2c_start(base, addr, is_read, 1);
}

static inline void imx_i2c_stop(I2C_Type *base)
{
    udelay(100);
    if (base->I2CR & I2C_I2CR_MSTA_MASK)
        base->I2CR &= ~(I2C_I2CR_MSTA_MASK | I2C_I2CR_MTX_MASK | I2C_I2CR_TXAK_MASK);
}

/*
 *
 * I2C driver related definitions
 *
 */

enum i2c_fsm_state {
    i2c_fsm_state_idle          = 0,
    i2c_fsm_state_error         = 1,
    i2c_fsm_state_address       = 2,
    i2c_fsm_state_command       = 3,
    i2c_fsm_state_tx            = 4,
    i2c_fsm_state_rx_address    = 5,
    i2c_fsm_state_set_rx        = 6,
    i2c_fsm_state_rx            = 7,
    i2c_fsm_state_done          = 8,
};

static const char *get_fsm_state_string(enum i2c_fsm_state state)
{
    static const char *fsm_state_string[] = {
        "i2c_fsm_state_idle",
        "i2c_fsm_state_error",
        "i2c_fsm_state_address",
        "i2c_fsm_state_command",
        "i2c_fsm_state_tx",
        "i2c_fsm_state_rx_address",
        "i2c_fsm_state_set_rx",
        "i2c_fsm_state_rx",
        "i2c_fsm_state_done",
    };
    if (state > i2c_fsm_state_done)
        return "unknown";

    return fsm_state_string[((unsigned)state)];
}

struct imx_i2c_xfer {
    unsigned flags;
#define IMX_I2C_FLAG_READ       (0x1U << 0)
#define IMX_I2C_FLAG_REGISTER   (0x1U << 1)
    uint8_t address;
    uint8_t reg;
    uint8_t *data;
    size_t len;
    status_t status;
};

static inline int imx_i2c_is_xfer_reg(struct imx_i2c_xfer *xfer)
{
    return !!(xfer->flags & IMX_I2C_FLAG_REGISTER);
}

static inline int imx_i2c_is_xfer_stream(struct imx_i2c_xfer *xfer)
{
    return !(xfer->flags & IMX_I2C_FLAG_REGISTER);
}

static inline int imx_i2c_is_xfer_read(struct imx_i2c_xfer *xfer)
{
    return !!(xfer->flags &IMX_I2C_FLAG_READ);
}

static inline int imx_i2c_is_xfer_write(struct imx_i2c_xfer *xfer)
{
    return !(xfer->flags &IMX_I2C_FLAG_READ);
}


struct imx_i2c_state {
    int bus_id;
    I2C_Type *io_base;
    spin_lock_t lock;
    mutex_t mutex;
    event_t wait;
    enum i2c_fsm_state fsm_state;
    struct imx_i2c_xfer xfer;
    struct device *device;
    struct list_node node;
};


#define I2C_TRACE(state, msg) \
        printlk(LK_NOTICE, "%s:%d: [%d:%d] - %s : %s\n", \
                        __PRETTY_FUNCTION__, \
                        __LINE__, \
                        state->bus_id, \
                        state->xfer.address, \
                        get_fsm_state_string(state->fsm_state), \
                        msg)

static inline void imx_i2c_set_fsm_state(struct imx_i2c_state *state,
        enum i2c_fsm_state fsm_state,
        status_t status)
{

    printlk(LK_INFO, "%s:%d: Switch state from %s to %s\n", __PRETTY_FUNCTION__,
            __LINE__, get_fsm_state_string(state->fsm_state),
            get_fsm_state_string(fsm_state));
//    DEBUG_ASSERT(fsm_state != i2c_fsm_state_error);
    state->fsm_state = fsm_state;
    state->xfer.status = status;
    smp_wmb();
}

static inline int imx_i2c_should_ignore_nak(struct imx_i2c_state *state)
{
    if ((state->fsm_state == i2c_fsm_state_tx) && (state->xfer.len == 0))
        return 1;

    if ((state->fsm_state == i2c_fsm_state_rx) && (state->xfer.len == 1))
        return 1;

    return 0;
}

static inline void imx_i2c_set_txak_if_needed(struct imx_i2c_state *state)
{
    if (state->xfer.len == 1)
        state->io_base->I2CR |= I2C_I2CR_TXAK_MASK;
}

static enum handler_return imx_i2c_isr(void *arg)
{
    struct device *dev = (struct device *)arg;
    ASSERT(dev);
    struct imx_i2c_state *state = dev->state;
    ASSERT(state);
    I2C_Type *base= state->io_base;
    ASSERT(base);
    uint16_t status;
    enum i2c_fsm_state next_state;
    status_t next_status;
    struct imx_i2c_xfer *xfer = &state->xfer;

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    /* A interrupt is generated in any of of the following conditions
     *
     * - One byte transfer is completed
     * - An address is received that matches its own specific address in slave
     *   receive mode
     * - Arbitration is lost
     *
     * */

    status = base->I2SR;

    /* Clear interrupt flag */
    base->I2SR &= ~I2C_I2SR_IIF_MASK;

    printlk(LK_INFO, "%s:%d: current state: %s\n", __PRETTY_FUNCTION__,
            __LINE__, get_fsm_state_string(state->fsm_state));
    next_state = state->fsm_state;
    next_status = NO_ERROR;

    /* Check Arbitration lost flag */
    if (status & I2C_I2SR_IAL_MASK) {
        I2C_TRACE(state,"Arbitration lost detected");
        /* Clear the Arbitration lost flag */
        base->I2SR &= ~I2C_I2SR_IAL_MASK;

        /* FIXME: Do this in Tread context
         * Reset the controller
         * */
        base->I2CR &= ~I2C_I2CR_IEN_MASK;
        base->I2CR |= I2C_I2CR_IEN_MASK;

        /* Signal an error */
        next_state = i2c_fsm_state_error;
        next_status = ERR_CANCELLED;
        goto wake;
    }

    /* Should we ignore NAK bit ? */
    if (imx_i2c_should_ignore_nak(state))
        status &= ~I2C_I2SR_RXAK_MASK;

    /* NAK handling */
    if (status & I2C_I2SR_RXAK_MASK) {
        I2C_TRACE(state,"NAK detected");
        /* Signal an error */
        next_state = i2c_fsm_state_error;
        next_status = ERR_I2C_NACK;
        goto wake;
    }


    /* The slave address should have been sent ... */
    if (state->fsm_state == i2c_fsm_state_address) {
        if (imx_i2c_is_xfer_reg(xfer))
            next_state = i2c_fsm_state_command;
        else if (imx_i2c_is_xfer_read(xfer))
            next_state = i2c_fsm_state_set_rx;
        else
            next_state = i2c_fsm_state_tx;
        imx_i2c_set_fsm_state(state, next_state, NO_ERROR);
    }

    switch (state->fsm_state) {
            volatile uint8_t dummy;
        case i2c_fsm_state_command:
            base->I2DR = xfer->reg;
            if (imx_i2c_is_xfer_read(xfer))
                next_state = i2c_fsm_state_rx_address;
            else
                next_state = i2c_fsm_state_tx;
            break;
        case i2c_fsm_state_tx:
            if (xfer->len) {
                base->I2DR = *xfer->data;
                xfer->data++;
                xfer->len--;
            }
            else {
                next_state = i2c_fsm_state_done;
                next_status = NO_ERROR;
                goto wake;
            }
            break;
        case i2c_fsm_state_rx_address:
            if (
                (status & I2C_I2SR_IBB_MASK)
                && (!(base->I2CR & I2C_I2CR_MSTA_MASK))) {
                I2C_TRACE(state, "Bus ownership lost\n");
                next_state = i2c_fsm_state_error;
                next_status = ERR_BUSY;
                goto wake;
            }
            imx_i2c_restart(base, xfer->address, 1);
            next_state = i2c_fsm_state_set_rx;
            break;
        case i2c_fsm_state_set_rx:
            base->I2CR &= ~(I2C_I2CR_MTX_MASK | I2C_I2CR_TXAK_MASK);
            imx_i2c_set_txak_if_needed(state);
            dummy = base->I2DR;
            dummy++;
            next_state = i2c_fsm_state_rx;
            break;
        case i2c_fsm_state_rx:
            *xfer->data = base->I2DR;
            xfer->data++;
            xfer->len--;
            if (xfer->len == 0) {
                next_state = i2c_fsm_state_done;
                next_status = NO_ERROR;
                goto wake;
            }
            imx_i2c_set_txak_if_needed(state);
            break;
        /* Those following states are not supposed to happen --> error */
        case i2c_fsm_state_idle:
        case i2c_fsm_state_done:
        case i2c_fsm_state_error:
        case i2c_fsm_state_address:
        default:
            panic("Inconsistent state %d in i2c isr\n", state->fsm_state);
            break;
    }

    imx_i2c_set_fsm_state(state, next_state, next_status);

    printlk(LK_VERBOSE, "%s: exit\n", __PRETTY_FUNCTION__);
    return INT_NO_RESCHEDULE;
wake:
    imx_i2c_set_fsm_state(state, next_state, next_status);

    /* Disable IP interrupt */
    imx_i2c_disable_interrupt(base);

    /* wake up the thread initiating the transfer */
    event_signal(&state->wait, false);
    printlk(LK_INFO, "%s:%d: Signal sent\n", __PRETTY_FUNCTION__, __LINE__);

    /* We want to force a reschedule to wake up as early as possible */

    printlk(LK_VERBOSE, "%s: exit\n", __PRETTY_FUNCTION__);
    return INT_RESCHEDULE;
}
#define PINMUX_ENABLE 1
#define POWER_ENABLE 1

/*! @brief SCL clock divider used to calculate baudrate. */
static const uint16_t s_i2cDividerTable[] = {
    30,  32,  36,  42,  48,  52,  60,  72,
    80,   88,   104,  128,  144,  160,  192,  240,
    288, 320, 384, 480, 576, 640, 768, 960,
    1152, 1280, 1536, 1920, 2304, 2560, 3072, 3840,
    22,  24,  26,  28,  32,  36,  40,  44,
    48,   56,   64,   72,   80,   96,   112,  128,
    160, 192, 224, 256, 320, 384, 448, 512,
    640,  768,  896,  1024, 1280, 1536, 1792, 2048
};

static void I2C_MasterSetBaudRate(I2C_Type *base, uint32_t baudRate_Bps, uint32_t srcClock_Hz)
{
    uint32_t computedRate;
    uint32_t absError;
    uint32_t bestError = UINT32_MAX;
    uint32_t bestIcr = 0u;
    uint8_t i;

    /* Scan table to find best match. */
    for (i = 0u; i < sizeof(s_i2cDividerTable) / sizeof(s_i2cDividerTable[0]); ++i) {
        computedRate = srcClock_Hz / s_i2cDividerTable[i];
        absError = baudRate_Bps > computedRate ? (baudRate_Bps - computedRate) : (computedRate - baudRate_Bps);

        if (absError < bestError) {
            bestIcr = i;
            bestError = absError;

            /* If the error is 0, then we can stop searching because we won't find a better match. */
            if (absError == 0) {
                break;
            }
        }
    }

    /* Set frequency register based on best settings. */
    base->IFDR = I2C_IFDR_IC(bestIcr);
}


static void imx_i2c_reset(I2C_Type *base, unsigned baud_rate, unsigned clk_rate)
{
    /* Reset the module. */
    base->IADR = 0;
    base->IFDR = 0;
    base->I2CR = 0;
    base->I2SR = 0;

    /* Configure baud rate. */
    I2C_MasterSetBaudRate(base, baud_rate, clk_rate);

    /* Enable the I2C peripheral based on the configuration. */
    base->I2CR = I2C_I2CR_IEN_MASK;

}

static struct list_node imx_i2c_list = LIST_INITIAL_VALUE(imx_i2c_list);
static spin_lock_t imx_i2c_list_lock = SPIN_LOCK_INITIAL_VALUE;

static status_t imx_i2c_init(struct device *dev)
{
    const struct device_config_data *config = dev->config;
    struct imx_i2c_state *state;
    status_t ret = 0;

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);
    /* Let's do all sanity check in a row */
    ASSERT(config);

    state = malloc(sizeof(struct imx_i2c_state));
    ASSERT(state);
    memset(state, 0, sizeof(*state));
    dev->state = state;
    state->device = dev;

    state->bus_id = config->bus_id;
    spin_lock_init(&state->lock);
    mutex_init(&state->mutex);
    event_init(&state->wait, false, 0);

    struct device_cfg_reg *reg =
                        device_config_get_register_by_name(config, "core");
    ASSERT(reg);
    struct device_cfg_irq *irq =
                        device_config_get_irq_by_name(config, "core");
    ASSERT(irq);
    struct device_cfg_clk *clk =
                        device_config_get_clk_by_name(config, "core");
    ASSERT(clk);

    devcfg_set_clocks(config->clks);

    devcfg_set_pins_by_name(config->pins_cfg,
                            config->pins_cfg_cnt, "default");

    state->io_base = (I2C_Type *) reg->vbase;

    /* FIXME: Get the baudrate from the platform config data */

    unsigned baud_rate = 100000;
    of_device_get_int32(dev, "baud-rate", &baud_rate);
    printlk(LK_INFO, "%s:%d: Using %d as i2c baud rate\n", __PRETTY_FUNCTION__,
            __LINE__, baud_rate);
    imx_i2c_reset(state->io_base, baud_rate, clk->rate);
    imx_i2c_set_fsm_state(state, i2c_fsm_state_idle, NO_ERROR);

    register_int_handler(irq->irq, imx_i2c_isr, dev);
    unmask_interrupt(irq->irq);

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&imx_i2c_list_lock, lock_state);

    list_add_tail(&imx_i2c_list, &state->node);

    spin_unlock_irqrestore(&imx_i2c_list_lock, lock_state);

    printlk(LK_INFO, "%s:%d: dev: %p state: %p state->dev: %p dev->state: %p\n",
            __PRETTY_FUNCTION__, __LINE__, dev, state, state->device,
            dev->state);
    printlk(LK_VERBOSE, "%s: exit\n", __PRETTY_FUNCTION__);

    return ret;
}

static int imx_i2c_needs_stop(struct imx_i2c_state *state)
{
    struct imx_i2c_xfer *xfer = &state->xfer;

    return ((state->fsm_state == i2c_fsm_state_done) || (xfer->status == ERR_I2C_NACK));
}

static status_t imx_i2c_do_xfer(struct imx_i2c_state *state, struct imx_i2c_xfer *_xfer)
{

    I2C_Type *base = state->io_base;
    struct imx_i2c_xfer *xfer = &state->xfer;
    status_t ret;
    unsigned retries = 0;

    mutex_acquire(&state->mutex);
retry:
    while (imx_i2c_bus_is_busy(base))
        udelay(50);

    *xfer = *_xfer;

    /* TODO: Handle gracefully a state different than idle */
    DEBUG_ASSERT(state->fsm_state == i2c_fsm_state_idle);

    /* Disable local IP interrupt */
    imx_i2c_disable_interrupt(base);

    /* Clear Status Flag: Must write 0 to clear... */
    base->I2SR = (uint16_t) ~(I2C_I2SR_IIF_MASK | I2C_I2SR_IAL_MASK);
    unsigned direction_is_read;

    if (imx_i2c_is_xfer_reg(xfer))
        direction_is_read = 0;
    else
        direction_is_read = imx_i2c_is_xfer_read(xfer);

    imx_i2c_set_fsm_state(state, i2c_fsm_state_address, NO_ERROR);

    imx_i2c_start(base, xfer->address, direction_is_read);

    /* Enable local IP interrupt */
    imx_i2c_enable_interrupt(base);

    /* Wait for the end of the transfer - 1s of timeout */
    ret = event_wait_timeout(&state->wait, 1000);

    event_unsignal(&state->wait);

    if (ret == ERR_TIMED_OUT) {
        imx_i2c_disable_interrupt(base);
    }
    else if (state->fsm_state != i2c_fsm_state_done) {
        DEBUG_ASSERT(xfer->status != NO_ERROR);
        ret = xfer->status;
    }
    else {
        DEBUG_ASSERT(xfer->status == NO_ERROR);
        ret = xfer->status;
    }

    if (imx_i2c_needs_stop(state))
        imx_i2c_stop(base);

    /*    I2C_TRACE(state, "XFER completed"); */

    if ((xfer->status == ERR_I2C_NACK) && (retries++ < 5)) {
        thread_sleep(1 * retries);
        I2C_TRACE(state, "Retrying xfer after xfer nack\n");
        imx_i2c_set_fsm_state(state, i2c_fsm_state_idle, NO_ERROR);
        goto retry;
    }
    retries = 0;
    if ((xfer->status == ERR_CANCELLED) && (retries++ < 5)) {
        thread_sleep(1 * retries);
        I2C_TRACE(state, "Retrying xfer after arbitration lost \n");
        imx_i2c_set_fsm_state(state, i2c_fsm_state_idle, NO_ERROR);
        goto retry;
    }



    DEBUG_ASSERT(state->fsm_state == i2c_fsm_state_done);
    /*
     * TODO: FIXME: Shall we check the bus is free ?
     *  imx_i2c_bus_is_free()
    */
    imx_i2c_set_fsm_state(state, i2c_fsm_state_idle, NO_ERROR);


    mutex_release(&state->mutex);

    return ret;

}

static status_t imx_i2c_do_xfer_reg(
    struct device *dev,
    uint8_t addr,
    uint8_t reg,
    uint8_t *value,
    size_t cnt,
    int is_read)
{
    struct imx_i2c_state *state = dev->state;
    ASSERT(state);
    struct imx_i2c_xfer xfer;

    printlk(LK_INFO, "%s:%d: read: %d, dev: %p state: %p addr: %#x, reg: %#x, buf: %p, *buf: %#x, len: %lu \n",
            __PRETTY_FUNCTION__, __LINE__, is_read, dev, state, addr, reg,
            value, *value, cnt);

    xfer.flags = IMX_I2C_FLAG_REGISTER;

    if (is_read)
        xfer.flags |= IMX_I2C_FLAG_READ;

    xfer.address = addr;
    xfer.reg = reg;
    xfer.data = value;
    xfer.len = cnt;

    return imx_i2c_do_xfer(state, &xfer);
}

static status_t imx_i2c_do_xfer_stream(
    struct device *dev,
    uint8_t addr,
    uint8_t *value,
    size_t cnt,
    int is_read)
{
    struct imx_i2c_state *state = dev->state;
    ASSERT(state);
    struct imx_i2c_xfer xfer;

    if (is_read)
        xfer.flags = IMX_I2C_FLAG_READ;

    xfer.address = addr;
    xfer.data = value;
    xfer.len = cnt;

    return imx_i2c_do_xfer(state, &xfer);
}

static status_t imx_i2c_read_reg(struct device *dev, uint8_t addr, uint8_t reg, void *value)
{
    return imx_i2c_do_xfer_reg(dev, addr, reg, (uint8_t *)value, 1, 1);
}

static status_t imx_i2c_write_reg(struct device *dev, uint8_t addr, uint8_t reg, uint8_t value)
{
    return imx_i2c_do_xfer_reg(dev, addr, reg, &value, 1, 0);
}

static status_t imx_i2c_write(struct device *dev, uint8_t addr, const void *buf, size_t len)
{
    return imx_i2c_do_xfer_stream(dev, addr, (uint8_t *) buf, len, 0);
}

static status_t imx_i2c_read(struct device *dev, uint8_t addr, void *buf, size_t len)
{
    return imx_i2c_do_xfer_stream(dev, addr, (uint8_t *) buf, len, 1);
}

static struct device_class i2c_device_class = {
    .name = "i2c",
};

static struct i2c_ops the_ops = {
    .std = {
        .device_class = &i2c_device_class,
        .init = imx_i2c_init,
    },
    .write = imx_i2c_write,
    .read = imx_i2c_read,
    .write_reg = imx_i2c_write_reg,
    .read_reg = imx_i2c_read_reg,
};

DRIVER_EXPORT_WITH_LVL(i2c, &the_ops.std, DRIVER_INIT_CORE);

static struct device *imx_i2c_get_device_by_id(int id)
{
    struct device *dev = NULL;
    struct imx_i2c_state *state = NULL;

    spin_lock_saved_state_t lock_state;
    spin_lock_irqsave(&imx_i2c_list_lock, lock_state);

    list_for_every_entry(&imx_i2c_list, state, struct imx_i2c_state, node) {
        if (state->bus_id == id) {
            dev = state->device;
            break;
        }
    }
    spin_unlock_irqrestore(&imx_i2c_list_lock, lock_state);

    return dev;
}

struct device *class_i2c_get_device_by_id(int bus_id)
{
    return imx_i2c_get_device_by_id(bus_id);
}

/* send and receive blocks of data */
status_t i2c_transmit(int bus, uint8_t address, const void *buf, size_t count)
{
    struct device *dev = imx_i2c_get_device_by_id(bus);
    if (dev == NULL)
        return ERR_NOT_FOUND;

    return imx_i2c_do_xfer_stream(dev, address, (uint8_t *) buf, count, 0);
}

status_t i2c_receive(int bus, uint8_t address, void *buf, size_t count)
{
    struct device *dev = imx_i2c_get_device_by_id(bus);
    if (dev == NULL)
        return ERR_NOT_FOUND;

    return imx_i2c_do_xfer_stream(dev, address, (uint8_t *) buf, count, 1);
}

status_t i2c_write_reg_bytes(
    int bus,
    uint8_t address,
    uint8_t reg,
    const uint8_t *value,
    size_t cnt)
{
    struct device *dev = imx_i2c_get_device_by_id(bus);

    printlk(LK_INFO, "%s:%d: bus: %d addr: %#x reg: %#x buf: %p len: %lu\n",
            __PRETTY_FUNCTION__, __LINE__, bus, address, reg, value, cnt);
    if (dev == NULL)
        return ERR_NOT_FOUND;

    return imx_i2c_do_xfer_reg(dev, address, reg, (uint8_t *) value, cnt, 0);
}

status_t i2c_read_reg_bytes(
    int bus,
    uint8_t address,
    uint8_t reg,
    uint8_t *value,
    size_t cnt)
{
    struct device *dev = imx_i2c_get_device_by_id(bus);

    printlk(LK_INFO, "%s:%d: bus: %d addr: %#x reg: %#x buf: %p len: %lu\n",
            __PRETTY_FUNCTION__, __LINE__, bus, address, reg, value, cnt);
    if (dev == NULL)
        return ERR_NOT_FOUND;

    return imx_i2c_do_xfer_reg(dev, address, reg, value, cnt, 1);
}

