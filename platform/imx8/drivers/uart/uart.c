// Copyright 2018 The Fuchsia Authors
// Copyright 2019-2020 NXP
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include <err.h>
#include <reg.h>
#include <stdio.h>
#include <lib/cbuf.h>
#include <kernel/thread.h>
#include <dev/interrupt.h>
#include <dev/uart.h>
#include <platform/debug.h>
#include <pdev/uart.h>
#include <imx-regs.h>

#ifdef WITH_LIB_APPARGS
#include <lib/appargs.h>
#endif

#if WITH_LIB_DEBUGLOG
#include <lib/debuglog.h>
#else
void dlog_panic(void) {}
bool dlog_bypass(void)
{
#if ENABLE_KERNEL_LL_DEBUG
    return true;
#else
    return false;
#endif
}
#endif


/* Registers */
#define MX8_URXD                    (0x00)
#define MX8_UTXD                    (0x40)
#define MX8_UCR1                    (0x80)
#define MX8_UCR2                    (0x84)
#define MX8_UCR3                    (0x88)
#define MX8_UCR4                    (0x8C)
#define MX8_UFCR                    (0x90)
#define MX8_USR1                    (0x94)
#define MX8_USR2                    (0x98)
#define MX8_UBIR                    (0xA4)
#define MX8_UBMR                    (0xA8)
#define MX8_ONEMS                   (0xB0)
#define MX8_UTS                     (0xB4)



/* UCR1 Bit Definition */
#define UCR1_TRDYEN                 (1 << 13)
#define UCR1_RRDYEN                 (1 << 9)
#define UCR1_UARTEN                 (1 << 0)

/* UCR2 Bit Definition */
#define UCR2_IRTS                   (1 << 14)
#define UCR2_WS                     (1 << 5)
#define UCR2_TXEN                   (1 << 2)
#define UCR2_RXEN                   (1 << 1)
#define UCR2_SRST                   (1 << 0)

/* UCR3 Bit Definition */
#define UCR3_RXDMUXSEL              (1 << 2)

/* UFCR Bit Definition */
#define UFCR_TXTL(x)                (x << 10)
#define UFCR_RXTL(x)                (x << 0)
#define UFCR_MASK                   (0x3f)

#define UFCR_RFDIV_MASK                     (0x380U)
#define UFCR_RFDIV_SHIFT                    (7U)
#define UFCR_RFDIV(x)                       (((uint32_t)(((uint32_t)(x)) << UFCR_RFDIV_SHIFT)) & UFCR_RFDIV_MASK)

/* USR1 Bit Definition */
#define USR1_TRDY                   (1 << 13)
#define USR1_RRDY                   (1 << 9)

/* USR2 Bit Definition */
#define USR2_TXFE                   (1 << 14)

/* UTS Bit Definition */
#define UTS_TXEMPTY                 (1 << 6)
#define UTS_RXEMPTY                 (1 << 5)
#define UTS_TXFULL                  (1 << 4)
#define UTS_RXFULL                  (1 << 3)

/* UBIR - UART BRM Incremental Register */
#define UBIR_INC_MASK               (0xFFFFu)

/* UBMR - UART BRM Modulator Register */
#define UART_UBMR_MOD_MASK                       (0xFFFFU)

/* UBRC - UART Baud Rate Count Register */
#define UART_UBRC_BCNT_MASK                      (0xFFFFU)

/* ONEMS - UART One Millisecond Register */
#define UART_ONEMS_ONEMS_MASK                    (0xFFFFFFU)

#define RXBUF_SIZE 64
#define TXBUF_SIZE 512

#ifndef UART_BASE
#define  UART_BASE  (SOC_REGS_VIRT + (CONFIG_CONSOLE_TTY_BASE - SOC_REGS_PHY))
#endif

#ifndef UART_IRQ
#define UART_IRQ 59
#endif
#define IMX_UART_TXTL_THRESHOLD 2
#define IMX_UART_TX_FIFO_SIZE 32

#define IMX_UART_USE_TXBUFF
// values read from zbi
static bool initialized = false;
static bool imx_tx_buffered = false;
static vaddr_t uart_base = 0;
static uint32_t uart_irq = 0;
static cbuf_t uart_rx_buf;
static cbuf_t uart_tx_buf;

static bool uart_tx_irq_enabled = false;
static event_t uart_dputc_event = EVENT_INITIAL_VALUE(uart_dputc_event,
                                  true,
                                  EVENT_FLAG_AUTOUNSIGNAL);

static spin_lock_t uart_spinlock = SPIN_LOCK_INITIAL_VALUE;

#define UARTREG(reg)          (*(volatile uint32_t*)((uart_base)  + (reg)))

static void imx_uart_txirq_enable(void)
{
    unsigned regVal;
    regVal = UARTREG(MX8_UCR1);
    regVal |= UCR1_TRDYEN;
    UARTREG(MX8_UCR1) = regVal;
}

static void imx_uart_txirq_disable(void)
{
    unsigned regVal;
    regVal = UARTREG(MX8_UCR1);
    regVal &= ~UCR1_TRDYEN;
    UARTREG(MX8_UCR1) = regVal;
}


static enum handler_return uart_irq_handler(void *arg)
{
    /* read interrupt status and mask */
    while ((UARTREG(MX8_USR1) & USR1_RRDY)) {
        if (cbuf_space_avail(&uart_rx_buf) == 0) {
            /* Empty the buffer */
            cbuf_read(&uart_rx_buf, NULL, RXBUF_SIZE, false);
        }
        char c = UARTREG(MX8_URXD) & 0xFF;
        cbuf_write_char(&uart_rx_buf, c, false);
    }

    /* Signal if anyone is waiting to TX */
    if (UARTREG(MX8_UCR1) & UCR1_TRDYEN) {
        spin_lock(&uart_spinlock);
        if (imx_tx_buffered) {
            if (UARTREG(MX8_USR1) & USR1_TRDY) {
                char buf[32];
                unsigned i = 0;
                if (cbuf_space_avail(&uart_tx_buf) == 0)
                    event_signal(&uart_dputc_event, false);

                size_t read = cbuf_read(&uart_tx_buf,
                            buf,
                            IMX_UART_TX_FIFO_SIZE - IMX_UART_TXTL_THRESHOLD,
                            false);
                while(i < read)
                    UARTREG(MX8_UTXD) = buf[i++];

                if (cbuf_space_used(&uart_tx_buf) == 0)
                    imx_uart_txirq_disable();
            }
        } else {
            if (!(UARTREG(MX8_UTS) & UTS_TXFULL)) {
                // signal
                event_signal(&uart_dputc_event, false);
                imx_uart_txirq_disable();
            }
        }
        spin_unlock(&uart_spinlock);
    }

    return INT_RESCHEDULE;
}



/* panic-time getc/putc */
static int imx_uart_pputc(char c)
{
    if (!uart_base) {
        return -1;
    }

    /* spin while fifo is full */
    while (UARTREG(MX8_UTS) & UTS_TXFULL)
        ;
    UARTREG(MX8_UTXD) = c;

    return 1;
}

static int imx_uart_pgetc(void)
{
    if (!uart_base) {
        return ERR_NOT_SUPPORTED;
    }

    if ((UARTREG(MX8_UTS) & UTS_RXEMPTY)) {
        return ERR_INTERNAL;
    }

    return UARTREG(MX8_URXD);
}

static int imx_uart_getc(bool wait)
{
    if (!uart_base) {
        return ERR_NOT_SUPPORTED;
    }

    if (initialized) {
        char c;
        if (cbuf_read_char(&uart_rx_buf, &c, wait) == 1) {
            return c;
        }
        return ERR_INTERNAL;
    }
    else {
        // Interrupts are not enabled yet. Use panic calls for now
        return imx_uart_pgetc();
    }

}
static void imx_dputs_buffered(const char *str, size_t len,
                      bool block, bool map_NL)
{
    spin_lock_saved_state_t state;
    bool copied_CR = false;

    if (!uart_base) {
        return;
    }

    spin_lock_irqsave(&uart_spinlock, state);
    ASSERT(uart_tx_irq_enabled);

    bool empty = !!(cbuf_space_used(&uart_tx_buf) == 0);
    size_t ret;

    while (len > 0) {
        char c;
        while (cbuf_space_avail(&uart_tx_buf) == 0) {
            spin_unlock_irqrestore(&uart_spinlock, state);
            event_wait(&uart_dputc_event);
            spin_lock_irqsave(&uart_spinlock, state);
        }

        if (*str == '\n' && map_NL && !copied_CR) {
            copied_CR = true;
            c = '\r';
        } else {
            copied_CR = false;
            c = *str++;
            len--;
        }

        ret = cbuf_write_char(&uart_tx_buf, c, false);
        ASSERT(ret);

        if (empty && uart_tx_irq_enabled) {
            imx_uart_txirq_enable();
            empty = false;
        }
    }

    spin_unlock_irqrestore(&uart_spinlock, state);
}


static void imx_dputs_unbuffered(const char *str, size_t len,
                      bool block, bool map_NL)
{
    spin_lock_saved_state_t state;
    bool copied_CR = false;

    if (!uart_base) {
        return;
    }

    spin_lock_irqsave(&uart_spinlock, state);
    if (!uart_tx_irq_enabled) {
        block = false;
    }

    while (len > 0) {
        char c;
        // is FIFO full?
        while ((UARTREG(MX8_UTS) & UTS_TXFULL)) {
            if (block) {
                imx_uart_txirq_enable();
                spin_unlock_irqrestore(&uart_spinlock, state);
                event_wait(&uart_dputc_event);
            } else {
                spin_unlock_irqrestore(&uart_spinlock, state);
                arch_spinloop_pause();
            }
            spin_lock_irqsave(&uart_spinlock, state);
        }

        if (*str == '\n' && map_NL && !copied_CR) {
            copied_CR = true;
            c = '\r';
        } else {
            copied_CR = false;
            c = *str++;
            len--;
        }
        imx_uart_pputc(c);
    }

    spin_unlock_irqrestore(&uart_spinlock, state);
}

static void imx_dputs(const char *str, size_t len,
                      bool block, bool map_NL)
{
    if (imx_tx_buffered && uart_tx_irq_enabled && block)
        imx_dputs_buffered(str, len, block, map_NL);
    else
        imx_dputs_unbuffered(str, len, block, map_NL);
}

static void imx_start_panic(void)
{
    spin_lock_saved_state_t state;
    bool needs_flushing;

    spin_lock_irqsave(&uart_spinlock, state);

    dlog_panic();

    imx_uart_txirq_disable();
    uart_tx_irq_enabled = false;
    needs_flushing = imx_tx_buffered;
    imx_tx_buffered = false;

    spin_unlock_irqrestore(&uart_spinlock, state);

    if (needs_flushing) {
        char c;
        while(cbuf_read_char(&uart_tx_buf, &c, false))
            imx_uart_pputc(c);
    }
}

static const struct pdev_uart_ops uart_ops = {
    .getc = imx_uart_getc,
    .pputc = imx_uart_pputc,
    .pgetc = imx_uart_pgetc,
    .start_panic = imx_start_panic,
    .dputs = imx_dputs,
};

/* This UART instantiation uses a slightly different baud rate calculation.
 * Baud Rate = Ref Freq / (16 * (UBMR + 1)/(UBIR+1)).
 * To get a baud rate, three register need to be writen, UFCR,UBMR and UBIR
 * At first, find the approximately maximum divisor of src_Clock and baudRate_Bps.
 * If the numerator and denominator are larger then register maximum value(0xFFFF),
 * both of numerator and denominator will be divided by the same value, which
 * will ensure numerator and denominator range from 0~maximum value(0xFFFF).
 * Then calculate UFCR and UBIR value from numerator, and get UBMR value from denominator.
 */
int imx_uart_set_baudrate(uint32_t baudRate_Bps, uint32_t srcClock_Hz)
{

    uint32_t numerator = 0u;
    uint32_t denominator = 0U;
    uint32_t divisor = 0U;
    uint32_t refFreqDiv = 0U;
    uint32_t divider = 1U;
    uint64_t baudDiff = 0U;
    uint64_t tempNumerator = 0U;
    uint32_t tempDenominator = 0u;

    /* get the approximately maximum divisor */
    numerator = srcClock_Hz;
    denominator = baudRate_Bps << 4;
    divisor = 1;

    while (denominator != 0) {
        divisor = denominator;
        denominator = numerator % denominator;
        numerator = divisor;
    }

    numerator = srcClock_Hz / divisor;
    denominator = (baudRate_Bps << 4) / divisor;

    /* numerator ranges from 1 ~ 7 * 64k */
    /* denominator ranges from 1 ~ 64k */
    if ((numerator > (UBIR_INC_MASK * 7)) || (denominator > UBIR_INC_MASK)) {
        uint32_t m = (numerator - 1) / (UBIR_INC_MASK * 7) + 1;
        uint32_t n = (denominator - 1) / UBIR_INC_MASK + 1;
        uint32_t max = m > n ? m : n;
        numerator /= max;
        denominator /= max;
        if (0 == numerator) {
            numerator = 1;
        }
        if (0 == denominator) {
            denominator = 1;
        }
    }
    divider = (numerator - 1) / UBIR_INC_MASK + 1;

    switch (divider) {
        case 1:
            refFreqDiv = 0x05;
            break;
        case 2:
            refFreqDiv = 0x04;
            break;
        case 3:
            refFreqDiv = 0x03;
            break;
        case 4:
            refFreqDiv = 0x02;
            break;
        case 5:
            refFreqDiv = 0x01;
            break;
        case 6:
            refFreqDiv = 0x00;
            break;
        case 7:
            refFreqDiv = 0x06;
            break;
        default:
            refFreqDiv = 0x05;
            break;
    }
    /* Compare the difference between baudRate_Bps and calculated baud rate.
     * Baud Rate = Ref Freq / (16 * (UBMR + 1)/(UBIR+1)).
     * baudDiff = (srcClock_Hz/divider)/( 16 * ((numerator / divider)/ denominator).
     */
    tempNumerator = srcClock_Hz;
    tempDenominator = (numerator << 4);
    divisor = 1;
    /* get the approximately maximum divisor */
    while (tempDenominator != 0) {
        divisor = tempDenominator;
        tempDenominator = tempNumerator % tempDenominator;
        tempNumerator = divisor;
    }
    tempNumerator = srcClock_Hz / divisor;
    tempDenominator = (numerator << 4) / divisor;
    baudDiff = (tempNumerator * denominator) / tempDenominator;
    baudDiff = (baudDiff >= baudRate_Bps) ? (baudDiff - baudRate_Bps) : (baudRate_Bps - baudDiff);

    if (baudDiff < (baudRate_Bps / 100) * 3) {
        UARTREG(MX8_UFCR) &= ~UFCR_RFDIV_MASK;
        UARTREG(MX8_UFCR) |= UFCR_RFDIV(refFreqDiv);
        UARTREG(MX8_UBIR) = denominator - 1;
        UARTREG(MX8_UBMR) = numerator / divider - 1;
        UARTREG(MX8_ONEMS) = srcClock_Hz / (1000 * divider);

        return 0;
    }
    else {
        return -1;
    }
}


static void imx_uart_init(int early)
{
    uint32_t regVal;

    if (!early) {
        // create circular buffer to hold received data
        cbuf_initialize(&uart_rx_buf, RXBUF_SIZE);
        // create circular buffer to hold received data
        cbuf_initialize(&uart_tx_buf, TXBUF_SIZE);
        // register uart irq
        register_int_handler(uart_irq, &uart_irq_handler, NULL);
    }

    // set rx fifo threshold to 1 character
    regVal = UARTREG(MX8_UFCR);
    regVal &= ~UFCR_RXTL(UFCR_MASK);
    regVal &= ~UFCR_TXTL(UFCR_MASK);
    regVal |= UFCR_RXTL(1);
    regVal |= UFCR_TXTL(IMX_UART_TXTL_THRESHOLD);
    UARTREG(MX8_UFCR) = regVal;

    // enable rx interrupt
    regVal = UARTREG(MX8_UCR1);
    if (!early) {
        regVal |= UCR1_RRDYEN;
        if (dlog_bypass() == false) {
            regVal |= UCR1_TRDYEN;
        }
    }
    regVal |= UCR1_UARTEN;
    UARTREG(MX8_UCR1) = regVal;

    // enable rx and tx transmisster
    regVal = UARTREG(MX8_UCR2);
    regVal |= UCR2_RXEN | UCR2_TXEN;
    // 8bit Word Size
    regVal |= UCR2_WS;
    // Ignore RTS Pin
    regVal |= UCR2_IRTS;
    UARTREG(MX8_UCR2) = regVal;

    regVal = UARTREG(MX8_UCR3);
    regVal |= UCR3_RXDMUXSEL;
    UARTREG(MX8_UCR3) = regVal;

    if (!early) {
        if (dlog_bypass() == true) {
            uart_tx_irq_enabled = false;
        } else {
            /* start up tx driven output */
            printlk(LK_NOTICE, "%s:%d: UART: started IRQ driven TX\n",
                    __PRETTY_FUNCTION__, __LINE__);
            uart_tx_irq_enabled = true;
        }

        initialized = true;
#ifdef IMX_UART_USE_TXBUFF
        imx_tx_buffered = true;
#endif
        smp_wmb();

        printlk(LK_NOTICE, "%s:%d: Using TX %sbuffered mode\n",
                __PRETTY_FUNCTION__, __LINE__, imx_tx_buffered ? "":"un-");
        // enable interrupts
        unmask_interrupt(uart_irq);
    }
}

bool uart_disabled;

void uart_init(void)
{
    if (uart_disabled)
        return;
    imx_uart_init(0);
}

void uart_init_early(void)
{
#ifdef WITH_LIB_APPARGS
#define MAX_DEVICE_CFG_CLKS 4
#define MAX_DEVICE_CFG_PINS 4 /* RX, TX, CTS, RTS */
    uart_disabled = of_get_bool(0, "uart-disabled");
    if (uart_disabled)
        return;

    of_get_console_irq_and_base(&uart_base, &uart_irq);
    struct device_cfg_clk cfgs[MAX_DEVICE_CFG_CLKS];
    struct device_cfg_pin pins[MAX_DEVICE_CFG_PINS];
    unsigned i;

    unsigned count = of_get_console_clocks(cfgs, MAX_DEVICE_CFG_CLKS);
    for (i = 0; i < count; i++)
        devcfg_set_clock(&cfgs[i]);

    count = of_get_console_pins(pins, MAX_DEVICE_CFG_PINS);
    for (i = 0; i < count; i++)
        devcfg_set_pin(&pins[i]);

#endif
    if (uart_base == 0)
        uart_base = UART_BASE;

    ASSERT(uart_base);

    if (uart_irq == 0)
        uart_irq = UART_IRQ;


    imx_uart_init(1);
    pdev_register_uart(&uart_ops);

    imx_uart_set_baudrate(115200, cfgs[0].rate);
}

