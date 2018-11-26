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
#include <debug.h>
#include <err.h>
#include <compiler.h>
#include <arch/ops.h>
#include <arch/mp.h>
#include <platform.h>
#include <platform/debug.h>
#include <stdio.h>
#include <lib/console.h>
#include <dev/uart.h>
#include <pdev/interrupt.h>
#if WITH_LIB_DEBUGLOG
#include <lib/debuglog.h>
#endif

void platform_halt(platform_halt_action suggested_action,
                          platform_halt_reason reason)
{
    if (reason == HALT_REASON_SW_PANIC) {
        unsigned target = ~(1U << arch_curr_cpu_num());
        arch_mp_send_ipi(target, MP_IPI_HALT);
        shutdown_interrupts();
        uart_start_panic();
        arch_disable_ints();
#if WITH_LIB_DEBUGLOG
        dlog_flush();
#endif
#if ENABLE_PANIC_SHELL
        dprintf(ALWAYS, "CRASH: starting debug shell... (reason = %d)\n", reason);
        panic_shell_start();
#endif
    }

    dprintf(ALWAYS, "CRASH: action: %d, reason = %d\n", suggested_action, reason);
    switch (suggested_action) {
        case HALT_ACTION_SHUTDOWN:
        case HALT_ACTION_HALT:
            dprintf(ALWAYS, "HALT: spinning forever... (reason = %d)\n", reason);
            unsigned target = ~(1U << arch_curr_cpu_num());
            arch_mp_send_ipi(target, MP_IPI_HALT);
            arch_disable_ints();
            for (;;)
                arch_idle();
            break;
        case HALT_ACTION_REBOOT:
            dprintf(INFO, "REBOOT\n");
            dprintf(ALWAYS, "REBOOT: Implementation to be done\n");
            arch_disable_ints();
            for (;;) {
            }
            break;
    }

    dprintf(ALWAYS, "HALT: spinning forever... (reason = %d)\n", reason);
    arch_disable_ints();
    for (;;);

}
