/*
 * Copyright (c) 2017 Google Inc. All rights reserved
 * Copyright 2019-2020 NXP
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

#include <kernel/thread.h>
#include <kernel/mutex.h>
#include <platform/debug.h>
#include <dev/uart.h>
#include <err.h>

mutex_t dputs_lock = MUTEX_INITIAL_VALUE(dputs_lock);
extern bool uart_disabled;

void platform_dputs_thread(const char *str, size_t len)
{
    if (uart_disabled)
        return;

    mutex_acquire(&dputs_lock);
    uart_puts(str, len, true, true);
    mutex_release(&dputs_lock);
}

void platform_dputs_irq(const char *str, size_t len)
{
    if (uart_disabled)
        return;

    uart_puts(str, len, false, true);
}
void platform_dputc(char c)
{
    if (uart_disabled)
        return;

    uart_putc(c);
}

void platform_pputc(char c)
{
    if (uart_disabled)
        return;

    if (c == '\n')
        uart_pputc('\r');
    uart_pputc(c);
}

void platform_dputs(const char *str, size_t len)
{
    if (uart_disabled)
        return;

    uart_puts(str, len, false, true);
}

int platform_dgetc(char *c, bool wait)
{
    if (uart_disabled)
        return ERR_NOT_SUPPORTED;

    int res;
    res = uart_getc(wait);

    if (res < 0)
        return res;

    *c = (char) res;
    return 0;

}

int platform_pgetc(char *c, bool wait)
{
    if (uart_disabled)
        return ERR_NOT_SUPPORTED;

    int res;
    res = uart_pgetc();

    if (res < 0)
        return res;

    *c = (char) res;
    return 0;
}

bool platform_is_console_enabled(void)
{
    return !uart_disabled;
}
