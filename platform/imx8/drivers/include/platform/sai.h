/*
 * Copyright 2018 NXP
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
#ifndef __IMX_PLATFORM_SAI
#define __IMX_PLATFORM_SAI

struct imx_sai_config {
    /* BUS id */
    int bus_id;
    /* IRQ number */
    unsigned irq;
    /* IP clk rate */
    unsigned clk_rate;
    /* IP IO phys address */
    paddr_t phys_io;
    /* IP IO virtual address if mapped by caller */
    vaddr_t virt_io;
    /* pin callback */
    status_t (*set_pinmux_state)(int);
    /* power callback */
    status_t (*set_power_state)(int);
};

/* defines for sai_power_state() function */
#define SAI_POWER_ENABLE_MASK  (0x01)
#define SAI_POWER_DISABLE      (0x00)
#define SAI_POWER_ENABLE       (0x01)
#define SAI_SAMPLERATE_MASK    (0x02)
#define SAI_SAMPLERATE_48kHz   (0x00)
#define SAI_SAMPLERATE_44_1kHz (0x02)

/* defines for sai_pinmux_state() */
#define SAI_PINMUX_ENABLE      1

#endif /* __IMX_PLATFORM_SAI */
