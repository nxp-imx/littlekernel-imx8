/*
 * The Clear BSD License
 * Copyright 2019 NXP
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

#ifndef __IMX_PLATFORM_GPIO
#define __IMX_PLATFORM_GPIO
#include <platform/interrupts.h>
#include <dev/gpio.h>
#include <sys/types.h>

#define IMX_MAX_GPIO_PER_BANK_MASK (0x1F)
#define IMX_GPIO_DEFAULT_VALUE_SHIFT (8)
#define GPIO(bank, id, value, flags)\
    (\
        ( ((flags) & 0xFFFF) << 16 \
        | (bank & 0xF) << 8) \
        | ((value & 1) << IMX_GPIO_DEFAULT_VALUE_SHIFT )\
        | (id & IMX_MAX_GPIO_PER_BANK_MASK)\
    )

#define GPIO_GET_FLAGS(x) ((x >> 16) & 0xFFFF)

#define MAX_GPIO_BANKS 16
#define MAX_GPIO_IRQ_PER_BANK 16

void register_gpio_int_handler(unsigned gpio, int_handler handler, void *args);
void unregister_gpio_int_handler(unsigned gpio);
void imx_gpio_config(unsigned gpio);

static inline uint8_t extract_bank(unsigned gpio_id)
{
    return ((gpio_id >> 8) & 0xF);
}

static inline uint8_t extract_bit (unsigned gpio_id)
{
    return (gpio_id & IMX_MAX_GPIO_PER_BANK_MASK);
}

static inline uint8_t extract_default (unsigned gpio_id)
{
    return ((gpio_id >> IMX_GPIO_DEFAULT_VALUE_SHIFT) & 1);
}

#endif
