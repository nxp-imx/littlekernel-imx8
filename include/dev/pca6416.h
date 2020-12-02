/*
 * The Clear BSD License
 * Copyright 2018 NXP
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

#ifndef _PCA6416_H_
#define _PCA6416_H_
#include <platform/interrupts.h>
#include "err.h"

struct pca6416_config {
    /* i2c bus id */
    int bus_id;
    /* 7bit slave address */
    unsigned char address_7bit;
    /* SoC Interrupt id*/
    int irq;
    /* pin callback */
    status_t (*set_pinmux_state)(int);
    /* power callback */
    status_t (*set_power_state)(int);
    /* GPIO IRQ - to be handled through gpio framework */
#define IMX_PCA_GPIO_IRQ_INIT   0
#define IMX_PCA_GPIO_IRQ_UNMASK 1
#define IMX_PCA_GPIO_IRQ_MASK   2
#define IMX_PCA_GPIO_IRQ_CLEAR  3
    void (*gpio_irq_action)(int);

};
typedef void *pca6416_handle_t;

/* note: In all port API calls, the PCA6416A hardware port0 is mapped to bits[7:0], port1 is mapped to bits[15:8] */

/* Initialize PCA6416A driver instance */
pca6416_handle_t pca6416_probe(const struct pca6416_config *config);
int pca6416_register_int(pca6416_handle_t, int, int_handler, void *);
int pca6416_unregister_int(pca6416_handle_t, int);

int pca6416_set_as_input(pca6416_handle_t dev, uint32_t input_pins);
int pca6416_set_as_output(pca6416_handle_t dev, uint32_t output_pins);
int pca6416_get_direction(pca6416_handle_t dev, uint32_t *input_pins);

int pca6416_set_polarity_normal(pca6416_handle_t dev, uint32_t normal_pins);
int pca6416_set_polarity_inverted(pca6416_handle_t dev, uint32_t inverted_pins);
int pca6416_get_polarity(pca6416_handle_t dev, uint32_t *inverted_pins);

int pca6416_write_output_pins(pca6416_handle_t dev, uint32_t output, uint32_t pin_mask);
int pca6416_read_output_pins(pca6416_handle_t dev, uint32_t *output);
int pca6416_read_input_pins(pca6416_handle_t dev, uint32_t *input);

extern struct pca6416 *g_pca6416_device;

#endif //_ADV7627_H_
