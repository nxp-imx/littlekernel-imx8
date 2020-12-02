/*
 * Copyright 2019-2020 NXP
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

#ifndef __IVSHM_H
#define __IVSHM_H
#ifdef LK
#include <sys/types.h>
#include <platform/interrupts.h>
#endif

#define IVSHM_V2_REG_ID            0
#define IVSHM_V2_REG_MAX_PEERS     4
#define IVSHM_V2_REG_INT_CTRL      8
#define IVSHM_V2_REG_DBELL         12
#define IVSHM_V2_REG_STATE         16

#define IVSHM_V1_REG_INTX_CTRL   0
#define IVSHM_V1_REG_IVPOS   8
#define IVSHM_V1_REG_DBELL   12
#define IVSHM_V1_REG_LSTATE  16
#define IVSHM_V1_REG_RSTATE  20

#ifdef LK

struct ivshm_dev_data {
    u8 revision;
    u16 bdf;
    u16 irq;
    void *registers;
    u32 *state_table;
    u32 state_table_sz;
    u32 *rw_section;
    u64 rw_section_sz;
    u32 *in_sections;
    u32 *out_section;
    u64 out_section_sz;
    u32 *msix_table;
    u32 id;
    int msix_cap;
    int_handler handler;
    void *handler_arg;
};



struct ivshm_dev_data *ivshm_register(int, int_handler, void *);
struct ivshm_dev_data *ivshm_get_device(int);
#endif

#endif
