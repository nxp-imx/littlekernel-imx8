/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017,2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _IMX_REGS_H_
#define _IMX_REGS_H_

#define MACH_IMX8

#define GIC_BASE_PHY  0x38800000
#define GIC_BASE_VIRT 0x38800000

#define SOC_REGS_PHY  0x30000000
#define SOC_REGS_VIRT 0xFFFFFFFF30000000LLU
#define SOC_REGS_SIZE 0x0A000000
#define SOC_REGS(x) (SOC_REGS_VIRT - SOC_REGS_PHY + x)

/* Registers for GIC */
#define MAX_INT 128
#define GICBASE(b) SOC_REGS(GIC_BASE_PHY)

#define PLATFORM_GICBASE SOC_REGS(GIC_BASE_PHY)
#define PLATFORM_GICR_STRIDE (0x20000)
#define PLATFORM_GICR_OFFSET (0x80000)
#define PLATFORM_GICD_OFFSET (0x0000)
#define PLATFORM_GIC_IPI_BASE (0)

#endif
