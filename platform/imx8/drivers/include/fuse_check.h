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

#ifndef __FUSE_CHECK_H
#define __FUSE_CHECK_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <err.h>
#include <fsl_device_registers.h>


#define FUSE_DTS         0
#define FUSE_DATMOS      1
#define FUSE_AC4         2
#define FUSE_I3D         3
#define FUSE_DD          4
#define FUSE_DDP         5
#define FUSE_DOLBY_MS12  6

#ifndef BIT
    #define BIT(x) (1UL << (x))
#endif


/*
 * self-contained routines for dealing with fuse regs
 * Note i.MX8M mini, nano and plus share same reg addresses for:
 * - CCM CCGR[34]: OCOTP clock gating
 * - OCOTP HW_OCOTP_TESTER5: audio fuses
 * - CCM_ANALOG DIGPROG: CPU identification
 * Audio fuse bitmap assignment is also common, however logic for
 * decoder enablement is not:
 * - mini: fuse blown means decoder enabled
 * - nano, plus: fuse blown means decoder disabled
 */

static inline __ALWAYS_INLINE void __fuse_ocotp_clock_enable(int enable)
{
    CCM_Type *ccm = CCM;
    uint32_t val;

    /* OCOTP clock gating mapped onto CCGR34 */
    val = ccm->CCGR[34].CCGR;
    if (enable)
        val |= CCM_CCGR_SET_SETTING0_MASK;
    else
        val &= ~CCM_CCGR_SET_SETTING0_MASK;

    ccm->CCGR[34].CCGR = val;
 }

static inline __ALWAYS_INLINE uint32_t __fuse_ocotp_audio_raw(void)
{
    OCOTP_Type *ocotp = OCOTP;

    __fuse_ocotp_clock_enable(1);
    return(ocotp->HW_OCOTP_TESTER5);
}

static inline __ALWAYS_INLINE uint32_t __fuse_digprog_raw(void)
{
    CCM_ANALOG_Type *ccma = CCM_ANALOG;

    return(ccma->DIGPROG);
}

static inline __ALWAYS_INLINE uint32_t __fuse_digprog_soc(void)
{
    uint32_t val = __fuse_digprog_raw();
    /* extract major lower */
    val = ((val & CCM_ANALOG_DIGPROG_DIGPROG_MAJOR_LOWER_MASK)
                >> CCM_ANALOG_DIGPROG_DIGPROG_MAJOR_LOWER_SHIFT);
    /* Bits [0..3] indicate soc */
    val &= 0xf;

    return val;
}

static inline __ALWAYS_INLINE int __fuse_is_enabled(int bit)
{
    uint32_t val, soc;
    val    = __fuse_ocotp_audio_raw();
    soc    = __fuse_digprog_soc();

    switch(soc) {
        case 1: /* mini */
            break;
        case 2: /* nano */
        case 3: /* plus */
            /* inverted logic: fuse blown when decoder disabled */
            val = ~val;
            break;
        default:
            /* unkown SOC - invalidate fuse read */
            val = 0;
            break;
    }

    return !!(val & BIT(bit));
}

int fuse_check(void);

#endif  /* __FUSE_CHECK_H */
