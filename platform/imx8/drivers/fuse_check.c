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

#include <platform.h>
#include <lib/dpc.h>
#include "fsl_clock.h"
#include "fuse_check.h"
#include "debug.h"


static void dpc_platform_halt_cb(void *arg)
{
    platform_halt(HALT_ACTION_SHUTDOWN, HALT_REASON_OTHER_HW);
}

int fuse_check(void)
{
#ifdef ENABLE_FUSE_CHECK
    if (!__fuse_is_enabled(FUSE_I3D)) {
        printlk(LK_INFO, "%s:%d: IMM3D: Fuse check failed. Abort.\n", __PRETTY_FUNCTION__, __LINE__);
        dpc_queue(dpc_platform_halt_cb, NULL, DPC_FLAG_NORESCHED);
        return ERR_NOT_VALID;
    }
#else
    printlk(LK_INFO, "%s:%d: IMM3D: Fuse Value: disabled\n",
            __PRETTY_FUNCTION__, __LINE__);
#endif

    return NO_ERROR;
}
