/*
 * The Clear BSD License
 * Copyright 2019-2021 NXP
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <err.h>
#include <app.h>
#include <kernel/thread.h>
#include "fuse_check.h"


#if WITH_LIB_CONSOLE
#include <lib/console.h>

static int cmd_fuse_read(int argc, const cmd_args *argv);

#define APP_DESC "Fuse Read Application"

STATIC_COMMAND_START
STATIC_COMMAND("fuse_read", APP_DESC, &cmd_fuse_read)
STATIC_COMMAND_END(fuse_read);

int cmd_fuse_read(int argc, const cmd_args *argv)
{
    const char *check = "n";

#ifdef ENABLE_FUSE_CHECK
    check = "y";
#endif

    printf("FUSE read value              : 0x%02X\n", __fuse_ocotp_audio_raw());
    printf("DIGPROG value                : 0x%02X\n", __fuse_digprog_raw());
    printf("FUSE check enabled           : %s\n", check);

    return NO_ERROR;
}
#endif

static int fuse_check_thread_main(void *arg)
{
    for(;;) {
        thread_sleep(1000);
        fuse_check();
    }

    return 0;
}

static void fuse_check_entry(const struct app_descriptor *app, void *args)
{
    fuse_check_thread_main(NULL);
}

APP_START(fuse_check)
    .entry = fuse_check_entry,
    .flags = 0,
APP_END
