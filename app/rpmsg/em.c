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

#include <app.h>
#include <debug.h>
#include <err.h>
#include <string.h>
#include <platform.h>
#include <kernel/thread.h>
#include <event-manager.h>


#if WITH_LIB_CONSOLE
#include <lib/console.h>
#else
#error "em app needs a console"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static int cmd_em(int argc, const cmd_args *argv);

/*******************************************************************************
 * Variables
 ******************************************************************************/
STATIC_COMMAND_START
STATIC_COMMAND("em", "em commands", &cmd_em)
STATIC_COMMAND_END(em);


/*******************************************************************************
 * Code
 ******************************************************************************/

/*
 * Event-Manager tests commands
 */
static int cmd_em(int argc, const cmd_args *argv)
{
    if (argc < 2) {
        printlk(LK_ERR, "%s:%d: not enough arguments\n", __PRETTY_FUNCTION__,
                __LINE__);
usage:
        printlk(LK_NOTICE,
                "%s:%d: %s loop [N][period] : Send event every period (ms)\n",
                __PRETTY_FUNCTION__, __LINE__, argv[0].str);
        printlk(LK_NOTICE, "%s:%d: default period 1ms, loop 100\n",
                __PRETTY_FUNCTION__, __LINE__);
        return -1;
    }

    if (!strcmp(argv[1].str, "loop")) {
        uint32_t loop = 100, period = 1;

        if (argc >= 4)
            period = (uint32_t)argv[3].u;
        if (argc >= 3)
            loop = (uint32_t)argv[2].u;

        uint32_t i = 0;
        char message[128];
        lk_bigtime_t us;

        while (i < loop) {
            us = current_time_hires();

            snprintf(message, 128, "--> em time=%llu", us);
            ivshm_send_event(message);
            thread_sleep(period);

            i++;
        }

        printlk(LK_NOTICE, "%s:%d: %u event sent !\n", __PRETTY_FUNCTION__,
                __LINE__, loop);

        return 0;
    }

    printlk(LK_ERR, "%s:%d: Unrecognized subcommand '%s'\n",
            __PRETTY_FUNCTION__, __LINE__, argv[1].str);
    goto usage;
}

APP_START(em)
    .flags = 0,
APP_END


