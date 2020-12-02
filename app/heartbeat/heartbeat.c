/*
 * Copyright 2019-2021 NXP
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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <compiler.h>
#include <kernel/thread.h>
#include <kernel/event.h>
#include <kernel/mutex.h>
#include <platform.h>
#include <assert.h>
#include <debug.h>
#include <ivshmem-endpoint.h>
#include <cipc.h>
#include "watchdog.h"


//#define BUILTIN_TEST 1

#if WITH_LIB_CONSOLE

#include <lib/console.h>

#define MIN_KEEPALIVE_PERIOD_MS 100
#define MAX_KEEPALIVE_PERIOD_MS 1000
#define DEFAULT_KEEPALIVE_PERIOD_MS 500

static int cmd_heartbeat(int argc, const cmd_args *argv);

#define APP_DESC "Heartbeat Application"

STATIC_COMMAND_START
STATIC_COMMAND("heartbeat", APP_DESC, &cmd_heartbeat)
STATIC_COMMAND_END(heartbeat);

static unsigned int ka_period = DEFAULT_KEEPALIVE_PERIOD_MS;

static void usage (const char *app_name)
{
    printf("\nUsage:\n%s <period>\n", app_name);
    printf("\nWith:\n"
        "\t<period>    Specified keepalive period in ms. (min=%d, max=%d, default=%d)\n",
        MIN_KEEPALIVE_PERIOD_MS,
        MAX_KEEPALIVE_PERIOD_MS,
        DEFAULT_KEEPALIVE_PERIOD_MS);
}

#ifdef BUILTIN_TEST
static unsigned int nloops = 0;
#endif

static int heartbeat_thread_main(void *arg)
{
    size_t count;
    struct watchdog_message msg = {.magic=WD_MAGIC, .type=WD_MSG_TYPE_ALIVE, .subtype=0};

    for(;;) {
        thread_sleep(ka_period);

        count = cipc_write_buf(IVSHM_EP_ID_WATCHDOG, &msg, sizeof(struct watchdog_message));

        printlk(LK_DEBUG, "%lu bytes sent\n", count);

#ifdef BUILTIN_TEST
        if (nloops >= 60*(1000/ka_period)) { /* generates 5s blackout every 60s */
            printlk(LK_NOTICE, "%s:%d: heartbeat: blackout during 5s\n",
                    __PRETTY_FUNCTION__, __LINE__);
            thread_sleep(5000);
        } else
            nloops++;
#endif
    }

    return 0;
}

static int cmd_heartbeat (int argc, const cmd_args *argv)
{
    int retcode = 0;

    if (argc >= 2) {
        if (argc > 2) {
            usage(argv[0].str);
            retcode = -1;
            goto err_config;
        } else {
            ka_period = atoui(argv[1].str);
            if ((ka_period < MIN_KEEPALIVE_PERIOD_MS) || (ka_period > MAX_KEEPALIVE_PERIOD_MS)) {
                printf("keep-alive period %d is out range\n", ka_period);
                usage(argv[0].str);
                retcode = -1;
                goto err_config;
            }
        }
    }

    thread_detach_and_resume(thread_create("heartbeat_thread", &heartbeat_thread_main, NULL, DEFAULT_PRIORITY, DEFAULT_STACK_SIZE));

err_config:
    return retcode;
}

static void heartbeat_entry(const struct app_descriptor *app, void *args)
{
    heartbeat_thread_main(NULL);
}

APP_START(heartbeat)
    .entry = heartbeat_entry,
    .flags = 0,
APP_END

#endif // WITH_LIB_CONSOLE


