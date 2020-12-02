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
#include <stdlib.h>
#include <kernel/thread.h>

#include <assert.h>

#if WITH_LIB_CONSOLE
#include <lib/console.h>
#else
#error "rpc app needs a console"
#endif

#include "ivshmem-rpc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define APP_BUF_SIZE (64 << 10)


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static int cmd_rpc(int argc, const cmd_args *argv);

/*******************************************************************************
 * Variables
 ******************************************************************************/
STATIC_COMMAND_START
STATIC_COMMAND("rpc", "rpc commands", &cmd_rpc)
STATIC_COMMAND_END(rpc);


/* Test app RPC possible commands */
#define TEST_APP_ADDITION_ID    IVSHM_RPC_ID(8, 4, 0x01)
#define TEST_APP_TWICE_ID       IVSHM_RPC_ID(4, 4, 0x02)
#define TEST_APP_CALLME_ID      IVSHM_RPC_ID(0, 0, 0x03)

/* Client commands */
#define TEST_APP_CLIENT_VERSION IVSHM_RPC_ID(0, 32, 0x01)

IVSHM_RPC_CALLBACK(TEST_APP_CLIENT_VERSION, test_version);

static struct rpc_client_callback *rpc_app_callback_all[] = {
    &rpc_cb_test_version,
    NULL,
};

/*******************************************************************************
 * Code
 ******************************************************************************/
static int test_version(struct ivshm_rpc_service *s,
                               struct rpc_client_callback *cb, void *payload)
{
    char version[32] = "RPC version 1.0";

    return ivshm_rpc_reply(s, cb->id, &version, strlen(version) + 1);
}

/* RPMSG command
 */
static int cmd_rpc(int argc, const cmd_args *argv)
{
    struct ivshm_rpc_service *service = ivshm_get_rpc_service(6);

    ASSERT(service);

    ivshm_rpc_register_client(service, rpc_app_callback_all);

    if (argc < 2) {
        printf("not enough arguments\n");
usage:
        printf("%s add <x> <y> : \n", argv[0].str);
        printf("%s twice <x> : \n", argv[0].str);
        printf("%s callme : \n", argv[0].str);

        return -1;
    }

    /* Add command */
    if (!strcmp(argv[1].str, "add")) {
        if (argc < 4) {
            printf("Invalid command\n");
            goto usage;
        }

        struct param {
            unsigned op1;
            unsigned op2;
        } param = { argv[2].u, argv[3].u };

        unsigned result;
        size_t out_sz = sizeof(unsigned);
        ssize_t ret;

        ret = ivshm_rpc_call(service, TEST_APP_ADDITION_ID,
                             &param, sizeof(struct param), &result, &out_sz);

        if (ret == 0)
            printf("result %u\n", result);

        return ret;
    }

    /* Twice command */
    if (!strcmp(argv[1].str, "twice")) {
        if (argc < 3) {
            printf("Invalid command\n");
            goto usage;
        }

        unsigned param = argv[2].u;
        size_t out_sz = sizeof(unsigned);
        unsigned result;
        ssize_t ret;

        ret = ivshm_rpc_call(service, TEST_APP_TWICE_ID,
                             &param, sizeof(unsigned), &result, &out_sz);

        if (ret == 0)
            printf("result %u\n", result);

        return ret;
    }

    /* Callme command */
    if (!strcmp(argv[1].str, "callme")) {
        if (argc < 2) {
            printf("Invalid command\n");
            goto usage;
        }

        ssize_t ret;

        ret = ivshm_rpc_call(service, TEST_APP_CALLME_ID,
                             NULL, 0, NULL, NULL);

        return ret;
    }
    printf("unrecognized subcommand\n");
    goto usage;
}

APP_START(rpc)
    .flags = 0,
APP_END

