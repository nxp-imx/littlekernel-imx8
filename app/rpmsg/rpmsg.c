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
#include <ivshmem-endpoint.h>
#include <ivshmem-binary.h>
#include <cipc.h>

#include <assert.h>

#include "md5.h"

#if WITH_LIB_CONSOLE
#include <lib/console.h>
#else
#error "rpmsg app needs a console"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define APP_BUF_SIZE (16 << 20) /* 16 MBytes */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static int cmd_rpmsg(int argc, const cmd_args *argv);

/*******************************************************************************
 * Variables
 ******************************************************************************/
STATIC_COMMAND_START
STATIC_COMMAND("rpmsg", "rpmsg commands", &cmd_rpmsg)
STATIC_COMMAND_END(rpmsg);


/*******************************************************************************
 * Code
 ******************************************************************************/

static void *app_buf = NULL;

/* RPMSG command
 */
static int cmd_rpmsg(int argc, const cmd_args *argv)
{
    if (argc < 2) {
        printlk(LK_ERR, "%s:%d: not enough arguments\n", __PRETTY_FUNCTION__,
                __LINE__);
usage:
        printlk(LK_NOTICE, "%s:%d: %s list : List available binary endpoints\n",
                __PRETTY_FUNCTION__, __LINE__, argv[0].str);
        printlk(LK_NOTICE,
                "%s:%d: %s send (id) (nb) : Send nb bytes from test buffer to binary pipe id\n",
                __PRETTY_FUNCTION__, __LINE__, argv[0].str);
        printlk(LK_NOTICE,
                "%s:%d: %s read (id) (nb) : Read nb bytes from binary path id, copy into test \
                buffer\n",
                __PRETTY_FUNCTION__, __LINE__, argv[0].str);
        printlk(LK_NOTICE,
                "%s:%d: %s dump (nb) : dump nb bytes from test buffer\n",
                __PRETTY_FUNCTION__, __LINE__, argv[0].str);
        printlk(LK_NOTICE,
                "%s:%d: %s write_byte (name) (nb) (value) : send nb bytes of value to file\n",
                __PRETTY_FUNCTION__, __LINE__, argv[0].str);
        printlk(LK_NOTICE,
                "%s:%d: %s write_file (name) (text) : send text to file\n",
                __PRETTY_FUNCTION__, __LINE__, argv[0].str);
        printlk(LK_NOTICE,
                "%s:%d: %s write_random (name) (nb) : send random bytes to file\n",
                __PRETTY_FUNCTION__, __LINE__, argv[0].str);
        printlk(LK_NOTICE,
                "%s:%d: %s read_file (name) (nb) : read from file (nb = 0 will read \
		all the file)\n",
                __PRETTY_FUNCTION__, __LINE__, argv[0].str);
        printlk(LK_NOTICE,
                "%s:%d: %s flush (id) : flush all binary pipes + reset test buffer\n",
                __PRETTY_FUNCTION__, __LINE__, argv[0].str);
        printlk(LK_NOTICE, "%s:%d: %s loop (id) : loopback test (%u MB max)\n",
                __PRETTY_FUNCTION__, __LINE__, argv[0].str, APP_BUF_SIZE >> 20);
        return -1;
    }

    /* List available binary pipes */
    if (!strcmp(argv[1].str, "list")) {
        ivshm_binary_list();
        return 0;
    }

    /* Allocate rpmsg test buffer */
    if (!app_buf) {
        size_t offset = 0;
        void *ptr;

        app_buf = malloc(APP_BUF_SIZE);

        ASSERT(app_buf);

        ptr = app_buf;
        while (offset < APP_BUF_SIZE) {
            *(uintptr_t *)(ptr + offset) = (uintptr_t)(ptr + offset);
            offset += sizeof(void *);
        }
    }

    /* Flush rpmsg test buffer */
    if (!strcmp(argv[1].str, "flush")) {
        if (argc < 3) {
            printlk(LK_ERR, "%s:%d: Invalid command\n", __PRETTY_FUNCTION__,
                    __LINE__);
            goto usage;
        }

        unsigned id = argv[2].u;
        ssize_t count;
        size_t offset = 0;
        void *ptr;

        while ((count = cipc_read_buf(id, app_buf, APP_BUF_SIZE)) > 0)
            ;

        if (count < 0)
            return 0;

        ptr = app_buf;
        while (offset < APP_BUF_SIZE) {
            *(uintptr_t *)(ptr + offset) = (uintptr_t)(ptr + offset);
            offset += sizeof(void *);
        }

        return 0;
    }

    /* Rpmsg FILE test commands (through CIPC node only) */
    if (!strcmp(argv[1].str, "send")) {
        if (argc < 4) {
            printlk(LK_ERR, "%s:%d: Invalid command\n", __PRETTY_FUNCTION__,
                    __LINE__);
            goto usage;
        }

        unsigned id = argv[2].u;
        ssize_t count, len = argv[3].u;

        count = cipc_write_buf(id, app_buf, MIN(len, APP_BUF_SIZE));
        printlk(LK_NOTICE, "%s:%d: %lu bytes sent\n", __PRETTY_FUNCTION__,
                __LINE__, count);

        return 0;
    }

    if (!strcmp(argv[1].str, "write_file")) {
        if (argc < 4) {
            printlk(LK_ERR, "%s:%d: Invalid command\n", __PRETTY_FUNCTION__,
                    __LINE__);
            goto usage;
        }

        char *name = (char*) argv[2].str;
        char *text = (char*) argv[3].str;
        int len = strlen(argv[3].str);
        ssize_t count = 0;
        int ret = (-1);

        count = cipc_write_file(IVSHM_EP_ID_BINARY, text,
                                MIN(len, APP_BUF_SIZE), name, FILE_O_CREATE);
        if (count > 0) {
            printlk(LK_NOTICE, "%s:%d: %ld bytes sent to '%s'\n",
                    __PRETTY_FUNCTION__, __LINE__, count, name);
            ret = 0;
        } else
            printlk(LK_ERR, "%s:%d: Error %ld while writing file '%s'!\n",
                    __PRETTY_FUNCTION__, __LINE__, count, name);

        return ret;
    }

    if (!strcmp(argv[1].str, "write_byte")) {
        if (argc < 5) {
            printlk(LK_ERR, "%s:%d: Invalid command\n", __PRETTY_FUNCTION__,
                    __LINE__);
            goto usage;
        }

        char *name = (char*) argv[2].str;
        size_t len = (size_t)argv[3].u;
        char value = (char)  argv[4].u;
        int ret = (-1);
        ssize_t count = 0;

        memset(app_buf, value, len);

        count = cipc_write_file(IVSHM_EP_ID_BINARY, app_buf,
                                MIN(len, APP_BUF_SIZE), name, FILE_O_CREATE);
        if (count > 0) {
            printlk(LK_NOTICE, "%s:%d: %ld bytes of 0x%02x sent to %s\n",
                    __PRETTY_FUNCTION__, __LINE__, count, value, name);
            ret = 0;
        } else
            printlk(LK_ERR, "%s:%d: Error %ld while writing bytes to '%s'!\n",
                __PRETTY_FUNCTION__, __LINE__, count, name);

        return ret;
    }

    if (!strcmp(argv[1].str, "write_random")) {
        if (argc < 4) {
            printlk(LK_ERR, "%s:%d: Invalid command\n", __PRETTY_FUNCTION__,
                    __LINE__);
            goto usage;
        }

        /* MD5 hash of memsink buffer */
        MD5_CTX ctx;
        MD5_Init(&ctx);

        char *name = (char*) argv[2].str;
        size_t len = (size_t)argv[3].u;
        char *buffer = (char*) app_buf;

        ssize_t count = 0;
        uint8_t random = 0;
        size_t i;
        uint8_t result[16];
        char md5str[32 + 1];
        int ret = (-1);

        for (i = 0; i < len; i++) {
            random = rand() & 0xFF;
            buffer[i] = random;
        }

        MD5_Update(&ctx, buffer, len);
        MD5_Final(result, &ctx);

        snprintf(md5str, sizeof(md5str),
              "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
             result[ 0], result[ 1], result[ 2], result[ 3],
             result[ 4], result[ 5], result[ 6], result[ 7],
             result[ 8], result[ 9], result[10], result[11],
             result[12], result[13], result[14], result[15]
             );

        count = cipc_write_file(IVSHM_EP_ID_BINARY, app_buf,
                                MIN(len, APP_BUF_SIZE), name, FILE_O_CREATE);
        if (count > 0) {
            printlk(LK_NOTICE, "%s:%d: %ld random bytes sent to %s md5sum:%s\n",
                __PRETTY_FUNCTION__, __LINE__, count, name, md5str);
            ret = 0;
        } else
            printlk(LK_ERR, "%s:%d: Error %ld while writing random data to '%s'!\n",
                __PRETTY_FUNCTION__, __LINE__, count, name);

        return ret;
    }

    if (!strcmp(argv[1].str, "read")) {
        if (argc < 4) {
            printlk(LK_ERR, "%s:%d: Invalid command\n", __PRETTY_FUNCTION__,
                    __LINE__);
            goto usage;
        }

        unsigned id = argv[2].u;
        size_t count, len = argv[3].u;

        count = cipc_read_buf(id, app_buf, MIN(len, APP_BUF_SIZE));
        printlk(LK_NOTICE, "%s:%d: %lu bytes read from bin buffer\n",
                __PRETTY_FUNCTION__, __LINE__, count);

        return 0;
    }

    if (!strcmp(argv[1].str, "read_file")) {
        if (argc < 4) {
            printlk(LK_ERR, "%s:%d: Invalid command\n", __PRETTY_FUNCTION__,
                    __LINE__);
            goto usage;
        }
        int ret = (-1);
        char *filename = (char*) argv[2].str;
        ssize_t len = argv[3].u;
        ssize_t count;
        char md5str[33]; /* 32 bytes + null byte */
        uint8_t result[16];
        char *buffer = (char*) app_buf;

        /* Initialize context for md5 */
        MD5_CTX ctx;
        MD5_Init(&ctx);

        count = cipc_read_file(IVSHM_EP_ID_BINARY, app_buf,
            MIN(len, APP_BUF_SIZE), filename);
        if (count > 0) {
            /* Compute md5 for file just read */
            MD5_Update(&ctx, buffer, count);
            MD5_Final(result, &ctx);
            snprintf(md5str, sizeof(md5str),
                  "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
                 result[ 0], result[ 1], result[ 2], result[ 3],
                 result[ 4], result[ 5], result[ 6], result[ 7],
                 result[ 8], result[ 9], result[10], result[11],
                 result[12], result[13], result[14], result[15]
                 );
            printlk(LK_NOTICE, "%ld bytes read from %s, md5sum:%s\n",
                count, filename, md5str);
            ret = 0;
        } else
            printlk(LK_ERR, "%s:%d: Error %ld while reading'%s'!\n",
                    __PRETTY_FUNCTION__, __LINE__, count, filename);

        return ret;
    }

    if (!strcmp(argv[1].str, "dump")) {
        if (argc < 3) {
            printlk(LK_ERR, "%s:%d: Invalid command\n", __PRETTY_FUNCTION__,
                    __LINE__);
            goto usage;
        }

        size_t len = argv[2].u;

        hexdump(app_buf, MIN(len, APP_BUF_SIZE));

        return 0;
    }

    if (!strcmp(argv[1].str, "loop")) {
        if (argc < 3) {
            printlk(LK_ERR, "%s:%d: Invalid command\n", __PRETTY_FUNCTION__,
                    __LINE__);
            goto usage;
        }

        unsigned id = argv[2].u;;
        ssize_t offset = 0, sent, count;

        while (offset < APP_BUF_SIZE) {
            size_t chunk = MIN(CHUNK_SIZE, APP_BUF_SIZE - offset);

            count = cipc_read_buf(id, app_buf + offset, chunk);

            if (count < 0)
                return 0;

            if (count == 0) {
                thread_yield();
                continue;
            }

            sent = cipc_write_buf(id, app_buf + offset, count);

            ASSERT(count == sent);

            offset += count;

            printlk(LK_INFO, "%s:%d: New chunk (%lu Bytes) captured (%lu/%u)\n",
                    __PRETTY_FUNCTION__, __LINE__, count, offset, APP_BUF_SIZE);
        }

        return 0;
    }
    printlk(LK_ERR, "%s:%d: Unrecognized subcommand '%s'\n",
            __PRETTY_FUNCTION__, __LINE__, argv[1].str);
    goto usage;
}

APP_START(rpmsg)
    .flags = 0,
APP_END

