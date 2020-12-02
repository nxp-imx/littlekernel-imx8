/*
 * Copyright 2020 NXP
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
#include <dev/ivshm.h>
#include <ivshmem-pipe.h>
#include <ivshmem-endpoint.h>

#include <assert.h>

#include "md5.h"

#if WITH_LIB_CONSOLE
#include <lib/console.h>
#else
#error "ivshmem app needs a console"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define APP_BUF_SIZE (16 << 20) /* 16 MBytes */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static int cmd_ivshmem(int argc, const cmd_args *argv);

/*******************************************************************************
 * Variables
 ******************************************************************************/
STATIC_COMMAND_START
STATIC_COMMAND("ivshmem", "ivshmem commands", &cmd_ivshmem)
STATIC_COMMAND_END(ivshmem);


/*******************************************************************************
 * Code
 ******************************************************************************/

static void show_endpoint_stats(struct ivshm_endpoint *ep)
{
    printlk(LK_NOTICE, "%s:%d: endpoint %d\nname: %s\nnum_rx: %u\nnum_tx: %u\ncbuf: %u/%u\nlatency(us): mean %u max %u\n",
           __PRETTY_FUNCTION__, __LINE__,
           IVSHM_EP_GET_ID(ep->id), ep->name, ep->num_rx, ep->num_tx,
           ep->level_max, ep->rx.len,
           ep->latency_mean, ep->latency_max);
}

/* ivshmem command
 */
static int cmd_ivshmem(int argc, const cmd_args *argv)
{
    struct ivshm_dev_data *ivshm_dev = ivshm_get_device(0);
    struct ivshm_endpoint *ep;

    if (!ivshm_dev)
        return ERR_NOT_READY;

    struct ivshm_info *info = ivshm_dev->handler_arg;
    if (!info)
        return ERR_NOT_READY;

    if (argc > 1) {
        unsigned id = argv[1].u;

        list_for_every_entry(&info->endpoints_list, ep, struct ivshm_endpoint, node)
            if (IVSHM_EP_GET_ID(ep->id) == id)
                show_endpoint_stats(ep);

        return 0;
    }

    printlk(LK_NOTICE,
            "%s:%d: ivshmem monitor\trx\ttx\tlevel max\tlatency us (mean/max) \n",
            __PRETTY_FUNCTION__, __LINE__);
    list_for_every_entry(&info->endpoints_list, ep, struct ivshm_endpoint, node) {
        printlk(LK_NOTICE, "%s:%d: %d: %s:\t%u\t%u\t%u\t%u/%u\n",
           __PRETTY_FUNCTION__, __LINE__,
           IVSHM_EP_GET_ID(ep->id), ep->name,
           ep->num_rx, ep->num_tx,
           ep->level_max,
           ep->latency_mean, ep->latency_max);
    }

    return 0;
}

APP_START(ivshmem)
    .flags = 0,
APP_END

