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

#include <err.h>
#include <string.h>
#include <stdlib.h>
#include <dev/driver.h>
#include <lib/cbuf.h>
#include <lib/appargs.h>
#include <assert.h>

#include <event-manager.h>
#include <cipc.h>


#define EM_CBUF_SIZE                    (4 << 10)

#define EM_ENDPOINT_ID_UNKNOWN          0xdead

struct ivshm_event_manager {
    uint32_t id;
    cbuf_t circ_buf;
    thread_t *thread;
};

static struct ivshm_event_manager g_event_manager = {
    .id = EM_ENDPOINT_ID_UNKNOWN,
};

int ivshm_send_event(const char *str)
{
    size_t len;

    if (g_event_manager.id == EM_ENDPOINT_ID_UNKNOWN)
        return ERR_NOT_READY;

    len = strlen(str) + 1;

    DEBUG_ASSERT(len < EM_CBUF_SIZE);
    DEBUG_ASSERT(cbuf_space_avail(&g_event_manager.circ_buf) >= len);

    return cbuf_write(&g_event_manager.circ_buf, str, len, false);
}

static int ivshm_event_manager_thread(void *arg)
{
    struct ivshm_event_manager *em = arg;
    uint32_t *em_buffer = malloc(EM_CBUF_SIZE * sizeof(uint8_t));
    if (!em_buffer)
        return ERR_NO_MEMORY;

    size_t count;

    while (1) {
        thread_sleep(50);
        /* blocking read */
        count = cbuf_read(&em->circ_buf, em_buffer, EM_CBUF_SIZE, true);

        cipc_write_buf(em->id, em_buffer, count);
    }

    return 0;
}

static status_t ivshm_event_manager_init(struct device *dev)
{
    struct device *ivshm_dev = of_device_lookup_device(dev, "service");
    int ret;

    DEBUG_ASSERT(ivshm_dev);
    DEBUG_ASSERT(g_event_manager.id == EM_ENDPOINT_ID_UNKNOWN);

    ret = of_device_get_int32(ivshm_dev, "id", &g_event_manager.id);
    if (ret) {
        printlk(LK_ERR, "%s:%d: Cannot retrieve binary endpoint id\n",
                __PRETTY_FUNCTION__, __LINE__);
        return ERR_INVALID_ARGS;
    }

    cbuf_initialize(&g_event_manager.circ_buf, EM_CBUF_SIZE);

    g_event_manager.thread = thread_create(
                "event-manager",
                &ivshm_event_manager_thread,
                &g_event_manager,
                LOW_PRIORITY,
                DEFAULT_STACK_SIZE);

    thread_detach_and_resume(g_event_manager.thread);

    printlk(LK_NOTICE, "%s:%d: event manager ready, using %#x endpoint\n",
            __PRETTY_FUNCTION__, __LINE__, g_event_manager.id);


    return 0;
}

static struct driver_ops the_ops = {
    .init = ivshm_event_manager_init,
};

DRIVER_EXPORT_WITH_LVL(imx_event_manager, &the_ops, DRIVER_INIT_PLATFORM);
