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

#include <stdlib.h>
#include <string.h>
#include <kernel/mutex.h>
#include <err.h>
#include <assert.h>
#include <list.h>
#include <lib/appargs.h>
#include <dev/driver.h>

#include <dev/ivshm.h>
#include "ivshmem-pipe.h"
#include "ivshmem-endpoint.h"
#include "ivshmem-rpc.h"


//#define DEBUG_BUFFER 1


#define RPC_DEFAULT_TIMEOUT             1000
#define RPC_BUFFER_SIZE                 (8 << 10)


struct ivshm_rpc_service {
    unsigned id;
    struct list_node node;
    mutex_t mutex;
    event_t wait;
    char *buf;
    size_t size;
    struct rpc_client_callback **callbacks;
    void *data;
    uint32_t timeout;
    struct ivshm_info *info;
    unsigned tid; /* transaction id */
};

#define to_ivshm_rpc_service(x) \
    ((struct ivshm_rpc_service *)&x->private)

static struct list_node rpc_service_list = LIST_INITIAL_VALUE(rpc_service_list);

/* ivshm RPC API */
struct ivshm_rpc_service *ivshm_get_rpc_service(unsigned id)
{
    struct ivshm_rpc_service *service = NULL;

    list_for_every_entry(&rpc_service_list, service, struct ivshm_rpc_service, node) {
        if (service->id == id)
            return service;
    }

    return NULL;
}

int ivshm_rpc_call(struct ivshm_rpc_service *service, unsigned rpc_id,
                       void *in, size_t len, void *out, size_t *out_len)
{
    struct ivshm_ep_buf ep_buf;
    struct ivshm_rpc_header *hdr_out, hdr_in = {
        .id = rpc_id,
        .type = IVSHM_RPC_CALL,
        .len = len,
    };
    status_t ret;

    if (len > IVSHM_RPC_GET_SZ_IN(rpc_id)) {
        printlk(LK_ERR, "%s:%d: Incorrect in size: %ld > %d\n",
                __PRETTY_FUNCTION__, __LINE__, len,
                IVSHM_RPC_GET_SZ_IN(rpc_id));
        return ERR_INVALID_ARGS;
    }

    if (IVSHM_RPC_GET_SZ_OUT(rpc_id))
        if (!out_len || (IVSHM_RPC_GET_SZ_OUT(rpc_id) != *out_len)) {
            if (out_len)
                printlk(LK_ERR, "%s:%d: Incorrect out size: %ld != %d\n",
                        __PRETTY_FUNCTION__, __LINE__, *out_len,
                        IVSHM_RPC_GET_SZ_OUT(rpc_id));
            else
                printlk(LK_ERR, "%s:%d: No valid output size\n",
                        __PRETTY_FUNCTION__, __LINE__);
            return ERR_INVALID_ARGS;
        }

    mutex_acquire(&service->mutex);
    hdr_in.tid = ++service->tid;

    ivshm_ep_buf_init(&ep_buf);

#ifdef DEBUG_BUFFER
    printlk(LK_NOTICE, "%s:%d: >> parameters\n", __PRETTY_FUNCTION__, __LINE__);
    hexdump(in, len);

    printlk(LK_NOTICE, "%s:%d: >> header\n", __PRETTY_FUNCTION__, __LINE__);
    hexdump(&hdr_in, sizeof(struct ivshm_rpc_header));
#endif

    memset(service->buf, 0, sizeof(struct ivshm_rpc_header));
    smp_wmb();
    ivshm_ep_buf_add(&ep_buf, in, len);
    ivshm_ep_buf_add(&ep_buf, &hdr_in, sizeof(struct ivshm_rpc_header));

    /* Send message and wait for response */
    ivshm_endpoint_write(to_ivshm_endpoint(service), &ep_buf);

    if (IVSHM_RPC_GET_SZ_OUT(rpc_id) == 0) {
        /* No response expected */
        ret = NO_ERROR;
        goto out;
    }

    unsigned retry = 5;

repeat:
    ret = event_wait_timeout(&service->wait, service->timeout);
    if (ret == ERR_TIMED_OUT)
        goto out;

    /* Copy responses */
    hdr_out = (struct ivshm_rpc_header *)service->buf;

#ifdef DEBUG_BUFFER
    hexdump(hdr_out, sizeof(struct ivshm_rpc_header) + hdr_out->len);
#endif

    if (hdr_out->len > *out_len) {
        ret = ERR_INVALID_ARGS;
        goto out;
    }

    if (hdr_out->tid != service->tid && retry-- > 0) {
        /* Receive response for incorrect rpc call */
        printlk(LK_INFO, "%s:%d: Wrong transaction id. Expect %#x, receive %#x\n",
                __PRETTY_FUNCTION__, __LINE__, service->tid, hdr_out->tid);
        goto repeat;
    }

    if (retry == 0) {
        ret = ERR_INTERNAL;
        goto out;
    }

    if (hdr_out->id != rpc_id) {
        printlk(LK_INFO, "%s:%d: %s not valid response. Expect %#X\n",
                __PRETTY_FUNCTION__, __LINE__, __func__, rpc_id);
        hexdump(hdr_out, sizeof(struct ivshm_rpc_header));
        ret = ERR_NOT_READY;
        goto out;
    }

    /* Copy response into dest pointer */
    if (out)
        memcpy(out, hdr_out->payload, hdr_out->len);

    if (out_len)
        *out_len = hdr_out->len;

    ret = NO_ERROR;

out:
    mutex_release(&service->mutex);
    return ret;
}

int ivshm_rpc_reply(struct ivshm_rpc_service *service, unsigned rpc_id,
                    void *data, size_t len)
{
    struct ivshm_rpc_header hdr;
    struct ivshm_ep_buf ep_buf;

    hdr.id = rpc_id; //IVSHM_RPC_GET_ID(rpc_id);
    hdr.type = IVSHM_RPC_REPLY;
    hdr.len = len;

    ivshm_ep_buf_init(&ep_buf);
    ivshm_ep_buf_add(&ep_buf, data, len);
    ivshm_ep_buf_add(&ep_buf, &hdr, sizeof(struct ivshm_rpc_header));

    /* Send response */
    return ivshm_endpoint_write(to_ivshm_endpoint(service), &ep_buf);
}


/* Register RPC client */
void ivshm_rpc_register_client(struct ivshm_rpc_service *service,
                               struct rpc_client_callback **cb)
{
    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    DEBUG_ASSERT(cb);

    if (service->callbacks)
        printlk(LK_NOTICE, "%s:%d: Overwritting callbacks for service id %d\n",
                __PRETTY_FUNCTION__, __LINE__, service->id);

    service->callbacks = cb;
    printlk(LK_VERBOSE, "%s: exit\n", __PRETTY_FUNCTION__);
}

void ivshm_rpc_unregister_client(struct ivshm_rpc_service *service)
{
    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    DEBUG_ASSERT(service);
    service->callbacks = NULL;

    printlk(LK_VERBOSE, "%s: exit\n", __PRETTY_FUNCTION__);
}

void ivshm_rpc_set_data(struct ivshm_rpc_service *service, void *data)
{
    ASSERT(service);
    service->data = data;
}

void *ivshm_rpc_get_data(struct ivshm_rpc_service *service)
{
    ASSERT(service);
    return service->data;
}

static status_t ivshm_rpc_send(struct ivshm_rpc_service *service,
            unsigned rpc_id, void *in, size_t len, void *out, size_t *out_len)
{
    int ret;

    if (!service)
        return ERR_NOT_READY;

    if ((service->info == NULL)|| (!ivshm_pipe_is_running(service->info)))
        return ERR_NOT_READY;

    ret = ivshm_rpc_call(service, rpc_id, in, len, out, out_len);

    if (ret) {
        printlk(LK_ERR, "%s:%d: Error while sending rpc init call. Exiting...\n",
                __PRETTY_FUNCTION__, __LINE__);
        return ret;
    }

    return NO_ERROR;
}

status_t ivshm_rpc_cmd(struct ivshm_rpc_service *service,
            unsigned rpc_id)
{
    status_t ret;
    int retcode;
    size_t out_sz = sizeof(retcode);

    ret = ivshm_rpc_send(service, rpc_id, NULL, 0, &retcode, &out_sz);

    if (ret) {
        printlk(LK_ERR, "%s:%d: Error (%d) while sending %x call\n",
                __PRETTY_FUNCTION__, __LINE__, ret, rpc_id);
        goto error;
    }

    if (retcode) {
        printlk(LK_ERR, "%s:%d: Error (%d) while executing rpc %x \n",
                __PRETTY_FUNCTION__, __LINE__, ret, rpc_id);
        ret = ERR_INVALID_ARGS;
        goto error;
    }

error:
    return ret;
}

status_t ivshm_rpc_cmd_get(struct ivshm_rpc_service *service,
                                        unsigned rpc_id, void *in, size_t sz)
{
    status_t ret;
    size_t response_sz = sz;

    ret = ivshm_rpc_send(service, rpc_id, NULL, 0, in, &response_sz);
    ASSERT(sz == response_sz);
    if (ret) {
        printlk(LK_ERR, "%s:%d: Error (%d) while sending %x call\n",
                __PRETTY_FUNCTION__, __LINE__, ret, rpc_id);
        goto error;
    }

error:
    return ret;
}

status_t ivshm_rpc_cmd_set(struct ivshm_rpc_service *service,
                                        unsigned rpc_id, void *in, size_t sz)
{
    status_t ret;
    unsigned retcode = 0;
    size_t response_sz = 4;

    ret = ivshm_rpc_send(service, rpc_id, in, sz, &retcode, &response_sz);
    if ((response_sz != 4) || (retcode != 0)) {
        printlk(LK_ERR, "%s:%d: Error (%d) while executing %x call\n",
                __PRETTY_FUNCTION__, __LINE__, retcode, rpc_id);
        goto error;
    }

    if (ret) {
        printlk(LK_ERR, "%s:%d: Error (%d) while sending %x call\n",
                __PRETTY_FUNCTION__, __LINE__, ret, rpc_id);
        goto error;
    }

error:
    return ret;
}


static ssize_t ivshm_rpc_consume(struct ivshm_endpoint *ep,
                                 struct ivshm_pkt *pkt)
{
    struct ivshm_rpc_service *service = to_ivshm_rpc_service(ep);
    size_t len = ivshm_pkt_get_payload_length(pkt);
    struct ivshm_rpc_header *hdr;

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    /* Store message in service buffer */
    memcpy(service->buf, pkt->payload, len);
    service->size = len;
    smp_wmb();

    hdr = (struct ivshm_rpc_header *)service->buf;
    if (hdr->type == IVSHM_RPC_REPLY) {
        /* wake up client thread */
        event_signal(&service->wait, false);
    } else if (hdr->type == IVSHM_RPC_CALL) {
        /* Look for callback and jump in */
        int i = 0;
        struct rpc_client_callback *rpc_callback;

        while ((rpc_callback = *(service->callbacks + i)) != NULL) {
            printlk(LK_INFO, "%s:%d: check service callback %d: id %X\n",
                    __PRETTY_FUNCTION__, __LINE__, i, rpc_callback->id);
            if (rpc_callback->id == hdr->id)
                break;

            i++;
        }

        if (rpc_callback == NULL) {
            printlk(LK_ERR, "%s:%d: No callback available for cmd_id %X\n",
                    __PRETTY_FUNCTION__, __LINE__, hdr->id);
            return ERR_NOT_FOUND;
        }

        return rpc_callback->fn(service, rpc_callback, hdr->payload);
    } else {
        panic("Unsupported command type\n");
    }

    printlk(LK_VERBOSE, "%s: exit\n", __PRETTY_FUNCTION__);

    return 0;
}

int ivshm_init_rpc(struct ivshm_info *info)
{
    return NO_ERROR;
}

void ivshm_exit_rpc(struct ivshm_info *info)
{
    struct ivshm_rpc_service *service, *temp;

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    list_for_every_entry_safe(&rpc_service_list, service, temp,
                         struct ivshm_rpc_service, node) {

        list_delete(&service->node);
        free(service->buf);
        ivshm_endpoint_destroy(to_ivshm_endpoint(service));
    }

    printlk(LK_VERBOSE, "%s: exit\n", __PRETTY_FUNCTION__);
}

static status_t ivshm_rpc_dev_init(struct device *dev)
{
    struct ivshm_rpc_service *service;
    uint32_t id, timeout;
    struct ivshm_endpoint *ep;
    char name[IVSHM_EP_MAX_NAME];
    struct ivshm_dev_data *ivshm_dev = ivshm_get_device(0);
    int ret;

    printlk(LK_NOTICE, "%s: entry\n", __PRETTY_FUNCTION__);

    if (!ivshm_dev)
        return ERR_NOT_READY;

    struct ivshm_info *info = ivshm_dev->handler_arg;
    if (!info)
        return ERR_NOT_READY;

    ret = of_device_get_int32(dev, "id", &id);
    if (ret) {
        printlk(LK_ERR, "%s:%d: Endpoint id property can't be read, aborting!\n",
                __PRETTY_FUNCTION__, __LINE__);
        return ERR_NOT_FOUND;
    }

    ret = of_device_get_int32(dev, "timeout", &timeout);
    if (ret)
        timeout = RPC_DEFAULT_TIMEOUT;

    snprintf(name, IVSHM_EP_MAX_NAME, "rpc-%u", id);
    ep = ivshm_endpoint_create(
                name,
                id,
                ivshm_rpc_consume,
                info,
                RPC_BUFFER_SIZE,
                sizeof(struct ivshm_rpc_service)
                );

    service = to_ivshm_rpc_service(ep);
    memset(service, 0, sizeof(struct ivshm_rpc_service));
    service->info = info;
    service->id = id;
    service->tid = 0;
    service->timeout = timeout;
    mutex_init(&service->mutex);
    event_init(&service->wait, false, EVENT_FLAG_AUTOUNSIGNAL);
    service->buf = malloc(RPC_BUFFER_SIZE);

    if (!service->buf) {
        ivshm_endpoint_destroy(ep);
        return ERR_NO_MEMORY;
    }

    list_add_tail(&rpc_service_list, &service->node);

    printlk(LK_NOTICE, "%s: exit\n", __PRETTY_FUNCTION__);

    return NO_ERROR;
}

static struct driver_ops the_ops = {
    .init = ivshm_rpc_dev_init,
};

DRIVER_EXPORT_WITH_LVL(imx_ivshm_rpc, &the_ops, DRIVER_INIT_CORE);
