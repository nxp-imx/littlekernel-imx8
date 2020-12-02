/*
 * Copyright 2018-2021 NXP
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

#ifndef __IVSHMEM_ENDPOINT_H
#define __IVSHMEM_ENDPOINT_H

#ifdef LK
#define list_head list_node
#include <lib/cbuf.h>
#include <kernel/semaphore.h>
#else
#include "ivshmem-cbuf.h"
#include <linux/semaphore.h>
#endif

#define IVSHM_EP_MAX_NAME 32
#define IVSHM_EP_MIN_BUFSIZE 512

struct ivshm_pkt;

struct ivshm_endpoint {
    struct list_head node;
    unsigned id;
    unsigned flags;
    cbuf_t rx;
    struct ivshm_info *ivshm;
    ssize_t (*consume)(struct ivshm_endpoint *, struct ivshm_pkt *);
    char name[IVSHM_EP_MAX_NAME];
    size_t max_pkt_size;
#ifdef LK
    thread_t *thread;
    semaphore_t rx_pkt_sem;
    bool thread_running;
#else
    struct task_struct *thread;
    struct semaphore rx_pkt_sem;
#endif
#ifdef IVSHMEM_MONITOR
    unsigned num_tx;
    unsigned num_rx;
    unsigned level_max;
    unsigned latency_max;
    unsigned latency_mean;
#endif
    char private[] __attribute__((__aligned__(64)));
};

#define to_ivshm_endpoint(x) \
    containerof(x, struct ivshm_endpoint, private)

#define IVSHM_HDR_CUST_PARAMS_MAX 8

struct ivshm_imx8_hdr {
    int magic;
    unsigned len;
    int cmd;
    unsigned private[IVSHM_HDR_CUST_PARAMS_MAX]; /* Used to attach further parameters to ivshm packet */
    int pad;
};

struct ivshm_imx8_pkt {
    struct ivshm_imx8_hdr hdr;
    char payload[];
};

#define IVSHM_EP_MAX_IOVEC 8
struct ivshm_ep_buf {
    iovec_t vec[IVSHM_EP_MAX_IOVEC];
    int idx;
};

static inline void ivshm_ep_buf_init(struct ivshm_ep_buf *buf)
{
    buf->idx = IVSHM_EP_MAX_IOVEC;
}

static inline uint ivshm_ep_buf_iov_cnt(struct ivshm_ep_buf *buf)
{
    return IVSHM_EP_MAX_IOVEC - buf->idx;
}

static inline iovec_t *ivshm_ep_buf_iov_base(struct ivshm_ep_buf *buf)
{
    if (IVSHM_EP_MAX_IOVEC > buf->idx)
        return &buf->vec[buf->idx];

    return NULL;

}

static inline ssize_t ivshm_ep_buf_iov_size(struct ivshm_ep_buf *buf)
{
    return iovec_size(&buf->vec[buf->idx], ivshm_ep_buf_iov_cnt(buf));
}

static inline int ivshm_ep_buf_add(struct ivshm_ep_buf *buf, void *base, size_t len)
{
    if (buf->idx < 1)
        return -1;

    buf->idx -= 1;
    buf->vec[buf->idx].iov_base = base;
    buf->vec[buf->idx].iov_len = len;

    return 0;
}

/*
 *
 * The endpoint ID must be unique, and registers here.
 *
 */
#define IVSHM_EP_ID_CONSOLE             0x2

/* RPC endpoint IDs */
#define IVSHM_EP_ID_RPC_HDMI            0x100
#define IVSHM_EP_ID_RPC_DAC             0x101
#define IVSHM_EP_ID_RPC_ADC             0x102

/* Binary endpoints */
#define IVSHM_EP_ID_LKTRACES            0x200
#define IVSHM_EP_ID_BINARY              0x201
#define IVSHM_EP_ID_WATCHDOG            0x202
#define IVSHM_EP_ID_CIPC                0x203
#define IVSHM_EP_ID_EVENT_MANAGER       0x205

/* Customer endpoints - Starting from 0x400 */

/* Endpoint usefull macros */
#define EP_ID_MASK                      0xFFFF
#define EP_ID_SHIFT                     0
#define IVSHM_EP_GET_ID(x)              (((x) >> EP_ID_SHIFT) & EP_ID_MASK)
#define IVSHM_EP_SET_ID(x)              (((x) & EP_ID_MASK) << EP_ID_SHIFT)

#define EP_PRIO_MASK                    0xFF
#define EP_PRIO_SHIFT                   24
#define IVSHM_EP_GET_PRIO(x)            (((x) >> EP_PRIO_SHIFT) & EP_PRIO_MASK)
#define IVSHM_EP_SET_PRIO(x)            (((x) & EP_PRIO_MASK) << EP_PRIO_SHIFT)

#define EP_SCHED_MASK                   0xFF
#define EP_SCHED_SHIFT                  16
#define IVSHM_EP_GET_SCHED(x)           (((x) >> EP_SCHED_SHIFT) & EP_SCHED_MASK)
#define IVSHM_EP_SET_SCHED(x)           (((x) & EP_SCHED_MASK) << EP_SCHED_SHIFT)

/*
 * ivshm_endpoint_write:
 *  This queues a iovec like buffers to ivshm virtio ring queues.
 *  Despite the function does not block, it can sleep so should not be called
 *  from an atomic context.
 *
 *
 *  The caller can encapsulate its protocol layer within the ivshm_ep_buf
 *  adding them to it from lower to higher protocol layer.
 *
 */

ssize_t ivshm_endpoint_write(struct ivshm_endpoint *ep, struct ivshm_ep_buf *buf);

/*
 * ivshm_endpoint_create:
 *      This registers an endpoint, with an unique ID to the ivshm low level
 *      driver.
 *
 *  The caller can encapsulate its protocol layer within the ivshm_ep_buf
 *  adding them to it from lower to higher protocol layer.
 *
 *  For every packet receive, the consume callback will be called with payload
 *  in ivshm_pkt structure. The callback is called in thread context.
 *
 *  The packets are flowing to/from ivshm through circular buffers.
 *
 * name: endpoint name
 * id: unique endpoint id
 * consume: callback called for any packet receive. thread context.
 * ivshm_info: shm driver info - should be opaque to client..
 * bufsize: circular buffer size
 * private_size: Size of private client driver data embedded in endpoint.
 * private: Pointer to the private client driver data to be embeded in endpoint.
 */
struct ivshm_endpoint *ivshm_endpoint_create_w_private(
    const char *name,
    unsigned id,
    ssize_t (*consume)(struct ivshm_endpoint *ep, struct ivshm_pkt *),
    struct ivshm_info *info,
    size_t bufsize,
    size_t private_size,
    void *private);

struct ivshm_endpoint *ivshm_endpoint_create(
    const char *name,
    unsigned id,
    ssize_t (*consume)(struct ivshm_endpoint *ep, struct ivshm_pkt *),
    struct ivshm_info *info,
    size_t bufsize,
    size_t private_size);

void ivshm_endpoint_destroy(struct ivshm_endpoint *);

#endif
