/*
 * Copyright 2018-2020 NXP
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


#ifdef LK
#include <kernel/spinlock.h>
#include <dev/interrupt.h>
#include <dev/ivshm.h>
#include <lib/cbuf.h>
#include <iovec.h>
#include <err.h>
#include <malloc.h>
#include <platform.h>
#include "string.h"


#define swap(a, b) \
    do { typeof(a) __tmp = (a); (a) = (b); (b) = __tmp; } while (0)


#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define EINVAL ERR_INVALID_ARGS
#define ENOMEM ERR_NO_MEMORY

#define ERR_PTR(x) ((void*) x)

#define IVSHM_EP_DEFAULT(x) \
    (IVSHM_EP_SET_ID(x) | \
     IVSHM_EP_SET_PRIO(DEFAULT_PRIORITY) | \
     IVSHM_EP_SET_SCHED(0))

#else
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/completion.h>
#include <linux/jiffies.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,5,0)
#include <uapi/linux/sched/types.h>
#endif
#include "ivshmem-iovec.h"
#include "ivshmem-cbuf.h"

#define DEBUG_ASSERT(cond) BUG_ON(!(cond))

#define IVSHM_EP_DEFAULT(x) \
    (IVSHM_EP_SET_ID(x) | \
     IVSHM_EP_SET_PRIO(MAX_RT_PRIO - 10) | \
     IVSHM_EP_SET_SCHED(SCHED_RR))

typedef unsigned long spin_lock_saved_state_t;

#endif
#include "ivshmem-pipe.h"
#include "ivshmem-endpoint.h"
#include "debug.h"


static inline void ivshm_pipe_add_endpoint(struct ivshm_info *in, struct ivshm_endpoint *ep)
{
    spin_lock_saved_state_t flags;

    spin_lock_irqsave(&in->ep_lock, flags);
#ifdef LK
    list_add_tail(&in->endpoints_list, &ep->node);
#else
    list_add_tail(&ep->node, &in->endpoints_list);
#endif
    spin_unlock_irqrestore(&in->ep_lock, flags);
}

static inline void ivshm_pipe_remove_endpoint(struct ivshm_info *in, struct ivshm_endpoint *ep)
{
    spin_lock_saved_state_t flags;

    spin_lock_irqsave(&in->ep_lock, flags);
#ifdef LK
    list_delete(&ep->node);
#else
    list_del(&ep->node);
#endif
    spin_unlock_irqrestore(&in->ep_lock, flags);
}

static struct ivshm_endpoint *ivshm_pipe_get_endpoint(struct ivshm_info *in, unsigned id)
{
    struct ivshm_endpoint *ep;
    spin_lock_saved_state_t flags;

    spin_lock_irqsave(&in->ep_lock, flags);

#ifdef LK
    list_for_every_entry(&in->endpoints_list, ep, struct ivshm_endpoint, node) {
#else
    list_for_each_entry(ep, &in->endpoints_list, node) {
#endif
        if (IVSHM_EP_GET_ID(ep->id) == id) {
            spin_unlock_irqrestore(&in->ep_lock, flags);
            return ep;
        }
    }

    spin_unlock_irqrestore(&in->ep_lock, flags);
    return NULL;
}

int ivshm_endpoint_push(struct ivshm_info *in, void *buf, size_t len)
{
    struct ivshm_pkt_hdr *pkt_hdr = (struct ivshm_pkt_hdr *) buf;
    struct ivshm_endpoint *ep;
    size_t count, remaining;

    DEBUG_ASSERT(pkt_hdr->magic == IVSHM_PKT_MAGIC);
    DEBUG_ASSERT(pkt_hdr->len == len);

    ep = ivshm_pipe_get_endpoint(in, pkt_hdr->id);

    if (!ep) {
        printlk(LK_ERR, "%s:%d: No endpoint found for id %d\n",
                __PRETTY_FUNCTION__, __LINE__, pkt_hdr->id);
        return -EINVAL;
    }

#if 0
    buf += sizeof(struct ivshm_pkt_hdr);
    len -= sizeof(struct ivshm_pkt_hdr);
#endif

    remaining = len;

    printlk(LK_VERBOSE, "Pushing %zu to endpoint %s - id %d\n",
                                        len, ep->name, ep->id);

    while (remaining) {
#ifndef LK
        if (cbuf_space_avail(&ep->rx) < len) {
            int ret;

            mutex_lock(&ep->rx.mlock);
            ret = cbuf_increase(&ep->rx, len);
            mutex_unlock(&ep->rx.mlock);

            printlk(LK_NOTICE, "%s:%d: %s endpoint buffer is too small, increase to %u\n",
                   __PRETTY_FUNCTION__, __LINE__, ep->name, 1 << ep->rx.len_pow2);
            DEBUG_ASSERT(ret == 0);
        }
#endif

        count = cbuf_write(&ep->rx, buf, remaining, false);
#ifdef ENABLE_PERMISSIVE_MODE
        if (!count) {
            printlk(LK_WARNING, "%s:%d: WARNING PERMISSIVE: ep-%d RX buff is full\n",
                    __PRETTY_FUNCTION__, __LINE__, ep->id);
            continue;
        }
#else
        DEBUG_ASSERT(count > 0);
#endif
        remaining -= count;
        buf += count;
    }

#ifdef IVSHMEM_MONITOR
    {
        size_t used = cbuf_space_used(&ep->rx);

        if (used > ep->level_max)
            ep->level_max = used;

        ep->num_rx++;
    }
#endif

#ifdef LK
    sem_post(&ep->rx_pkt_sem, false);
#else
    up(&ep->rx_pkt_sem);
#endif

//   DEBUG_ASSERT(count == len);

    return remaining;
}

#ifndef LK
#ifdef IVSHMEM_MONITOR
#ifdef __arm__
static inline uint32_t arm_read_cntfrq(void)
{
    uint32_t _val;
    __asm__ volatile("mrc p15, 0, %0, c14, c0, 0": "=r" (_val));
    return _val;
}

static inline uint64_t arm_read_cntpct(void)
{
    uint64_t _val;
    __asm__ volatile("mrrc p15, 0, %0, %H0, c14" : "=r" (_val));
    return _val;
}
#else
static inline uint32_t arm_read_cntfrq(void)
{
    uint64_t _val;
    __asm__ volatile("mrs %0, cntfrq_el0 ": "=r" (_val));
    return (uint32_t) _val;
}

static inline uint64_t arm_read_cntpct(void)
{
    uint64_t _val;
    __asm__ volatile("mrs %0, cntpct_el0 ": "=r" (_val));
    return _val;
}
#endif

static inline uint64_t current_time_hires(void)
{
    uint32_t cntfrq = arm_read_cntfrq();
    uint64_t cntpct = arm_read_cntpct();
    uint64_t t;

    t = cntpct;

    if ((cntfrq % 1000000) == 0) {
        cntfrq /= 1000000;
    } else {
        t *= 1000;
        cntfrq /= 1000;
    }

    do_div(t, cntfrq);

    return t;
}
#endif
#endif

ssize_t ivshm_endpoint_write(struct ivshm_endpoint *ep, struct ivshm_ep_buf *buf)
{
    struct ivshm_pkt_hdr pkt_hdr;
    ivshm_ep_buf_add(buf, &pkt_hdr, sizeof(struct ivshm_pkt_hdr));

    pkt_hdr.magic = IVSHM_PKT_MAGIC;
    pkt_hdr.id = IVSHM_EP_GET_ID(ep->id);
    pkt_hdr.len = ivshm_ep_buf_iov_size(buf);
    pkt_hdr.pad = IVSHM_PKT_PAD_MAGIC;

#ifdef IVSHMEM_MONITOR
    smp_mb();
    pkt_hdr.ts = current_time_hires() & ((1ULL << 32) - 1);
    ep->num_tx++;
#endif

    return ivshm_write(
               ep->ivshm,
               ivshm_ep_buf_iov_base(buf),
               ivshm_ep_buf_iov_cnt(buf)
           );
}

ssize_t ivshm_endpoint_pkt_read(struct ivshm_endpoint *ep, void *buf, size_t len)
{
    struct ivshm_pkt *pkt = buf;
    size_t count = 0;
    unsigned remaining, pkt_len;
    iovec_t regions[2];

#define IVSHM_EP_TIMEOUT 1000
#ifndef LK
    int ret;

    ret = down_timeout(&ep->rx_pkt_sem, msecs_to_jiffies(IVSHM_EP_TIMEOUT));
    if (ret) {
        return (ssize_t) ret;
    }
#else
    status_t ret;
#ifdef IVSHM_EP_THREAD_TERMINATION
    ret = sem_timedwait(&ep->rx_pkt_sem, IVSHM_EP_TIMEOUT);
#else
    ret = sem_wait(&ep->rx_pkt_sem);
#endif
    if (ret) {
        return (ssize_t) ret;
    }
#endif

#ifndef LK
    mutex_lock(&ep->rx.mlock);
#endif

    count = cbuf_peek(&ep->rx, regions);

    DEBUG_ASSERT(count > sizeof(struct ivshm_pkt_hdr));


    count = iovec_to_membuf(buf, sizeof(struct ivshm_pkt_hdr), regions, 2, 0);
    DEBUG_ASSERT(count == sizeof(struct ivshm_pkt_hdr));
    DEBUG_ASSERT(pkt->hdr.magic == IVSHM_PKT_MAGIC);
    DEBUG_ASSERT(len > pkt->hdr.len);

    /* Save pkt->hdr.len here because it may change if endpoint buffer is
     * changed. pkt may point to invalid memory range then. */
    pkt_len = remaining = pkt->hdr.len;

    while (remaining) {
        count = cbuf_read(&ep->rx, buf, remaining, true);
        remaining -= count;
        buf += count;
    }

#ifndef LK
    mutex_unlock(&ep->rx.mlock);
#endif

    printlk(LK_VERBOSE, "pkt len: %d\n", pkt_len);

    return pkt_len;
}

#ifdef LK
#define BUF_ALLOC(x)            malloc(x)
#define STRUCT_ALLOC(x)         malloc(x)
#define BUF_FREE(x)             free(x)
#define STRUCT_FREE(x)          free(x)
#else
#define BUF_ALLOC(x)            vmalloc(x)
#define STRUCT_ALLOC(x)         kmalloc(x, GFP_KERNEL)
#define BUF_FREE(x)             vfree(x)
#define STRUCT_FREE(x)          kfree(x)
#endif

static int ivshm_endpoint_thread(void *arg)
{
    struct ivshm_endpoint *ep = (struct ivshm_endpoint *) arg;
    ssize_t count = 0;
    char *buf = BUF_ALLOC(ep->max_pkt_size);
    struct ivshm_pkt *pkt = (struct ivshm_pkt *)buf;
    void *buf_end = buf + ep->max_pkt_size;

    if (!buf)
        return -ENOMEM;

    printlk(LK_NOTICE, "%s:%d: buf %s: [%p-%p]\n", __PRETTY_FUNCTION__,
            __LINE__, ep->name, buf, buf_end - 1);
#ifdef LK
    while (ep->thread_running) {
#else
    while (!kthread_should_stop()) {
#endif
        count = ivshm_endpoint_pkt_read(ep, buf, ep->max_pkt_size);
        if (count < 0)
            continue;

        DEBUG_ASSERT(count > 0);
        DEBUG_ASSERT(ep->max_pkt_size >= ((size_t) count));

#ifdef IVSHMEM_MONITOR
        {
            unsigned long long now, latency;
            size_t used = cbuf_space_used(&ep->rx);

            /* Compute circular buffer size */
            if (used > ep->level_max)
                ep->level_max = used;

            /* Compute endpoint latency */
            smp_mb();
            now = current_time_hires() & ((1ULL << 32) - 1);
            latency = now - pkt->hdr.ts;

            if (latency > ep->latency_max) {
                printlk(LK_DEBUG, "endpoint %d: new max latency %llu\n",
                        IVSHM_EP_GET_ID(ep->id), latency);
                ep->latency_max = latency;
            }

            ep->latency_mean = ep->num_rx * ep->latency_mean + latency;
#ifdef LK
            ep->latency_mean /= ++ep->num_rx;
#else
            do_div(ep->latency_mean, ++ep->num_rx);
#endif
        }
#endif
        printlk(LK_VERBOSE, "Received %zd bytes\n", count);
        if (ep->consume) {
            ssize_t consumed;
            consumed = ep->consume(ep, (struct ivshm_pkt *)pkt);
            DEBUG_ASSERT(consumed >= 0);
        }
    }

    printlk(LK_NOTICE, "%s:%d: free up %s buf\n", __PRETTY_FUNCTION__, __LINE__,
            ep->name);
    BUF_FREE(buf);

    return 0;
}

struct ivshm_endpoint *ivshm_endpoint_create_w_private(
    const char *name,
    unsigned id,
    ssize_t (*consume)(struct ivshm_endpoint *ep, struct ivshm_pkt *),
    struct ivshm_info *info,
    size_t bufsize,
    size_t private_size,
    void *private)
{
    struct ivshm_endpoint *ep;
#ifndef LK
    struct sched_param param;
#endif
    int i;

    printlk(LK_VERBOSE, "%s:%d: Creating endpoint %s - id %d\n",
            __PRETTY_FUNCTION__, __LINE__, name, id);

    if (bufsize < IVSHM_EP_MIN_BUFSIZE)
        bufsize = IVSHM_EP_MIN_BUFSIZE;

    if (ivshm_pipe_get_endpoint(info, id)) {
        printlk(LK_ERR, "This endpoint already exists: %d\n", id);
        return NULL;
    }

    ep = STRUCT_ALLOC(sizeof(struct ivshm_endpoint) + private_size);

    if (ep == NULL)
        return ERR_PTR(-ENOMEM);

    /* Set the unique identifier */
    memset(ep, 0, sizeof(struct ivshm_endpoint) + private_size);
    ep->id = IVSHM_EP_DEFAULT(id);
    for (i = 0; i < IVSHM_SCHED_PRIO_NUM_MAX; i++) {
        if (IVSHM_EP_GET_ID(info->prio[i]) == id) {
            printlk(LK_DEBUG, "endpoint id %d has prio %#x with class %#x\n",
                    id,
                    IVSHM_EP_GET_PRIO(info->prio[i]),
                    IVSHM_EP_GET_SCHED(info->prio[i]));
            ep->id = info->prio[i];
        }
    }
    strncpy(ep->name, name, IVSHM_EP_MAX_NAME - 1);
    ep->name[IVSHM_EP_MAX_NAME - 1] = '\0';

    ep->consume = consume;

    /* Initialize circular buffers */
    cbuf_initialize(&ep->rx, bufsize);
#ifdef LK
    sem_init(&ep->rx_pkt_sem, 0);
#else
    sema_init(&ep->rx_pkt_sem, 0);
#endif
    /* Place holder for flags */
    ep->flags = 0;

    /* Save the main ivshmem */
    ep->ivshm = info;

    ep->max_pkt_size = bufsize;
    if (private)
        memcpy(ep->private, private, private_size);

    smp_wmb();

#ifdef LK
    ep->thread_running = true;
    ep->thread = thread_create(
                     ep->name,
                     ivshm_endpoint_thread,
                     (void *) ep,
                     IVSHM_EP_GET_PRIO(ep->id),
                     DEFAULT_STACK_SIZE
                 );

    thread_resume(ep->thread);
#else
    param.sched_priority = IVSHM_EP_GET_PRIO(ep->id);
    ep->thread = kthread_run(ivshm_endpoint_thread, ep, ep->name);

    if (IS_ERR(ep->thread)) {
        return ERR_PTR(-EINVAL);
    }

    sched_setscheduler(ep->thread, IVSHM_EP_GET_SCHED(ep->id), &param);
#endif

    /* Add endpoint to ivshmem */
    ivshm_pipe_add_endpoint(info, ep);

    return ep;
}

struct ivshm_endpoint *ivshm_endpoint_create(
    const char *name,
    unsigned id,
    ssize_t (*consume)(struct ivshm_endpoint *ep, struct ivshm_pkt *),
    struct ivshm_info *info,
    size_t bufsize,
    size_t private_size)
{
    return ivshm_endpoint_create_w_private(
                name,
                id,
                consume,
                info,
                bufsize,
                private_size,
                NULL);
}

void ivshm_endpoint_destroy(struct ivshm_endpoint *ep)
{
    printlk(LK_VERBOSE, "%s:%d: Destroying endpoint %s\n", __PRETTY_FUNCTION__,
            __LINE__, ep->name);

    /* Remove endpoint from list */
    ivshm_pipe_remove_endpoint(ep->ivshm, ep);

    /* Stop thread */
#ifdef LK
#ifdef IVSHM_EP_THREAD_TERMINATION
    int retcode;
    ep->thread_running = false;
    smp_wmb();
    /* FIXME: Needs signal implementation
     * to unblock the lk semaphore
     * */
    thread_join(ep->thread, &retcode, 5000);
#endif
#else
    kthread_stop(ep->thread);
    printlk(LK_VERBOSE, "%s:%d: %s Kernel thread stopped\n", __PRETTY_FUNCTION__,
            __LINE__, ep->name);
#endif

    /* Destroy semaphore */
#ifdef LK
    sem_destroy(&ep->rx_pkt_sem);
#endif

    /* Free cbuf */
    BUF_FREE(ep->rx.buf);

    printlk(LK_VERBOSE, "%s:%d: %s destruction done <----\n", __PRETTY_FUNCTION__,
            __LINE__, ep->name);

    /* Free endpoint */
    STRUCT_FREE(ep);
}
