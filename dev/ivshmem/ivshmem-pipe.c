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

#ifdef LK
#include <sys/types.h>
#include <dev/interrupt.h>
#include <reg.h>
#include <stdio.h>
#include <kernel/vm.h>
#include <kernel/semaphore.h>
#include <lk/init.h>
#include <dev/ivshm.h>
#include <err.h>
#include <lib/appargs.h>

/*
 * I do not understand yet, but virtio_ring.h from LK is not aligned with Linux
#include <dev/virtio/virtio_ring.h>
*/
#include <lib/cbuf.h>
#include <iovec.h>
#include <string.h>


#define swap(a, b) \
    do { typeof(a) __tmp = (a); (a) = (b); (b) = __tmp; } while (0)


#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })


#define virt_load_acquire(x) smp_load_acquire(x); smp_mb()
#define virt_store_release(x, y) smp_store_release(x, y); smp_mb()
#define SMP_CACHE_BYTES CACHE_LINE
#define virt_mb smp_mb
#define virt_wmb smp_wmb

#define __iomem

#define ivshmem_atomic_add(a, v) atomic_add(a, v)
#define ivshmem_atomic_dec(a, v) atomic_add(a, -v)
#define ivshmem_atomic_read(p) (*p)

#else
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,5,0)
#include <uapi/linux/sched/types.h>
#endif
#include <asm/div64.h>

#include "ivshm.h"

#include "ivshmem-iovec.h"
#include "ivshmem-cbuf.h"

#define DEBUG_ASSERT(cond) BUG_ON(!(cond))
#define ERR_TIMED_OUT 0
#define spin_lock_saved_state_t unsigned long
#define ivshmem_atomic_add(a, v) atomic_add(v, a)
#define ivshmem_atomic_dec(a, v) atomic_sub(v, a)
#define ivshmem_atomic_read(p) atomic_read(p)

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,5,0)
/* Resolve missing headers on kernel 4.1.10 */
#define virt_load_acquire(x) smp_load_acquire(x)
#define virt_store_release(x, y) smp_store_release(x, y)
#define virt_mb smp_mb
#define virt_wmb smp_wmb
#endif
#endif

#include "ivshmem-pipe.h"
#include "debug.h"

#define IVSHM_VQ_ALIGN 128
#define IVSHM_FRAME_SIZE(s) ALIGN(0+ (s), SMP_CACHE_BYTES)

#ifndef LK
#include <asm/arch_timer.h>
#endif

static uint32_t ivshm_v1_get_id(struct ivshm_info *info)
{
    return readl(info->io + IVSHM_V1_REG_IVPOS);
}

static uint32_t ivshm_v1_get_rstate(struct ivshm_info *info)
{
    return readl(info->io + IVSHM_V1_REG_RSTATE);
}

static uint32_t ivshm_v1_get_max_peers(struct ivshm_info *info)
{
    return 1;
}

static void ivshm_v1_set_state(struct ivshm_info *info, uint32_t state)
{
    writel(state, info->io + IVSHM_V1_REG_LSTATE);
}

static void ivshm_v1_notify_peer(struct ivshm_info *info)
{
        writel(info->peer_id << 16, info->io + IVSHM_V1_REG_DBELL);
}

static void ivshm_v1_set_intctrl(struct ivshm_info *info, uint32_t val)
{
    writel(val, info->io + IVSHM_V1_REG_INTX_CTRL);
}

static uint32_t ivshm_v2_get_id(struct ivshm_info *info)
{
    return readl(info->io + IVSHM_V2_REG_ID);
}

static uint32_t ivshm_v2_get_rstate(struct ivshm_info *info)
{
    return READ_ONCE(info->state_table[info->peer_id]);
}

static uint32_t ivshm_v2_get_max_peers(struct ivshm_info *info)
{
    return readl(info->io + IVSHM_V2_REG_MAX_PEERS);
}

static void ivshm_v2_set_state(struct ivshm_info *info, uint32_t state)
{
    writel(state, info->io + IVSHM_V2_REG_STATE);
}

static void ivshm_v2_notify_peer(struct ivshm_info *info)
{
        writel(info->peer_id << 16, info->io + IVSHM_V2_REG_DBELL);
}

static void ivshm_v2_set_intctrl(struct ivshm_info *info, uint32_t val)
{
    writel(val, info->io + IVSHM_V2_REG_INT_CTRL);
}

static struct ivshm_reg_cb ivshm_reg_v1_cb = {
    .get_id = ivshm_v1_get_id,
    .get_rstate = ivshm_v1_get_rstate,
    .get_max_peers = ivshm_v1_get_max_peers,
    .set_state = ivshm_v1_set_state,
    .notify_peer = ivshm_v1_notify_peer,
    .set_intctrl = ivshm_v1_set_intctrl,
};

static struct ivshm_reg_cb ivshm_reg_v2_cb = {
    .get_id = ivshm_v2_get_id,
    .get_rstate = ivshm_v2_get_rstate,
    .get_max_peers = ivshm_v2_get_max_peers,
    .set_state = ivshm_v2_set_state,
    .notify_peer = ivshm_v2_notify_peer,
    .set_intctrl = ivshm_v2_set_intctrl,
};

/* Version agnostic accessors */
static inline uint32_t ivshm_get_id(struct ivshm_info *info)
{
    return info->cb->get_id(info);
}

static inline uint32_t ivshm_get_rstate(struct ivshm_info *info)
{
    return info->cb->get_rstate(info);
}

static inline uint32_t ivshm_get_max_peers(struct ivshm_info *info)
{
    return info->cb->get_max_peers(info);
}

static inline void ivshm_set_state(struct ivshm_info *info, uint32_t state)
{
    info->cb->set_state(info, state);
}

static inline void ivshm_notify_peer(struct ivshm_info *info)
{
    info->cb->notify_peer(info);
}

static inline void ivshm_set_intctrl(struct ivshm_info *info, uint32_t val)
{
    info->cb->set_intctrl(info, val);
}

static inline void ivshm_pkt_write(struct ivshm_pkt *pkt, void *buf)
{
    *(struct ivshm_pkt_hdr *) buf = pkt->hdr;
    buf += sizeof(struct ivshm_pkt_hdr);
    memcpy(buf, pkt->payload, pkt->hdr.len);
}

static inline size_t ivshm_pkt_len(const iovec_t *iov, uint iov_cnt)
{
    return sizeof(struct ivshm_pkt_hdr) + iovec_size(iov, iov_cnt);

}

static bool ivshm_pipe_check_state(struct ivshm_info *info)
{
    u32 rstate = ivshm_get_rstate(info);

    if ((rstate != info->rstate) || (info->lstate != IVSHM_PIPE_STATE_RUN)) {
        printlk(LK_VERBOSE, "%s:%d: ivshm pipe state changed\n", __PRETTY_FUNCTION__, __LINE__);
        return true;
    }


    return false;
}

static const char *ivshm_get_pipe_state_name(u32 state)
{
    static const char *ivshm_pipe_name[] = {
        "IVSHM PIPE STATE RESET",
        "IVSHM PIPE STATE INIT",
        "IVSHM PIPE STATE READY",
        "IVSHM PIPE STATE RUN",
    };

    if (state < ARRAY_SIZE(ivshm_pipe_name))
        return ivshm_pipe_name[state];
    else
        return "Unknown";
}

static void ivshm_pipe_set_state(struct ivshm_info *info, u32 state)
{
    printlk(LK_VERBOSE, "%s:%d: Set Pipe state to %s\n", __PRETTY_FUNCTION__,
            __LINE__, ivshm_get_pipe_state_name(state));

    smp_wmb();
    WRITE_ONCE(info->lstate, state);
    ivshm_set_state(info, state);
}

static void ivshm_init_queue(struct ivshm_info *in,
                             struct ivshm_queue *q,
                             void *mem, unsigned int len)
{
    memset(q, 0, sizeof(*q));

    vring_init(&q->vr, len, mem, IVSHM_VQ_ALIGN);
    q->data = mem + in->vrsize;
    q->end = q->data + in->qsize;
    q->size = in->qsize;

    printlk(LK_VERBOSE, "%s:%d: Queue %p: %#x descriptors, buffer [%p-%p] (%#x)\n",
            __PRETTY_FUNCTION__, __LINE__, q, len, q->data, q->end, q->size);
}

static void ivshm_init_queues(struct ivshm_info *info)
{
    unsigned int i;
    void *tx;
    void *rx;
    u32 id = ivshm_get_id(info);

    tx = info->shm +  id * info->shmlen / 2;
    rx = info->shm + !id * info->shmlen / 2;

    printlk(LK_VERBOSE, "%s:%d: Initializing queues (%x): %zd @ %p/%p\n",
            __PRETTY_FUNCTION__, __LINE__, id, info->shmlen / 2, rx, tx);

    memset(tx, 0, info->shmlen / 2);

    ivshm_init_queue(info, &info->rx, rx, info->qlen);
    ivshm_init_queue(info, &info->tx, tx, info->qlen);

    swap(info->rx.vr.used, info->tx.vr.used);

    info->tx.num_free = info->tx.vr.num;

    for (i = 0; i < info->tx.vr.num - 1; i++)
        info->tx.vr.desc[i].next = i + 1;
}

static int ivshm_calc_qsize(struct ivshm_info *in)
{
    unsigned int vrsize;
    unsigned int qsize;
    unsigned int qlen;

    for (qlen = 4096; qlen > 32; qlen >>= 1) {
        vrsize = vring_size(qlen, IVSHM_VQ_ALIGN);
        vrsize = ALIGN(vrsize, IVSHM_VQ_ALIGN);
        if (vrsize < in->shmlen / 16)
            break;
    }

    if (vrsize > in->shmlen / 2)
        return -1;

    qsize = in->shmlen / 2 - vrsize;
    /* Add a check against a minimal qsize ?*/

    in->vrsize = vrsize;
    in->qlen = qlen;
    in->qsize = qsize;

    printlk(LK_VERBOSE, "%s:%d: vrsize: %d, qlen: %d, qsize: %d\n",
            __PRETTY_FUNCTION__, __LINE__, vrsize, qlen, qsize);

    return 0;
}

static void *ivshm_desc_data(struct ivshm_info *in,
                             struct ivshm_queue *q,
                             struct vring_desc *desc,
                             u32 *len)
{
    u64 offs = READ_ONCE(desc->addr);
    u32 dlen = READ_ONCE(desc->len);
    u16 flags = READ_ONCE(desc->flags);
    void *data;

    printlk(LK_VERBOSE, "flags: %x, offs: %llx, shmlen: %zx, qend: %p qdata: %p\n",
            flags, offs, in->shmlen, q->end, q->data);

    if (flags)
        return NULL;

    if (offs >= in->shmlen)
        return NULL;

    data = in->shm + offs;

    if (data < q->data || data >= q->end)
        return NULL;

    if (dlen > q->end - data)
        return NULL;

    *len = dlen;

    return data;
}

static struct vring_desc *ivshm_rx_desc(struct ivshm_info *in)
{
    struct ivshm_queue *rx = &in->rx;
    struct vring *vr = &rx->vr;
    unsigned int avail;
    u16 avail_idx;

    avail_idx = virt_load_acquire(&vr->avail->idx);

    if (avail_idx == rx->last_avail_idx)
        return NULL;

    avail = vr->avail->ring[rx->last_avail_idx++ & (vr->num - 1)];
    if (avail >= vr->num) {
        printlk(LK_ERR, "%s:%d: invalid rx avail %d\n", __PRETTY_FUNCTION__,
                __LINE__, avail);
        return NULL;
    }

    virt_mb();

    return &vr->desc[avail];
}

static void ivshm_enable_rx_irq(struct ivshm_info *in)
{
    vring_avail_event(&in->rx.vr) = in->rx.last_avail_idx;
    virt_wmb();
}

static void ivshm_rx_finish(struct ivshm_info *in, struct vring_desc *desc)
{
    struct ivshm_queue *rx = &in->rx;
    struct vring *vr = &rx->vr;
    unsigned int desc_id = desc - vr->desc;
    unsigned int used;

    used = rx->last_used_idx++ & (vr->num - 1);
    vr->used->ring[used].id = desc_id;
    vr->used->ring[used].len = 1;

    virt_store_release(&vr->used->idx, rx->last_used_idx);
}

static bool ivshm_rx_avail(struct ivshm_info *in)
{
    virt_mb();
    return READ_ONCE(in->rx.vr.avail->idx) != in->rx.last_avail_idx;
}
#ifdef WITH_IVSHMEM_NOTIFY_RX
static void ivshm_notify_rx(struct ivshm_info *in, unsigned int num)
{
    u16 evt, old, new;

    virt_mb();

    evt = vring_used_event(&in->rx.vr);
    old = in->rx.last_used_idx - num;
    new = in->rx.last_used_idx;

    if (vring_need_event(evt, new, old)) {
        smp_wmb();
        printlk(LK_VERBOSE, "%s:%d: Generating a rx event\n",
                __PRETTY_FUNCTION__, __LINE__);
        ivshm_notify_peer(in);
    }
}
#endif

static void ivshm_enable_tx_irq(struct ivshm_info *in)
{
    printlk(LK_VERBOSE, "%s:%d: last_used_idx: %x\n",
            __PRETTY_FUNCTION__, __LINE__, in->tx.last_used_idx);
    vring_used_event(&in->tx.vr) = in->tx.last_used_idx;
    virt_wmb();
}

static u32 ivshm_tx_advance(struct ivshm_queue *q, u32 *pos, u32 len)
{
    u32 p = *pos;

    len = IVSHM_FRAME_SIZE(len);

    if (q->size - p < len)
        p = 0;
    *pos = p + len;

    return p;
}

static void ivshm_tx_clean(struct ivshm_info *in)
{
    struct ivshm_queue *tx = &in->tx;
    struct vring_used_elem *used;
    struct vring *vr = &tx->vr;
    struct vring_desc *desc;
    struct vring_desc *fdesc;
    unsigned int num;
    u16 used_idx;
    u16 last;
    u32 fhead;

    spin_lock_saved_state_t state;
    spin_lock_irqsave(&in->tx_clean_lock, state);

    used_idx = virt_load_acquire(&vr->used->idx);
    last = tx->last_used_idx;

    fdesc = NULL;
    fhead = 0;
    num = 0;

    printlk(LK_VERBOSE, "-->used_idx: %x, last: %x\n", used_idx, last);

    while (last != used_idx) {
        void *data;
        u32 len;
        u32 tail;
        u32 pos = tx->tail;;

        used = vr->used->ring + (last % vr->num);
        if (used->id >= vr->num || used->len != 1) {
            printlk(LK_ERR, "%s:%d: invalid tx used->id %d ->len %d\n",
                    __PRETTY_FUNCTION__, __LINE__, used->id, used->len);
            break;
        }

        printlk(LK_VERBOSE, "used_idx: %x, last: %x, used: %p\n", used_idx, last, used);

        desc = &vr->desc[used->id];

        data = ivshm_desc_data(in, &in->tx, desc, &len);
        if (!data) {
            printlk(LK_ERR, "%s:%d: bad tx descriptor, data == NULL %d\n",
                    __PRETTY_FUNCTION__, __LINE__, used->id);
            break;
        }

        tail = ivshm_tx_advance(tx, &pos, len);
        if (data != tx->data + tail) {
            printlk(LK_ERR, "%s:%d: bad tx descriptor %d\n",
                    __PRETTY_FUNCTION__, __LINE__, used->id);
            break;
        }
        tx->tail = pos;

        if (!num)
            fdesc = desc;
        else
            desc->next = fhead;

        fhead = used->id;
        last++;
        num++;
    }

    tx->last_used_idx = last;
    printlk(LK_VERBOSE, "Last used idx: %x\n", tx->last_used_idx);
    virt_mb();
    spin_unlock_irqrestore(&in->tx_clean_lock, state);

    if (num) {
        spin_lock_irqsave(&in->tx_free_lock, state);
        fdesc->next = tx->free_head;
        tx->free_head = fhead;
        tx->num_free += num;

        BUG_ON(tx->num_free > vr->num);
        virt_mb();
        spin_unlock_irqrestore(&in->tx_free_lock, state);
    }
}

static size_t ivshm_tx_space(struct ivshm_info *in)
{
    struct ivshm_queue *tx = &in->tx;
    u32 tail = tx->tail;
    u32 head = tx->head;
    u32 space;

    if (head < tail)
        space = tail - head;
    else
        space = max(tx->size - head, tail);

    return space;
}

static bool ivshm_tx_ok(struct ivshm_info *in, size_t count)
{
    return in->tx.num_free >= 1 &&
           ivshm_tx_space(in) >= 1 * IVSHM_FRAME_SIZE(count);
}

static void ivshm_notify_tx(struct ivshm_info *in, unsigned int num)
{
    u16 evt, old, new;

    virt_mb();

    evt = READ_ONCE(vring_avail_event(&in->tx.vr));
    old = in->tx.last_avail_idx - num;
    new = in->tx.last_avail_idx;

    printlk(LK_VERBOSE, "evt: %x, new: %x old: %x\n", evt, new, old);
    if (vring_need_event(evt, new, old)) {
        smp_wmb();
        printlk(LK_VERBOSE, "Generating a tx event\n");
//        printlk(LK_INFO, "%s:%d: Generating a tx event\n", __PRETTY_FUNCTION__, __LINE__);
        ivshm_notify_peer(in);
    }
}

static ssize_t ivshm_tx_frame(struct ivshm_info *in, const iovec_t *iov, uint iov_cnt)
{
    struct ivshm_queue *tx = &in->tx;
    struct vring *vr = &tx->vr;
    struct vring_desc *desc;
    unsigned int desc_idx;
    unsigned int avail;
    u32 head;
    void *buf;
    spin_lock_saved_state_t state;
    size_t count = iovec_size(iov, iov_cnt);

    spin_lock_irqsave(&in->tx_free_lock, state);

    BUG_ON(tx->num_free < 1);
    desc_idx = tx->free_head;
    desc = &vr->desc[desc_idx];
    tx->free_head = desc->next;
    tx->num_free--;

    head = ivshm_tx_advance(tx, &tx->head, count);
    buf = tx->data + head;

    printlk(LK_VERBOSE, "head: %x, buf: %p, count: %zx\n", head, buf, count);
    iovec_to_membuf(buf, count, iov, iov_cnt, 0);

    desc->addr = buf - in->shm;
    desc->len = count;
    desc->flags = 0;

    avail = tx->last_avail_idx++ & (vr->num - 1);
    vr->avail->ring[avail] = desc_idx;
    tx->num_added++;

    virt_store_release(&vr->avail->idx, tx->last_avail_idx);

    ivshm_notify_tx(in, tx->num_added);
    tx->num_added = 0;

    spin_unlock_irqrestore(&in->tx_free_lock, state);

    return count;
}

static int ivshm_pipe_isr(struct ivshm_info *info)
{
    bool state_change, data_available;
    spin_lock(&info->lock);

    state_change = ivshm_pipe_check_state(info);
    data_available =
        (info->lstate == IVSHM_PIPE_STATE_RUN) && (ivshm_rx_avail(info));

    if (data_available)
        ivshmem_atomic_add(&info->data_available, 1);

    if (state_change)
        ivshmem_atomic_add(&info->state_change, 1);

    smp_wmb();

    if (data_available || state_change) {
#ifdef LK
        int ret = sem_post(&info->semwait, false);
        if (0 == ret)
            printlk(LK_NOTICE, "%s:%d: WARNING: sem posted but nobody pending at this time (LK stuck ?)\n",
                    __PRETTY_FUNCTION__, __LINE__);
#else
        wake_up_interruptible(&info->wait);
#endif
    } else {
        printlk(LK_NOTICE,"%s:%d: Spurious ivshmem interrupt (data_available=%d, state_change=%d)\n",
                __PRETTY_FUNCTION__, __LINE__,
                ivshmem_atomic_read(&info->data_available),
                ivshmem_atomic_read(&info->state_change));
    }

    spin_unlock(&info->lock);

    return 0;
}

bool ivshm_pipe_is_running(struct ivshm_info *info)
{
    return info->lstate >= IVSHM_PIPE_STATE_RUN;
}

#ifdef LK
#define IVSHM_PIPE_HANDLER_RETURN INT_RESCHEDULE
static enum handler_return ivshm_pipe_handler(void *arg)
#else
#define IVSHM_PIPE_HANDLER_RETURN IRQ_HANDLED
static irqreturn_t ivshm_pipe_handler(int irq, void *arg)
#endif
{
    struct ivshm_info *info = arg;

    ivshm_pipe_isr(info);

    return IVSHM_PIPE_HANDLER_RETURN;
}


static void ivshm_pipe_fsm(struct ivshm_info *info)
{
    u32 rstate = ivshm_get_rstate(info);

    printlk(LK_VERBOSE, "%s:%d: state: %s\n", __PRETTY_FUNCTION__, __LINE__,
            ivshm_get_pipe_state_name(info->lstate));
    switch (info->lstate) {
        case IVSHM_PIPE_STATE_RESET:
            if ((rstate) < IVSHM_PIPE_STATE_READY) {
                ivshm_pipe_set_state(info, IVSHM_PIPE_STATE_INIT);
            }
            break;
        case IVSHM_PIPE_STATE_INIT:
            if ((rstate) > IVSHM_PIPE_STATE_RESET ) {
                ivshm_init_queues(info);
                ivshm_pipe_set_state(info, IVSHM_PIPE_STATE_READY);
            }

            break;
        case IVSHM_PIPE_STATE_READY:
            if ((rstate) >= IVSHM_PIPE_STATE_READY ) {
                printlk(LK_NOTICE, "%s:%d: Initialization IVSHM services...\n",
                        __PRETTY_FUNCTION__, __LINE__);
                ivshm_init_services(info);
                ivshm_pipe_set_state(info, IVSHM_PIPE_STATE_RUN);
            }

            break;
        case IVSHM_PIPE_STATE_RUN:
            if ((rstate) == IVSHM_PIPE_STATE_RESET ) {
                printlk(LK_NOTICE, "%s:%d: Disabling IVSHM services...\n",
                        __PRETTY_FUNCTION__, __LINE__);
                ivshm_disable_services(info);
                ivshm_pipe_set_state(info, IVSHM_PIPE_STATE_RESET);
            }
            break;
    }

    smp_wmb();
    WRITE_ONCE(info->rstate, rstate);
}

static int ivshm_pipe_poll(struct ivshm_info *in)
{
    int received = 0;

    ivshm_tx_clean(in);
    do {
        struct vring_desc *desc;
        void *data;
        u32 len;

        desc = ivshm_rx_desc(in);
        if (!desc)
            break;

        data = ivshm_desc_data(in, &in->rx, desc, &len);
        if (!data) {
            printlk(LK_NOTICE, "%s:%d: bad rx descriptor\n",
                    __PRETTY_FUNCTION__, __LINE__);
            break;
        }

        ivshm_endpoint_push(in, data, len);

        ivshm_rx_finish(in, desc);
        received++;
        ivshm_enable_rx_irq(in);
    } while (ivshm_rx_avail(in));

#ifdef WITH_IVSHMEM_NOTIFY_RX
    if (received)
        ivshm_notify_rx(in, received);
#endif

    return received;
}

static int ivshm_pipe_thread(void *arg)
{
    struct ivshm_info *info = (struct ivshm_info *) arg;
    bool state_change, data_available;
    spin_lock_saved_state_t irq_state;
    int ret;

    printlk(LK_NOTICE, "%s: entry\n", __PRETTY_FUNCTION__);

#ifdef LK
    bool thread_running = true;

    while (thread_running) {
        ret = sem_timedwait(&info->semwait, 1000);
#else
    while (!kthread_should_stop()) {
        ret = wait_event_interruptible_timeout(info->wait,
                        (ivshmem_atomic_read(&info->state_change) > 0)
                        || (ivshmem_atomic_read(&info->data_available) > 0)
                        || kthread_should_stop(), HZ);

        if (kthread_should_stop()) {
            printlk(LK_NOTICE, "%s:%d: Kthread asked to terminate\n",
                    __PRETTY_FUNCTION__, __LINE__);
            break;
        }

        if (ret < 0) {
            printlk(LK_NOTICE, "%s:%d: Kthread termination signal received\n",
                    __PRETTY_FUNCTION__, __LINE__);
            return -ERESTARTSYS;
        }
#endif
        if (ret == ERR_TIMED_OUT && info->lstate == IVSHM_PIPE_STATE_RUN) {
            printlk(LK_VERBOSE, "%s:%d: ivshmem thread timeout\n",
                    __PRETTY_FUNCTION__, __LINE__);
            data_available = true;
        }

        spin_lock_irqsave(&info->lock, irq_state);
        if (ivshmem_atomic_read(&info->state_change) > 0) {
            state_change = true;
            ivshmem_atomic_dec(&info->state_change, 1);
        }

        if (ivshmem_atomic_read(&info->data_available) > 0) {
            data_available = true;
            ivshmem_atomic_dec(&info->data_available, 1);
        }
        smp_wmb();

        spin_unlock_irqrestore(&info->lock, irq_state);

        if (state_change) {
            ivshm_pipe_fsm(info);
            state_change = false;
        }

        if (data_available) {
            ivshm_pipe_poll(info);
            data_available = false;
        } else {
            printlk(LK_VERBOSE, "%s:%d: No ivshmem data available ?\n",
                    __PRETTY_FUNCTION__, __LINE__);
        }

    }

#ifdef LK
    sem_destroy(&info->semwait);
#endif

    printlk(LK_NOTICE, "%s: exit\n", __PRETTY_FUNCTION__);

    return 0;
}

int ivshm_pipe_init(struct ivshm_info *info)
{
    int ret;
    u32 ivpos;

#ifndef LK
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,8,0)
    struct sched_param param = {.sched_priority = MAX_RT_PRIO - 1,};
#endif
#endif

    /* Not using revision of cb index array saves some cycles on registors
     * accesses
     */
    switch (info->revision) {
    case 1:
        info->cb = &ivshm_reg_v1_cb;
        break;
    case 2:
        info->cb = &ivshm_reg_v2_cb;
        break;
    default:
        printlk(LK_ERR, "%s:%d: Incorrect ivshm revision (%d), fallback to rev 2\n",
                __PRETTY_FUNCTION__, __LINE__, info->revision);
        info->cb = &ivshm_reg_v2_cb;
    }

    spin_lock_init(&info->tx_free_lock);
    spin_lock_init(&info->tx_clean_lock);
    spin_lock_init(&info->ep_lock);
    spin_lock_init(&info->lock);

#ifdef LK
    list_initialize(&info->endpoints_list);
    ret = 0;
    sem_init(&info->semwait, 0);

    info->thread = thread_create(
                       "ivshm-pipe",
                       ivshm_pipe_thread,
                       (void *) info,
                       HIGH_PRIORITY - 1,
                       DEFAULT_STACK_SIZE);

    thread_detach_and_resume(info->thread);
#else
    INIT_LIST_HEAD(&info->endpoints_list);
    init_waitqueue_head(&info->wait);

    ret = devm_request_irq(&info->pdev->dev, info->irq,
                            ivshm_pipe_handler, IRQF_SHARED, "ivshm", info);
    if (ret) {
        printlk(LK_ERR, "%s:%d: Requesting irq failed\n", __PRETTY_FUNCTION__,
                __LINE__);
        return -ENODEV;
    }

    info->thread = kthread_run(
                       ivshm_pipe_thread, info, "ivshm-pipe");

    if (IS_ERR(info->thread)) {
        return -EINVAL;
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,8,0)
    sched_set_fifo(info->thread);
#else
    sched_setscheduler(info->thread, SCHED_FIFO, &param);
#endif
#endif

    /* Common to all OSes */

    ivpos = ivshm_get_id(info);

    if (ivpos > 1) {
        printlk(LK_ERR, "%s:%d: invalid IVPosition %d\n", __PRETTY_FUNCTION__,
                __LINE__, ivpos);
        return -1;
    }

    info->peer_id = !ivpos;

    printlk(LK_NOTICE, "%s:%d: IRQ: %#x\n", __PRETTY_FUNCTION__, __LINE__,
            info->irq);
    printlk(LK_NOTICE, "%s:%d: IO Mapping: %p\n", __PRETTY_FUNCTION__, __LINE__,
            info->io);
    printlk(LK_NOTICE, "%s:%d: SHM Mapping: [%p-%p]\n", __PRETTY_FUNCTION__,
            __LINE__, info->shm, info->shm + info->shmlen - 1);
    printlk(LK_NOTICE, "%s:%d: IVpos: %d\n", __PRETTY_FUNCTION__, __LINE__,
            ivpos);

    ivshm_set_intctrl(info, 1);

    ivshm_calc_qsize(info);

    ivshm_pipe_set_state(info, IVSHM_PIPE_STATE_RESET);

    ret = ivshm_pipe_check_state(info);

    if (ret) {
        spin_lock_saved_state_t irq_state;
        spin_lock_irqsave(&info->lock, irq_state);
        ivshmem_atomic_add(&info->state_change, 1);
        spin_unlock_irqrestore(&info->lock, irq_state);
#ifdef LK
        sem_post(&info->semwait, false);
#else
        wake_up_interruptible(&info->wait);
#endif
    }

    return 0;
}

void ivshm_pipe_remove(struct ivshm_info *info)
{
    /* TODO free resources */
#ifndef LK
    devm_free_irq(&info->pdev->dev, info->irq, info);
#ifdef __arm__
    info->irq = NO_IRQ;
#else
    /* based on this thread
     * https://lore.kernel.org/patchwork/patch/598967/
     */
    info->irq = 0;
#endif
    kthread_stop(info->thread);
#else
//    TODO
#endif
}

int ivshm_write(struct ivshm_info *in, const iovec_t *iov, uint iov_cnt)
{
    ssize_t sz, sent;

    ivshm_tx_clean(in);

    sz = iovec_size(iov, iov_cnt);
    if (sz < 0)
        return 0;

    if (!ivshm_tx_ok(in, sz)) {
        ivshm_enable_tx_irq(in);
    }

    sent = ivshm_tx_frame(in, iov, iov_cnt);

    return sent;
}

#ifdef LK
static void ivshm_pipe_init_lk(uint level)
{
    static struct ivshm_info ivshm_info_data;

    ivshm_info_data.dev = ivshm_register(0, ivshm_pipe_handler, &ivshm_info_data);
    assert(ivshm_info_data.dev != NULL);
    ivshm_info_data.revision = ivshm_info_data.dev->revision;

    ivshm_info_data.state_table = ivshm_info_data.dev->state_table;
    /* TODO: Remove io */
    ivshm_info_data.io = ivshm_info_data.dev->registers;
    ivshm_info_data.shm = ivshm_info_data.dev->rw_section;
    ivshm_info_data.shmlen = ivshm_info_data.dev->rw_section_sz;
    ivshm_info_data.irq = ivshm_info_data.dev->irq;

    /* FIXME handle multiple shared mem */
    memset(&ivshm_info_data.prio, 0, sizeof(ivshm_info_data.prio));
    int node = of_get_node_by_path("/ivshmem");
    if (node > 0)
        of_get_int_array(node, "prio", ivshm_info_data.prio, IVSHM_SCHED_PRIO_NUM_MAX);

    ivshm_pipe_init(&ivshm_info_data);
}
LK_INIT_HOOK(ivshm_pipe, ivshm_pipe_init_lk, LK_INIT_LEVEL_VM + 2);
#endif
