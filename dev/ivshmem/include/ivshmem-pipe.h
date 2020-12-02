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

#ifndef __IVSHMEM_PIPE_H
#define __IVSHMEM_PIPE_H

/* Adapting linux kernel code to Little Kernel */
#ifdef LK
#include <sys/types.h>
#include "virtio_ring.h"
#include <iovec.h>
#include <kernel/spinlock.h>
#include <kernel/thread.h>
#include <kernel/semaphore.h>

#define __iomem
#define list_head list_node
#define atomic_t int

#else
#include <linux/device.h>
#include <linux/virtio_ring.h>
#include "ivshmem-iovec.h"

#endif

struct ivshm_queue {
    struct vring vr;
    u32 free_head;
    u32 num_free;
    u32 num_added;
    u16 last_avail_idx;
    u16 last_used_idx;

    void *data;
    void *end;
    u32 size;
    u32 head;
    u32 tail;
};

#define IVSHM_SCHED_PRIO_NUM_MAX    32

struct ivshm_info;

struct ivshm_reg_cb {
    uint32_t (*get_id)(struct ivshm_info *);
    uint32_t (*get_rstate)(struct ivshm_info *);
    uint32_t (*get_max_peers)(struct ivshm_info *);
    void (*set_state)(struct ivshm_info *, uint32_t);
    void (*notify_peer)(struct ivshm_info *);
    void (*set_intctrl)(struct ivshm_info *, uint32_t);
};

struct ivshm_info {
    uint32_t revision;
    const char *device_name;
    void  __iomem *io;
    u32 __iomem *state_table;
    void  __iomem *shm;
    struct ivshm_reg_cb *cb;
    size_t shmlen;
    int irq;
    u32 vrsize;
    u32 qlen;
    u32 qsize;

    struct ivshm_queue tx;
    struct ivshm_queue rx;
#define IVSHM_PIPE_STATE_RESET    0x0
#define IVSHM_PIPE_STATE_INIT     0x1
#define IVSHM_PIPE_STATE_READY    0x2
#define IVSHM_PIPE_STATE_RUN      0x3
    u32 lstate;
    u32 rstate;
    u32 peer_id;
    atomic_t state_change;
    atomic_t data_available;
    struct list_head endpoints_list;
    /* OS specific */
#ifdef LK
    spin_lock_t tx_free_lock;
    spin_lock_t tx_clean_lock;
    spin_lock_t ep_lock;
    spin_lock_t lock;
    struct ivshm_dev_data *dev;
    thread_t *thread;
    semaphore_t semwait;
#else
    spinlock_t tx_free_lock;
    spinlock_t tx_clean_lock;
    spinlock_t ep_lock;
    spinlock_t lock;

    struct resource resource[2];
    struct pci_dev *pdev;
    struct task_struct *thread;
    wait_queue_head_t wait;
#ifdef CONFIG_DEBUG_FS
    struct dentry *debug;
#endif
#endif
    unsigned prio[IVSHM_SCHED_PRIO_NUM_MAX];
};

#define IVSHM_PKT_MAGIC 0x78746B70      /* 'pktx' */
#define IVSHM_PKT_PAD_MAGIC 0x58444150  /* 'PADX' */

struct ivshm_pkt_hdr {
    unsigned magic; /* 'pktx' */
    unsigned id;    /* id */
    unsigned len;     /* Payload length */
    unsigned pad;
#ifdef IVSHMEM_MONITOR
    unsigned long long ts;
#endif
};

struct ivshm_pkt {
    struct ivshm_pkt_hdr hdr;
    char payload[];  /* Payload */
};

static inline size_t ivshm_pkt_get_payload_length(struct ivshm_pkt *pkt)
{
    if (pkt->hdr.len  < sizeof(struct ivshm_pkt))
        return 0;

    return pkt->hdr.len - sizeof(struct ivshm_pkt);
}

int ivshm_pipe_init(struct ivshm_info *);
bool ivshm_pipe_is_running(struct ivshm_info *info);
void ivshm_pipe_remove(struct ivshm_info *);
int ivshm_write(struct ivshm_info *, const iovec_t *iov, uint iov_cnt);
int ivshm_read(struct ivshm_info *, void *, size_t);

int ivshm_endpoint_push(struct ivshm_info *in, void *buf, size_t len);
int ivshm_init_services(struct ivshm_info *info);
void ivshm_disable_services(struct ivshm_info *info);

#endif
