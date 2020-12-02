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

#ifndef __IVSHMEM_BINARY_H
#define __IVSHMEM_BINARY_H
#include <stddef.h>

#ifdef LK
#define list_head list_node
#include <lib/cbuf.h>
#include <kernel/semaphore.h>
#include <kernel/thread.h>
#else
#include "ivshmem-cbuf.h"
#include <linux/semaphore.h>
#endif

//#define EN_LOOPBACK_TEST 1


#define IVSHM_BIN_SOH     0x01
#define IVSHM_BIN_STX     0x02
#define IVSHM_BIN_EOT     0x04
#define IVSHM_BIN_ACK     0x06
#define IVSHM_BIN_NAK     0x15
#define IVSHM_BIN_CAN     0x18
#define IVSHM_BIN_CTRLZ   0x1A

#define CHUNK_SIZE        (4096)

#define FILE_SIZE_MAX     (2 << 20)
/* 0x454C4946 = FILE (ascii - little endian) */
#define FILE_MAGIC        (0x454C4946)
#define FILE_NAME_MAX     128


/*
 * Reading/Writing file protocol description
 *
 * Control flag packet:
 *       +---------+------+--------+
 * NAME  |  UNUSED | MODE |   OP   |
 *       +---------+------+--------+
 * BIT   | 31 to 5 |  4   | 0 to 3 |
 *       +---------+------+--------+
 *   OP:
 *      - 0: reading from file
 *      - 1: writing to file
 *   MODE (used in writing operation i.e. OP = 1):
 *      - 0: if the file exists, append to file
 *      - 1: if the file exixts, replace with the new file
 *
 * Control response packet:
 *       +---------+--------+------+
 * NAME  |  ACK ID | UNUSED | RESP |
 *       +---------+--------+------+
 * BIT   | 31 to 8 | 7 to 1 |  0   |
 *       +---------+--------+------+
 *   RESP:
 *      - 0: File not found: no such file or directory
 *      - 1: File found
 *   ACK ID:
 *      - Acknowledgement identifier: 0x4B4341
 */

/* Bit define for control command */
#define IVSHM_CMD_OP_SHIFT             0U
#define IVSHM_CMD_OP_MASK              0xFU

#define IVSHM_CMD_OP_VAL_READ          0U
#define IVSHM_CMD_OP_VAL_WRITE         1U
#define IVSHM_CMD_OP_VAL_SIZE          2U

#define IVSHM_CMD_MODE_SHIFT           4U
#define IVSHM_CMD_MODE_MASK            0x10U

#define IVSHM_CMD_MODE_VAL_APPEND      0U
#define IVSHM_CMD_MODE_VAL_CREATE      1U

/* Bit define for control response */
#define IVSHM_RSP_RESP_SHIFT           0U
#define IVSHM_RSP_RESP_MASK            0x1U

#define IVSHM_RSP_RESP_VAL_SUCCESS     0U
#define IVSHM_RSP_RESP_VAL_ERROR       1U

#define IVSHM_RSP_ACK_ID_SHIFT         8U
#define IVSHM_RSP_ACK_ID_MASK          0xFFFFFF00U

/* Acknowledgement identifier for response packet */
#define IVSHM_RSP_ACK_ID_VAL           0x4B4341


#define IVSHM_VAL_TO_FIELD(field, value) \
            (((value) << (field ## _SHIFT)) & (field ## _MASK))
#define IVSHM_FIELD_TO_VAL(field, value) \
            (((value) & (field ## _MASK)) >> (field ## _SHIFT))
#define IVSHM_SET_FIELD(var, field, value) \
            do { \
                var &= ~(field ## _MASK); \
                var |= IVSHM_VAL_TO_FIELD(field, value); \
            } while (0);


enum ivshm_binary_state {
    RESET = 1,
    RECEIVING,
    TRANSMITTING,
    ERROR,
    EOT
};

/* Xmodem header type */
struct binary_pkt {
    char hdr;
    short pkt;
    short n_pkt;
    uint64_t len;
    char payload[];
};

struct ivshm_binary_handler {
    enum ivshm_binary_state state;
#ifdef LK
    cbuf_t cbuf;
    semaphore_t sem_ack;
    semaphore_t sem_queue;
    bool thread_running;
    thread_t *thread;
    struct list_node pkt_list;
    spin_lock_t pkt_lock;
    unsigned flags;
#ifdef EN_LOOPBACK_TEST
    semaphore_t sem_loop;
#endif
#else
    struct rpmsg_cbuf cbuf;
    struct semaphore sem;
#endif
    char last_pkt;
    char ack;
    char retry __attribute__((__aligned__(64)));
};

struct ivshm_info;

/* ivshm binary service API */
int ivshm_init_binary(struct ivshm_info *);
void ivshm_exit_binary(struct ivshm_info *);

struct ivshm_binary_file_pkt {
    uint32_t start_magic;
    uint32_t ctrl_flag;
    char name[FILE_NAME_MAX];
    uint32_t length;
    uint32_t stop_magic;
};

/* ivshm binary service helper function */
void ivshm_binary_list(void);

#endif
