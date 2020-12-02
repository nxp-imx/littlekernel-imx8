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

#ifndef __IVSHMEM_RPC_H
#define __IVSHMEM_RPC_H

#include "ivshmem-pipe.h"


#define IVSHM_RPC_ID(sz_in, sz_out, id) \
        (\
            ((sz_in & 0x3FF) << 22) | \
            ((sz_out & 0x3FF) << 12) | \
            (id & 0xFFF)\
        )

#define IVSHM_RPC_GET_ID(x) (x & 0xFFF)
#define IVSHM_RPC_GET_SZ_IN(x) ((x >> 22) & 0x3FF)
#define IVSHM_RPC_GET_SZ_OUT(x) ((x >> 12) & 0x3FF)


#ifdef LK
#define IVSHM_RPC_CALLBACK(_id, _fn) \
    static int _fn(struct ivshm_rpc_service *s, \
                   struct rpc_client_callback *cb, \
                   void *payload); \
    struct rpc_client_callback rpc_cb_##_fn = { .id = _id, .fn = _fn }
#else
#define RPMSG_RPC_CALLBACK(_id, _fn) \
    static int _fn(struct rpmsg_rpc_dev *rpcdev, \
                   struct rpc_client_callback *cb, \
                   void *payload); \
    struct rpc_client_callback rpc_cb_##_fn = { .id = _id, .fn = _fn }
#endif

#define RPC_DAC_CMD                     0x100
#define RPC_HDMI_CMD                    0x200
#define RPC_ADC_CMD                     0x300


/* RPC DAC commands */
#define RPC_DAC_INIT_ID                 IVSHM_RPC_ID(0,  4, (RPC_DAC_CMD | 0x01))
#define RPC_DAC_OPEN_ID                 IVSHM_RPC_ID(0,  4, (RPC_DAC_CMD | 0x02))
#define RPC_DAC_CLOSE_ID                IVSHM_RPC_ID(0,  4, (RPC_DAC_CMD | 0x03))
#define RPC_DAC_G_CAP_ID                IVSHM_RPC_ID(0,  4, (RPC_DAC_CMD | 0x21))
#define RPC_DAC_S_FORMAT_ID             IVSHM_RPC_ID(20, 4, (RPC_DAC_CMD | 0x41))

/* RPC HDMI commands */
#define RPC_HDMI_INIT_ID                IVSHM_RPC_ID(0,  4, (RPC_HDMI_CMD | 0x01))
#define RPC_HDMI_OPEN_ID                IVSHM_RPC_ID(0,  4, (RPC_HDMI_CMD | 0x02))
#define RPC_HDMI_CLOSE_ID               IVSHM_RPC_ID(0,  4, (RPC_HDMI_CMD | 0x03))
#define RPC_HDMI_G_CAP_ID               IVSHM_RPC_ID(0,  4, (RPC_HDMI_CMD | 0x21))
#define RPC_HDMI_G_CUSTOM_FMT_ID        IVSHM_RPC_ID(0,  8, (RPC_HDMI_CMD | 0x22))
#define RPC_HDMI_G_PKT_LAYOUT_ID        IVSHM_RPC_ID(0,  4, (RPC_HDMI_CMD | 0x23))
#define RPC_HDMI_G_PKT_TYPE_ID          IVSHM_RPC_ID(0,  4, (RPC_HDMI_CMD | 0x24))
#define RPC_HDMI_G_INFOFRAME_ID         IVSHM_RPC_ID(0,  8, (RPC_HDMI_CMD | 0x25))
#define RPC_HDMI_G_CS_ID                IVSHM_RPC_ID(0, 28, (RPC_HDMI_CMD | 0x26))
#define RPC_HDMI_S_FORMAT_ID            IVSHM_RPC_ID(8,  4, (RPC_HDMI_CMD | 0x41))
#define RPC_HDMI_S_IF_ID                IVSHM_RPC_ID(12, 4, (RPC_HDMI_CMD | 0x42))
#define RPC_HDMI_S_PKT_ID               IVSHM_RPC_ID(8,  4, (RPC_HDMI_CMD | 0x43))

/* Event message
 *   Setup maximum size, to avoid memory corruption:
 *    - event id : 4 bytes
 *    - channel status : 28 bytes
 */
#define RPC_HDMI_HANDLE_EVENT_ID        IVSHM_RPC_ID(32, 0, (RPC_HDMI_CMD | 0x51))

/* ADC RPC commands */
#define RPC_ADC_INIT_ID                 IVSHM_RPC_ID(0,  4, (RPC_ADC_CMD | 0x01))
#define RPC_ADC_OPEN_ID                 IVSHM_RPC_ID(0,  4, (RPC_ADC_CMD | 0x02))
#define RPC_ADC_CLOSE_ID                IVSHM_RPC_ID(0,  4, (RPC_ADC_CMD | 0x03))
#define RPC_ADC_G_CAP_ID                IVSHM_RPC_ID(0,  4, (RPC_ADC_CMD | 0x21))
#define RPC_ADC_S_FORMAT_ID             IVSHM_RPC_ID(20, 4, (RPC_ADC_CMD | 0x41))


#ifdef LK
struct ivshm_rpc_service;
#else
struct rpmsg_rpc_dev;
#endif

struct rpc_client_callback {
    unsigned id;
    unsigned flags;
#ifdef LK
    int (*fn)(struct ivshm_rpc_service *s, struct rpc_client_callback *cb,
              void *payload);
#else
    int (*fn)(struct rpmsg_rpc_dev *rpcdev, struct rpc_client_callback *cb,
              void *payload);
#endif
};

typedef enum _ivshm_rpc_type {
    IVSHM_RPC_CALL = 0xCA,
    IVSHM_RPC_REPLY = 0xAC,
} ivshm_rpc_type_t;

struct ivshm_rpc_header {
    unsigned id;
    ivshm_rpc_type_t type;
    uint32_t len;
    uint32_t tid;
    char payload[];
};

#ifdef LK
int ivshm_init_rpc(struct ivshm_info *);
void ivshm_exit_rpc(struct ivshm_info *);

/* External API */
struct ivshm_rpc_service *ivshm_get_rpc_service(unsigned);

int ivshm_rpc_call(struct ivshm_rpc_service *, unsigned,
                   void *, size_t, void *, size_t *);

int ivshm_rpc_reply(struct ivshm_rpc_service *, unsigned,
                    void *, size_t);

void ivshm_rpc_register_client(struct ivshm_rpc_service *,
                               struct rpc_client_callback **);
void ivshm_rpc_unregister_client(struct ivshm_rpc_service *);
void ivshm_rpc_set_data(struct ivshm_rpc_service *, void *);
void *ivshm_rpc_get_data(struct ivshm_rpc_service *);

status_t ivshm_rpc_cmd(struct ivshm_rpc_service *, unsigned);
status_t ivshm_rpc_cmd_get(struct ivshm_rpc_service *, unsigned, void *, size_t);
status_t ivshm_rpc_cmd_set(struct ivshm_rpc_service *, unsigned, void *, size_t);
#endif
#endif
