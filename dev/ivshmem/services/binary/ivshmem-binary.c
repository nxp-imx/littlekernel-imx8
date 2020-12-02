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

#include <string.h>
#include <stdlib.h>
#include <lib/cbuf.h>
#include <assert.h>
#include <err.h>
#include <lib/appargs.h>
#include <dev/driver.h>

#include <dev/ivshm.h>
#include <ivshmem-pipe.h>
#include <ivshmem-endpoint.h>
#include <ivshmem-binary.h>
#include <cipc.h>
#include <debug.h>


#define BIT(x) (1ULL << x)

static event_t ivshm_ready_evt = EVENT_INITIAL_VALUE(ivshm_ready_evt, 0, 0);

/* Do not overwrite payload when buffer is full */
#define IVSHM_BINARY_NO_OVERWRITE       BIT(0)

struct ivshm_binary_pkt {
    struct list_node node;
    size_t len;
    char payload[];
};

struct ivshm_binary_service {
    struct list_node node;
    unsigned id;
    struct ivshm_endpoint *ept;
#ifdef EN_LOOPBACK_TEST
    thread_t *loopback_thread;
#endif
};

static struct list_node binary_service_list =
        LIST_INITIAL_VALUE(binary_service_list);


#define to_ivshm_binary_rx_handler(x) \
    ((struct ivshm_binary_handler *)(&x->private))
#define to_ivshm_binary_tx_handler(x) \
    ((struct ivshm_binary_handler *)(&x->private) + 1)

/**
 * @brief Fill packet for writing operation
 *
 * @param ppkt   Pointer to the packet
 * @param len    File length
 * @param name   File name
 * @param ofl    Writing operation mode
 */
static inline void fill_writing_packet(struct ivshm_binary_file_pkt *ppkt,
    size_t len, char *name, uint32_t ofl)
{
    memset(ppkt, 0, sizeof(struct ivshm_binary_file_pkt));
    strncpy(ppkt->name, name, FILE_NAME_MAX);
    ppkt->start_magic = FILE_MAGIC;
    ppkt->length = len;
    IVSHM_SET_FIELD(ppkt->ctrl_flag, IVSHM_CMD_OP, IVSHM_CMD_OP_VAL_WRITE);
    ppkt->stop_magic = FILE_MAGIC;
    if (ofl == FILE_O_CREATE) {
        IVSHM_SET_FIELD(ppkt->ctrl_flag, IVSHM_CMD_MODE, IVSHM_CMD_MODE_VAL_CREATE);
        printlk(LK_VERBOSE, "Create mode writing operation\n");
    } else {
        IVSHM_SET_FIELD(ppkt->ctrl_flag, IVSHM_CMD_MODE, IVSHM_CMD_MODE_VAL_APPEND);
        printlk(LK_VERBOSE, "Append mode writing operation\n");
    }
}

/**
 * @brief Fill packet for reading operation
 *
 * @param ppkt   Pointer to the packet
 * @param len    File length
 * @param name   File name
 */
static inline void fill_reading_packet(struct ivshm_binary_file_pkt *ppkt,
    size_t len, char *name)
{
    memset(ppkt, 0, sizeof(struct ivshm_binary_file_pkt));
    strncpy(ppkt->name, name, FILE_NAME_MAX);
    ppkt->start_magic = FILE_MAGIC;
    ppkt->length = len;
    IVSHM_SET_FIELD(ppkt->ctrl_flag, IVSHM_CMD_OP, IVSHM_CMD_OP_VAL_READ);
    ppkt->stop_magic = FILE_MAGIC;
}


/**
 * @brief Fill packet for stating operation
 *
 * @param ppkt   Pointer to the packet
 * @param name   File name
 */
static inline void fill_sizing_packet(struct ivshm_binary_file_pkt *ppkt, char *name)
{
    memset(ppkt, 0, sizeof(struct ivshm_binary_file_pkt));
    strncpy(ppkt->name, name, FILE_NAME_MAX);
    ppkt->start_magic = FILE_MAGIC;
    IVSHM_SET_FIELD(ppkt->ctrl_flag, IVSHM_CMD_OP, IVSHM_CMD_OP_VAL_SIZE);
    ppkt->stop_magic = FILE_MAGIC;
}

void ivshm_binary_list(void)
{
    struct ivshm_binary_service *service;

    list_for_every_entry(&binary_service_list, service,
                         struct ivshm_binary_service, node) {
        printlk(LK_NOTICE, "%s:%d: %d: endpoint name %s\n",
                __PRETTY_FUNCTION__, __LINE__, service->id, service->ept->name);
    }
}

static inline size_t ivshm_binary_write_cbuf(struct ivshm_binary_handler *handler, void *buf, size_t len)
{
    size_t avail = cbuf_space_avail(&handler->cbuf);

    if (likely(avail >= len))
        return cbuf_write(&handler->cbuf, buf, len, false);

    size_t remaining = len - avail;

    if (!(handler->flags & IVSHM_BINARY_NO_OVERWRITE))
        /* Overwrite beginning of circular buffer */
        cbuf_read(&handler->cbuf, NULL, remaining, false);

    return cbuf_write(&handler->cbuf, buf, len, false);
}

static ssize_t ivshm_binary_rx_consume(struct ivshm_endpoint *ep,
                                       struct binary_pkt *pkt_in)
{
    struct ivshm_binary_handler *handler = to_ivshm_binary_rx_handler(ep);
    struct binary_pkt pkt_out = { 0 };
    struct ivshm_ep_buf ep_buf;
    uint8_t check = ~pkt_in->n_pkt;
    size_t pkt_len;

    DEBUG_ASSERT(handler->state == RESET || handler->state == RECEIVING);
    DEBUG_ASSERT(pkt_in->pkt == check);

    if (handler->state == RESET) {
        /* New reception */
        size_t count;

        DEBUG_ASSERT(pkt_in->hdr == IVSHM_BIN_SOH ||
                     pkt_in->hdr == IVSHM_BIN_EOT);
        DEBUG_ASSERT(pkt_in->pkt == 0x1);

        handler->state = RECEIVING;
        handler->last_pkt = 0x1;
        pkt_len = pkt_in->len;
        printlk(LK_DEBUG, "pkt %d: hdr 0x%X - len %lu\n", pkt_in->pkt, pkt_in->hdr, pkt_len);

        count = ivshm_binary_write_cbuf(handler, pkt_in->payload, pkt_len);

        DEBUG_ASSERT(count == pkt_len);

        pkt_out.hdr = IVSHM_BIN_ACK;
    } else if (pkt_in->pkt == handler->last_pkt + 1) {
        /* Receiving */
        size_t count;

        handler->last_pkt++;
        pkt_len = pkt_in->len;
        printlk(LK_DEBUG, "pkt %d: hdr 0x%X - len %lu\n", pkt_in->pkt, pkt_in->hdr, pkt_len);

        count = ivshm_binary_write_cbuf(handler, pkt_in->payload, pkt_len);

        DEBUG_ASSERT(count == pkt_len);

        pkt_out.hdr = IVSHM_BIN_ACK;
    } else {
        /* Already received packet, just acknowledge it */
        printlk(LK_DEBUG, "pkt %d: already received\n", pkt_in->pkt);
        pkt_out.hdr = IVSHM_BIN_ACK;
    }

    if (pkt_in->hdr == IVSHM_BIN_EOT)
        handler->state = EOT;

    ivshm_ep_buf_init(&ep_buf);
    ivshm_ep_buf_add(&ep_buf, &pkt_out, sizeof(struct binary_pkt));
    ivshm_endpoint_write(ep, &ep_buf);

    if (handler->state == EOT) {
        iovec_t regions[2];
        size_t size = cbuf_peek(&handler->cbuf, regions);

        printlk(LK_INFO, "%s:%d: Transmission complete - %lu bytes\n",
                __PRETTY_FUNCTION__, __LINE__, size);
        handler->state = RESET;

#ifdef EN_LOOPBACK_TEST
        printlk(LK_DEBUG, "trig thread execution\n");
        sem_post(&handler->sem_loop, false);
#endif
    }

    return 0;
}

static size_t _ivshm_binary_write(struct ivshm_endpoint *ept,
                                 void *buf, size_t len)
{
    struct ivshm_binary_handler *handler = to_ivshm_binary_tx_handler(ept);
    struct ivshm_ep_buf ep_buf;
    struct binary_pkt pkt_out = { 0 };
    size_t remaining = len, count = 0, offset = 0;
    int ret;

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    DEBUG_ASSERT(handler->state == RESET);

    handler->state = TRANSMITTING;
    handler->last_pkt = 0x1;
    handler->retry = 10;

    do {
        if (remaining > CHUNK_SIZE)
            pkt_out.hdr = IVSHM_BIN_SOH;
        else
            pkt_out.hdr = IVSHM_BIN_EOT;

        pkt_out.pkt = handler->last_pkt;
        pkt_out.n_pkt = ~handler->last_pkt;

        count = remaining > CHUNK_SIZE ? CHUNK_SIZE : remaining;
        pkt_out.len = count;
        /* chk field is not used */

        printlk(LK_DEBUG, "pkt %d - hdr 0x%X - len %lu\n",
                      pkt_out.pkt, pkt_out.hdr, count);

        /* Allways sending same amount of data */
        handler->ack = IVSHM_BIN_NAK;
        ivshm_ep_buf_init(&ep_buf);
        ivshm_ep_buf_add(&ep_buf, buf + offset, count);
        ivshm_ep_buf_add(&ep_buf, &pkt_out, sizeof(struct binary_pkt));

        ivshm_endpoint_write(ept, &ep_buf);

        /* Wait for acknowledgement */
        ret = sem_timedwait(&handler->sem_ack, 10000);
        if (ret == 0 && handler->ack == IVSHM_BIN_ACK) {
            /* Normal case */
            handler->retry = 10;
            handler->last_pkt++;
            remaining -= count;
            offset += count;

            if (remaining <= 0)
                handler->state = EOT;

            printlk(LK_DEBUG, "state 0x%X offset %lu\n", handler->state, offset);
        } else {
            if (--handler->retry == 0)
                goto out;
        }
    } while (handler->state != EOT);

out:
    handler->state = RESET;
    return offset;
}

#ifdef EN_LOOPBACK_TEST
static int ivhsm_binary_loopback_routine(void *arg)
{
    struct ivshm_binary_service *service = arg;
    struct ivshm_binary_handler *handler =
        to_ivshm_binary_rx_handler(service->ept);
    iovec_t regions[2];
    size_t size;
    int ret;

    printlk(LK_NOTICE, "%s:%d: loopback thread running\n", __PRETTY_FUNCTION__,
            __LINE__);

    while (handler->thread_running) {
        uint8_t *loop_buf;
        ret = sem_timedwait(&handler->sem_loop, 1000);
        if (ret)
            continue;

        size = cbuf_peek(&handler->cbuf, regions);
        printlk(LK_NOTICE, "%s:%d: LOOPBACK TEST %u: Send back cbuf size %lu to emitter\n",
               __PRETTY_FUNCTION__, __LINE__, service->id, size);
        loop_buf = malloc(size);
        if (!loop_buf)
            continue;

        cipc_read_buf(service->id, loop_buf, size);
#ifdef DUMP_BUFFER
        hexdump(loop_buf, size);
#endif

        cipc_write_buf(service->id, loop_buf, size);
        free(loop_buf);
    }

    return 0;
}
#endif

static int ivshm_binary_tx_thread(void *arg)
{
    struct ivshm_binary_service *service = arg;
    struct ivshm_binary_handler *handler;
    struct ivshm_binary_pkt *pkt;
    spin_lock_saved_state_t flags;
    size_t len;
    int ret;

    ASSERT(service);

    event_wait(&ivshm_ready_evt);

    printlk(LK_INFO, "%s:%d: binary-%d TX buffer queue thread running\n",
            __PRETTY_FUNCTION__, __LINE__, service->id);

    handler = to_ivshm_binary_tx_handler(service->ept);

    while (handler->thread_running) {
        ret = sem_timedwait(&handler->sem_queue, 1000);
        if (ret) {
            printlk(LK_VERBOSE, "timeout\n");
            continue;
        }

        /* Go through TX queue and send payload */
        spin_lock_irqsave(&handler->pkt_lock, flags);
        pkt = list_remove_head_type(&handler->pkt_list,
                                  struct ivshm_binary_pkt,
                                  node);
        spin_unlock_irqrestore(&handler->pkt_lock, flags);

        ASSERT(pkt);

        printlk(LK_INFO, "%s:%d: Send queued buffer len %lu\n", __PRETTY_FUNCTION__,
                __LINE__, pkt->len);

        len = _ivshm_binary_write(service->ept, pkt->payload, pkt->len);

#ifdef ENABLE_PERMISSIVE_MODE
        if (len != pkt->len)
            printlk(LK_WARNING, "%s:%d: WARNING PERMISSIVE: len(%lu) != pkt->len(%lu)\n",
                    __PRETTY_FUNCTION__, __LINE__, len, pkt->len);
#else
        DEBUG_ASSERT(len == pkt->len);
#endif

        free(pkt);
    }

    printlk(LK_NOTICE, "%s:%d: Goodbye !\n", __PRETTY_FUNCTION__, __LINE__);

    return 0;
}

static ssize_t ivshm_binary_tx_consume(struct ivshm_endpoint *ep,
                                       struct binary_pkt *pkt_in)
{
    struct ivshm_binary_handler *handler = to_ivshm_binary_tx_handler(ep);

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);
    handler->ack = pkt_in->hdr;
    sem_post(&handler->sem_ack, false);

    return 0;
}

static ssize_t ivshm_binary_consume(struct ivshm_endpoint *ep,
                                    struct ivshm_pkt *pkt)
{
    size_t len = ivshm_pkt_get_payload_length(pkt);
    struct binary_pkt *pkt_in = (struct binary_pkt *)&pkt->payload;

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    DEBUG_ASSERT(len <= sizeof(struct binary_pkt) + CHUNK_SIZE);

    if (pkt_in->hdr == IVSHM_BIN_SOH || pkt_in->hdr == IVSHM_BIN_EOT)
        return ivshm_binary_rx_consume(ep, pkt_in);
    else if (pkt_in->hdr == IVSHM_BIN_ACK || pkt_in->hdr == IVSHM_BIN_NAK)
        return ivshm_binary_tx_consume(ep, pkt_in);
    else
        printlk(LK_ERR, "%s:%d: Unsupported hdr\n", __PRETTY_FUNCTION__, __LINE__);

    return 0;
}

static void ivshm_binary_queue_buffer(struct ivshm_binary_handler *handler,
                                      void *buf, size_t len)
{
    spin_lock_saved_state_t flags;
    struct ivshm_binary_pkt *pkt;

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    pkt = malloc(sizeof(struct ivshm_binary_pkt) + len);
    ASSERT(pkt);

    spin_lock_irqsave(&handler->pkt_lock, flags);
    pkt->len = len;
    list_add_tail(&handler->pkt_list, &pkt->node);
    spin_unlock_irqrestore(&handler->pkt_lock, flags);

    /* Execute buffer copy outside of critical section */
    memcpy(pkt->payload, buf, len);

    /* Wake up transmision thread */
    sem_post(&handler->sem_queue, false);
}

static __inline struct ivshm_binary_service
                        *_ivshm_binary_get_service(unsigned id)
{
    struct ivshm_binary_service *service;

    list_for_every_entry(&binary_service_list, service,
                         struct ivshm_binary_service, node) {
        if (service->id == id)
            return service;
    }

    printlk(LK_ERR, "%s:%d: Could not find cipc-%d endpoint\n",
            __PRETTY_FUNCTION__, __LINE__, id);
    return NULL;
}

/*
 * Customer IPC (cipc) API functions
 */
ssize_t cipc_read_buf(unsigned id, void *buf, size_t len)
{
    struct ivshm_binary_service *service;
    struct ivshm_binary_handler *handler;

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    service = _ivshm_binary_get_service(id);

    if (!service)
        return ERR_NOT_FOUND;

    handler = to_ivshm_binary_rx_handler(service->ept);

    return cbuf_read(&handler->cbuf, buf, len, false);
}

ssize_t cipc_read_buf_blocking(unsigned id, void *buf, size_t len)
{
    struct ivshm_binary_service *service;
    struct ivshm_binary_handler *handler;

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    service = _ivshm_binary_get_service(id);

    if (!service)
        return ERR_NOT_FOUND;

    handler = to_ivshm_binary_rx_handler(service->ept);

    return cbuf_read(&handler->cbuf, buf, len, true);
}

ssize_t cipc_write_buf(unsigned id, void *buf, size_t len)
{
    struct ivshm_binary_service *service;
    struct ivshm_binary_handler *handler;

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    service = _ivshm_binary_get_service(id);

    if (!service)
        return ERR_NOT_FOUND;

    handler = to_ivshm_binary_tx_handler(service->ept);
    ivshm_binary_queue_buffer(handler, buf, len);

    return len;
}

/*
 * Receive response from second OS
 *
 * @param id      Service id
 * @param bytes   The actual number of bytes read/written
 *
 * @return NO_ERROR        No error
 * @return ERR_NOT_VALID   Not a valid acknowledgement
 * @return ERR_NOT_FOUND   Service not found
 * @return ERR_NOT_FILE    File not found
 */
static ssize_t receive_response(unsigned id, size_t *op_bytes)
{
    ssize_t nb;
    ssize_t ret;
    uint32_t resp_packet;
    size_t a_size;
    void *p_packet;

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);
    p_packet = (void *) &resp_packet;
    /* Blocking read until at least one bytes is available */
    nb = cipc_read_buf_blocking(id, p_packet, sizeof(resp_packet));
    if (nb < 0) {
        printlk(LK_DEBUG, "Receiving ACK: Error %ld\n", nb);
        ret = ERR_NOT_FOUND;
        goto exit;
    }
    if (IVSHM_FIELD_TO_VAL(IVSHM_RSP_ACK_ID, resp_packet) != IVSHM_RSP_ACK_ID_VAL) {
        printlk(LK_DEBUG, "Error: ack bit not correct: 0x%08X\n", resp_packet);
        ret = ERR_NOT_VALID;
        goto exit;
    }
    /* Check if response is positive */
    if (IVSHM_FIELD_TO_VAL(IVSHM_RSP_RESP, resp_packet) != IVSHM_RSP_RESP_VAL_SUCCESS) {
        printlk(LK_DEBUG, "resp bit not set 0x%08X: file error\n", resp_packet);
        ret = ERR_NOT_FILE;
        goto exit;
    }
    /* Retrieve actual size of the file */
    p_packet = (void *) &a_size;
    /* Blocking read until at least one bytes is available */
    nb = cipc_read_buf_blocking(id, p_packet, sizeof(a_size));
    if (nb < 0) {
        printlk(LK_DEBUG, "Receiving actual size: Error %ld\n", nb);
        ret = ERR_NOT_FOUND;
        goto exit;
    }
    /* Returning the actual size */
    if (!op_bytes) {
        printlk(LK_DEBUG, "Receiving actual size: Error pointer is NULL\n");
        ret = ERR_INVALID_ARGS;
        goto exit;
    }
    /* No error */
    *op_bytes = a_size;
    ret = NO_ERROR;

exit:
    printlk(LK_VERBOSE, "%s: exit\n", __PRETTY_FUNCTION__);
    return ret;
}

static int cipc_send_file_packet(unsigned id, unsigned op, char *name,
                                 void *buf, size_t len, uint32_t param)
{
    struct ivshm_binary_service *service;
    struct ivshm_binary_handler *handler;
    struct ivshm_binary_file_pkt file_pkt;
    ssize_t ret = NO_ERROR;

    ASSERT(name);
    if (strlen(name) > FILE_NAME_MAX) {
        printlk(LK_ERR, "File name cannot exceed %d characters\n", FILE_NAME_MAX);
        ret = ERR_TOO_BIG;
        goto exit;
    }
    /* Get the service */
    service = _ivshm_binary_get_service(id);
    if (!service) {
        ret = ERR_NOT_FOUND;
        goto exit;
    }

    /* Fill the packet to transmit */
    switch (op) {
        case IVSHM_CMD_OP_VAL_READ:
            fill_reading_packet(&file_pkt, len, name);
            break;
        case IVSHM_CMD_OP_VAL_WRITE:
            fill_writing_packet(&file_pkt, len, name, param);
            break;
        case IVSHM_CMD_OP_VAL_SIZE:
            fill_sizing_packet(&file_pkt, name);
            break;
        default:
            ret = ERR_INVALID_ARGS;
            goto exit;
    }

    /* Get the handler and send data */
    handler = to_ivshm_binary_tx_handler(service->ept);
    ivshm_binary_queue_buffer(handler, &file_pkt, sizeof(file_pkt));

exit:
    return ret;
}

/*
 * Read file from second OS
 *
 * @param id     service id
 * @param buf    buffer to store the file to be read
 * @param len    bytes to read. If len is 0, thw whole file will be read
 * @param name   file name including the path
 *
 * @return The number of bytes read, if no error occurs
 * @return ERR_NOT_VALID   Not a valid acknowledgement
 * @return ERR_NOT_FOUND   Service not found
 * @return ERR_NOT_FILE    File not found
 */
ssize_t cipc_read_file(unsigned id, void *buf, size_t len, char *name)
{
    ssize_t ret = 0;
    size_t read_bytes = 0;
    size_t rb = 0;
    size_t remaining;

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    /* Send read file request */
    ret = cipc_send_file_packet(id, IVSHM_CMD_OP_VAL_READ, name,
                                buf, len, 0 /* params */);
    if (ret != NO_ERROR)
        goto exit;

    /* Receiving response */
    ret = receive_response(id, &read_bytes);
    if (ret != NO_ERROR) {
        printlk(LK_ERR, "%s:%d: Error %ld in receive response\n",
                __PRETTY_FUNCTION__, __LINE__, ret);
        goto exit;
    }
    ASSERT(read_bytes);
    /* Calling reading until all bytes are read */
    remaining = read_bytes;
    do {
        rb = cipc_read_buf_blocking(id, buf + ret, remaining);
        remaining -= rb;
        ret += rb;
    } while (remaining);

exit:
    printlk(LK_DEBUG, "read %ld bytes\n", ret);
    printlk(LK_VERBOSE, "%s: exit\n", __PRETTY_FUNCTION__);
    return ret;
}


/**
 * @brief Reports the size of a file of the second OS into a buffer, through
 * one ivshm binary path. A daemon running on the second OS must open the file
 * and get file size.
 *
 * @param[in] id    Endpoint id of the ivshm binary path
 * @param[in] name  The name of the file
 *
 * @return The actual file size in bytes, or negative value if error
 */
ssize_t cipc_size_file(unsigned id, char *name)
{
    ssize_t ret = 0;
    size_t total = 0;

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    /* Send stat file request */
    ret = cipc_send_file_packet(id, IVSHM_CMD_OP_VAL_SIZE, name,
                                NULL /* buf */, 0 /* len */, 0 /* params */);
    if (ret != NO_ERROR)
        goto exit;
    /* Receiving response */
    ret = receive_response(id, &total);
    if (ret != NO_ERROR) {
        printlk(LK_ERR, "Error %ld in receive response\n", ret);
        goto exit;
    }
    ret = (ssize_t) total;

exit:
    printlk(LK_DEBUG, "stat %ld bytes\n", ret);
    printlk(LK_VERBOSE, "%s: exit\n", __PRETTY_FUNCTION__);
    return ret;
}

/*
 * Write a file to second OS
 *
 * @param id     service id
 * @param buf    buffer containing the content to be written
 * @param len    bytes to write
 * @param name   file name including the path
 *
 * @return The number of bytes written, if no error occurs
 * @return ERR_NOT_VALID   Not a valid acknowledgement
 * @return ERR_NOT_FOUND   Service not found
 * @return ERR_NOT_FILE    File not found
 */
ssize_t cipc_write_file(unsigned id, void *buf, size_t len, char *name,
                              uint32_t oflags)
{
    struct ivshm_binary_service *service;
    struct ivshm_binary_handler *handler;
    ssize_t ret;
    size_t write_bytes = 0;

    printlk(LK_VERBOSE, "%s: entry\n", __PRETTY_FUNCTION__);

    /* Send write file request */
    ret = cipc_send_file_packet(id, IVSHM_CMD_OP_VAL_WRITE, name,
                                buf, len, oflags);
    if (ret != NO_ERROR)
        goto exit;

    /* Get the service */
    service = _ivshm_binary_get_service(id);
    if (!service) {
        ret = ERR_NOT_FOUND;
        goto exit;
    }

    /* Get the handler and send file data */
    handler = to_ivshm_binary_tx_handler(service->ept);
    ivshm_binary_queue_buffer(handler, buf, len);

    /* Receiving response */
    ret = receive_response(id, &write_bytes);
    if (ret != NO_ERROR) {
        /* Error occurs */
        printlk(LK_ERR, "%s:%d: Error %ld in receive response\n",
                __PRETTY_FUNCTION__, __LINE__, ret);
        goto exit;
    }
    ASSERT(write_bytes == len);
    /* No errors */
    ret = (ssize_t) write_bytes;

exit:
    printlk(LK_DEBUG, "ret %ld\n", ret);
    printlk(LK_VERBOSE, "%s: exit\n", __PRETTY_FUNCTION__);
    return ret;
}

int ivshm_init_binary(struct ivshm_info *info)
{
    event_signal(&ivshm_ready_evt, false);
    return 0;
}

void ivshm_exit_binary(struct ivshm_info *info)
{
    struct ivshm_binary_service *service, *next;
    struct ivshm_binary_handler *rx_handler, *tx_handler;
    int retcode;

    list_for_every_entry_safe(&binary_service_list, service, next,
                         struct ivshm_binary_service, node) {

        rx_handler = to_ivshm_binary_rx_handler(service->ept);
        tx_handler = to_ivshm_binary_tx_handler(service->ept);

#ifdef EN_LOOPBACK_TEST
        rx_handler->thread_running = false;
        thread_join(service->loopback_thread, &retcode, 5000);
        sem_destroy(&tx_handler->sem_loop);
#endif

        tx_handler->thread_running = false;
        thread_join(tx_handler->thread, &retcode, 5000);
        sem_destroy(&tx_handler->sem_queue);

        /* Release resources */
        sem_destroy(&rx_handler->sem_ack);
        sem_destroy(&tx_handler->sem_ack);
        free(rx_handler->cbuf.buf);

        /* Destory binary endpoint */
        ivshm_endpoint_destroy(service->ept);

        list_delete(&service->node);
        free(service);
    }
}

static status_t ivshm_binary_dev_init(struct device *dev)
{
    struct ivshm_binary_service *service;
    struct ivshm_binary_handler *rx_handler, *tx_handler;
    uint32_t id, ep_size, cbuf_size, cbuf_size_bytes;
    char name[IVSHM_EP_MAX_NAME];
    struct ivshm_dev_data *ivshm_dev = ivshm_get_device(0);
    int ret;

    if (!ivshm_dev)
        return ERR_NOT_READY;

    struct ivshm_info *info = ivshm_dev->handler_arg;
    if (!info)
        return ERR_NOT_READY;

    ret = of_device_get_int32(dev, "id", &id);
    if (ret) {
        printlk(LK_ERR, "%s:%d: Endpoint id property can\t' be read, aborting!\n",
                __PRETTY_FUNCTION__, __LINE__);
        return ERR_NOT_FOUND;
    }

    ret = of_device_get_int32(dev, "size", &ep_size);
    if (ret) {
        printlk(LK_ERR, "%s:%d: Endpoint size property can\t' be read, aborting!\n",
                __PRETTY_FUNCTION__, __LINE__);
        return ERR_NOT_FOUND;
    }

    if (of_device_get_int32(dev, "buffer_bytes", &cbuf_size_bytes)) {
        if (of_device_get_int32(dev, "buffer", &cbuf_size))
            cbuf_size_bytes = FILE_SIZE_MAX;
        else
            cbuf_size_bytes = (cbuf_size * 1024 * 1024);
    }

    /* Limit circular buffer size to 32MB */
    DEBUG_ASSERT(cbuf_size_bytes < (32 * 1024 * 1024));

    service = malloc(sizeof(struct ivshm_binary_service));
    if (!service)
        return ERR_NO_MEMORY;

    service->id = id;
    snprintf(name, IVSHM_EP_MAX_NAME, "binary-%u", service->id);
    service->ept = ivshm_endpoint_create(
                 name,
                 service->id,
                 ivshm_binary_consume,
                 info,
                 ep_size,
                 2 * sizeof(struct ivshm_binary_handler)
             );

    if (!service->ept || (uintptr_t)service->ept == (uintptr_t)ERR_NO_MEMORY) {
        free(service);
        return ERR_INTERNAL;
    }

    rx_handler = to_ivshm_binary_rx_handler(service->ept);
    tx_handler = to_ivshm_binary_tx_handler(service->ept);

    rx_handler->state = tx_handler->state = RESET;
    cbuf_initialize(&rx_handler->cbuf, cbuf_size_bytes);
    sem_init(&rx_handler->sem_ack, 0);
    sem_init(&tx_handler->sem_ack, 0);

    if (!!of_device_get_bool(dev, "no-overwrite"))
        rx_handler->flags |= IVSHM_BINARY_NO_OVERWRITE;

    /* Setup tx thread to transmit queued buffers */
    list_initialize(&tx_handler->pkt_list);
    sem_init(&tx_handler->sem_queue, 0);
    spin_lock_init(&tx_handler->pkt_lock);
    tx_handler->thread = thread_create(
                        "ivshm-binary",
                        ivshm_binary_tx_thread,
                        (void *)service,
                        IVSHM_EP_GET_PRIO(service->ept->id),
                        DEFAULT_STACK_SIZE);

    tx_handler->thread_running = true;
    thread_resume(tx_handler->thread);

#ifdef EN_LOOPBACK_TEST
    sem_init(&rx_handler->sem_loop, 0);
    service->loopback_thread = thread_create(
                        "loopback",
                        ivhsm_binary_loopback_routine,
                        (void *)service,
                        DEFAULT_PRIORITY,
                        DEFAULT_STACK_SIZE);

    rx_handler->thread_running = true;
    thread_resume(service->loopback_thread);
#endif

    list_add_tail(&binary_service_list, &service->node);

    return 0;
}

static struct driver_ops the_ops = {
    .init = ivshm_binary_dev_init,
};

DRIVER_EXPORT_WITH_LVL(imx_ivshm_binary, &the_ops, DRIVER_INIT_CORE);
