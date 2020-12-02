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


#include "ivshmem-pipe.h"
#include "ivshmem-endpoint.h"
#include "ivshmem-console.h"
#include <debug.h>

static struct ivshm_endpoint *ep_console;

#define MIN(x, y) ((x < y) ? x : y)

#include <lib/console.h>
#include <string.h>
static void ivshm_start_logger(struct ivshm_endpoint *);
static void ivshm_stop_logger(struct ivshm_endpoint *);

static ssize_t ivshm_console_consume(struct ivshm_endpoint *ep, struct ivshm_pkt *pkt)
{
    char *payload = (char *) &pkt->payload;
    size_t len = ivshm_pkt_get_payload_length(pkt);

    printlk(LK_DEBUG, "payload : %lu bytes strlen %lu\n",
		  len, strnlen(payload, len));

    /* Only send null terminated string to console */
    if ((strnlen(payload, len) < len) && (len > 1))
        console_run_script(payload);

    return 0;
}

int ivshm_init_console(struct ivshm_info *info)
{
    ep_console = ivshm_endpoint_create(
                 "console",
                 IVSHM_EP_ID_CONSOLE,
                 ivshm_console_consume,
                 info,
                 8 * 1024,
                 0
             );

    ivshm_start_logger(ep_console);

    return 0;
}

void ivshm_exit_console(struct ivshm_info *info)
{
    ivshm_stop_logger(ep_console);
    ivshm_endpoint_destroy(ep_console);
}

#include <lk/init.h>
#ifdef IVSHM_CONSOLE_USE_CBUF
#include <lib/cbuf.h>
#else
#include <lib/klog.h>
#endif
#include <lib/io.h>
#include <string.h>
#include "assert.h"

struct ivshm_console {
    print_callback_t print_cb;
    spin_lock_t tx_lock;
#ifdef IVSHM_CONSOLE_USE_CBUF
    cbuf_t tx_buf;
#endif
    thread_t *thread;
    bool running;
    struct ivshm_endpoint *ep;
};

#ifndef IVSHM_CONSOLE_BUFFER_SIZE
#define IVSHM_CONSOLE_BUFFER_SIZE (16 * 1024)
#endif
#ifndef IVSHM_LOGGER_BUF_SIZE
#define IVSHM_LOGGER_BUF_SIZE 4096
#endif


static char ivshm_console_buf[IVSHM_CONSOLE_BUFFER_SIZE];

static struct ivshm_console _ivshm_console;
static struct ivshm_console *_con = &_ivshm_console;

#ifdef IVSHM_CONSOLE_USE_CBUF
static int ivshm_logger_thread(void *arg)
{

    struct ivshm_console *con = arg;
    struct ivshm_ep_buf ep_buf;
    iovec_t regions[2];
    size_t payload_sz;

    while (con->running) {
        ivshm_ep_buf_init(&ep_buf);
        event_wait(&con->tx_buf.event);
        payload_sz = cbuf_peek(&con->tx_buf, regions);
        DEBUG_ASSERT(payload_sz > 0);
        ivshm_ep_buf_add(&ep_buf, regions[0].iov_base, regions[0].iov_len);
        if (regions[1].iov_len)
            ivshm_ep_buf_add(&ep_buf, regions[1].iov_base, regions[1].iov_len);

        ivshm_endpoint_write(con->ep, &ep_buf);

        cbuf_read(&con->tx_buf, NULL, payload_sz, false);
    }

    return 0;
}
#else
static int ivshm_logger_thread(void *arg)
{

    struct ivshm_console *con = arg;
    struct ivshm_ep_buf ep_buf;
    size_t len, xfer_len, read_chars;
    char buf[IVSHM_LOGGER_BUF_SIZE];

    while (con->running) {
        while (!klog_has_data()) {
            if (con->running == 0)
                break;
            thread_sleep(50);
        }
        len =  klog_read(buf, IVSHM_LOGGER_BUF_SIZE, -1);
        read_chars = 0;
        while (len) {
//            xfer_len = MIN(len, con->ep->max_pkt_size - sizeof(struct ivshm_pkt) );
            xfer_len = MIN(len, 2048);
            ivshm_ep_buf_init(&ep_buf);
            ivshm_ep_buf_add(&ep_buf, buf + read_chars, xfer_len);
            ivshm_endpoint_write(con->ep, &ep_buf);
            read_chars += xfer_len;
            len -= xfer_len;
        }
    }

    return 0;
}

#endif

static void ivshm_start_logger(struct ivshm_endpoint *ep)
{

    struct ivshm_console *con = _con;

    con->thread = thread_create(
                    "logger",
                    ivshm_logger_thread,
                    (void *) con,
                    LOW_PRIORITY - 1,
                    IVSHM_LOGGER_BUF_SIZE +  4096
                 );

    con->running = true;
    con->ep = ep;
    smp_wmb();
    thread_resume(con->thread);
}

static void ivshm_stop_logger(struct ivshm_endpoint *ep)
{
    struct ivshm_console *con = _con;
    int retcode;

    con->running = false;
    smp_wmb();
    thread_join(con->thread, &retcode, 1000);
}

static void ivshm_console_print(print_callback_t *cb, const char *str, size_t len)
{
#ifdef IVSHM_CONSOLE_USE_CBUF
    struct ivshm_console *con = cb->context;

    cbuf_write(&con->tx_buf, str, len, false);
#else
    klog_puts_len(str, len);
#endif
}

static void ivshm_hook_console_init(unsigned level)
{
    memset(_con, 0x0, sizeof(*_con));

#ifdef IVSHM_CONSOLE_USE_CBUF
    cbuf_initialize_etc(&_con->tx_buf, IVSHM_CONSOLE_BUFFER_SIZE, &ivshm_console_buf);
#else
    klog_create(&ivshm_console_buf, IVSHM_CONSOLE_BUFFER_SIZE, 1);
#endif

    spin_lock_init(&_con->tx_lock);

    _con->print_cb.print = ivshm_console_print;
    _con->print_cb.context = _con;

    register_print_callback(&_con->print_cb);
}

LK_INIT_HOOK(ivshm_console, ivshm_hook_console_init, LK_INIT_LEVEL_PLATFORM_EARLY - 1);

