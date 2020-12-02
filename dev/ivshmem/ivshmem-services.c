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
#include "compiler.h"
#include <kernel/mutex.h>
#include "assert.h"

#include "ivshmem-pipe.h"

#if defined WITH_DEV_IVSHMEM_SERVICES_CONSOLE
#include "ivshmem-console.h"
#endif /* WITH_DEV_IVSHMEM_SERVICES_CONSOLE */

#if defined WITH_DEV_IVSHMEM_SERVICES_BINARY
#include "ivshmem-binary.h"
#endif /* WITH_DEV_IVSHMEM_SERVICES_BINARY */

#if defined WITH_DEV_IVSHMEM_SERVICES_RPC
#include "ivshmem-rpc.h"
#endif /* WITH_DEV_IVSHMEM_SERVICES_RPC */

#include "ivshmem-services.h"


static struct s_ivshm_services {
    int (*init)(struct ivshm_info *);
    void (*remove)(struct ivshm_info *);
    const char name[16];
} ivshm_services[] = {
#if defined WITH_DEV_IVSHMEM_SERVICES_CONSOLE
    { .init = ivshm_init_console, .remove= ivshm_exit_console, .name = "console"},
#endif
#if defined WITH_DEV_IVSHMEM_SERVICES_BINARY
    { .init = ivshm_init_binary, .remove = ivshm_exit_binary, .name = "binary"},
#endif
#if defined WITH_DEV_IVSHMEM_SERVICES_RPC
    { .init = ivshm_init_rpc, .remove = ivshm_exit_rpc, .name = "rpc"},
#endif
};

static u64 ivshm_services_mask;

static mutex_t ivshm_services_lock = MUTEX_INITIAL_VALUE(ivshm_services_lock);
#define _mutex_acquire mutex_acquire
#define _mutex_release mutex_release


int ivshm_init_services(struct ivshm_info *info)
{
    unsigned i;
    int ret;

    for (i = 0; i < sizeof(ivshm_services) / sizeof(struct s_ivshm_services); i++) {
        ret = _mutex_acquire(&ivshm_services_lock);
        DEBUG_ASSERT(!ret);

        if (ivshm_services_mask & (1ULL << i))
            goto unlock;

        ret = ivshm_services[i].init(info);

        if (ret)
            printlk(LK_ERR, "%s:%d: Error during initialization of ivshm service %s\n",
                __PRETTY_FUNCTION__, __LINE__, ivshm_services[i].name);
        else
            ivshm_services_mask |= (1ULL << i);
unlock:
        _mutex_release(&ivshm_services_lock);
    }

    return 0;
}

void ivshm_disable_services(struct ivshm_info *info)
{
    unsigned i;
    int ret;

    for (i = 0; i < sizeof(ivshm_services) / sizeof(struct s_ivshm_services); i++) {
        ret = _mutex_acquire(&ivshm_services_lock);
        DEBUG_ASSERT(!ret);

        if (!(ivshm_services_mask & (1ULL << i)))
            goto unlock;

        if (ivshm_services[i].remove)
            ivshm_services[i].remove(info);

        ivshm_services_mask &= ~(1ULL << i);
unlock:
        _mutex_release(&ivshm_services_lock);
    }
}

