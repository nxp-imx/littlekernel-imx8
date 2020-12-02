/*
 * Copyright 2018-2020 - NXP
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

#include <sys/types.h>
#include <dev/interrupt.h>
#include <reg.h>
#include <stdio.h>
#include <kernel/vm.h>
#include <lk/init.h>
#include <lib/appargs.h>
#include "err.h"


/* from jailhouse inmate_common.h */
#define PCI_CFG_VENDOR_ID   0x000
#define PCI_CFG_DEVICE_ID   0x002
#define PCI_CFG_COMMAND     0x004
# define PCI_CMD_IO     (1 << 0)
# define PCI_CMD_MEM        (1 << 1)
# define PCI_CMD_MASTER     (1 << 2)
# define PCI_CMD_INTX_OFF   (1 << 10)
#define PCI_CFG_STATUS      0x006
# define PCI_STS_INT        (1 << 3)
# define PCI_STS_CAPS       (1 << 4)
#define PCI_CFG_BAR     0x010
# define PCI_BAR_64BIT      0x4
#define PCI_CFG_CAP_PTR     0x034

#define PCI_ID_ANY      0xffff

#define PCI_DEV_CLASS_OTHER 0xff

#define PCI_CAP_MSI     0x05
#define PCI_CAP_VENDOR      0x09
#define PCI_CAP_MSIX        0x11

#define MSIX_CTRL_ENABLE    0x8000
#define MSIX_CTRL_FMASK     0x4000

typedef s8 __s8;
typedef u8 __u8;

typedef s16 __s16;
typedef u16 __u16;

typedef s32 __s32;
typedef u32 __u32;

typedef s64 __s64;
typedef u64 __u64;

#include <jailhouse/hypercall.h>
#include <dev/ivshm.h>

#define IVSHM_CFG_SHMEM_PTR  0x40
#define IVSHM_CFG_SHMEM_SZ   0x48

#define IVSHMEM_CFG_STATE_TAB_SZ    0x04
#define IVSHMEM_CFG_RW_SECTION_SZ   0x08
#define IVSHMEM_CFG_OUT_SECTION_SZ  0x10
#define IVSHMEM_CFG_ADDRESS     0x18

#define BAR_BASE            0xff000000

#define JAILHOUSE_SHMEM_PROTO_UNDEFINED        0x0000u

#define MAX_NDEV       4

/*
 * Following fallback definitions are expected to come from jailhouse node in dts
 * */
#ifndef PCI_CFG_BASE
#define PCI_CFG_BASE    0xbb800000
#endif
/* Default mapping size: pci_init() jailhouse/hypervisor/pci.c */
#ifndef PCI_CFG_SIZE
#define PCI_CFG_SIZE    0x100000
#endif

#ifndef IVSHM_IRQ
#define IVSHM_IRQ 108
#endif

#ifndef COMM_REGION_BASE
#define COMM_REGION_BASE    0x60000000
#endif

#define PCI_CFG_VIRT(x)         ((uintptr_t)(0xffffffff00000000 | (x & 0xffffffff)))
#define JH_CFG_VIRT(x)         ((uintptr_t)(0xffffffff00000000 | (x & 0xffffffff)))

static paddr_t g_pci_cfg_base = PCI_CFG_BASE;
static size_t g_pci_cfg_size = PCI_CFG_SIZE;
static vaddr_t g_pci_cfg_virt = PCI_CFG_VIRT(PCI_CFG_BASE);
static int irq_base = IVSHM_IRQ;

static struct s_pci_id {
    uint32_t vid;
    uint32_t pid;
} ivshm_pci_id[] = {
    {0x1AF4, 0x1110},
    {0x110A, 0x4106}
};

#ifndef PAGE_MASK
#define PAGE_MASK ~(PAGE_SIZE - 1)
#endif

static int irq_counter;

static struct ivshm_dev_data ivshm_devs[MAX_NDEV];

static status_t generic_arm64_map_regs(const char *name, vaddr_t vaddr,
                                       paddr_t paddr, size_t size)
{
    status_t ret;
    void *vaddrp = (void *)vaddr;
    ret = vmm_alloc_physical(vmm_get_kernel_aspace(), name,
                             size, &vaddrp, 0, paddr,
                             VMM_FLAG_VALLOC_SPECIFIC,
                             ARCH_MMU_FLAG_UNCACHED_DEVICE);
    if (ret) {
        dprintf(CRITICAL, "%s: failed %d name=%s\n", __func__, ret, name);
    }
    return ret;
}

struct jailhouse_comm_region *comm_region =
                (struct jailhouse_comm_region *) (JH_CFG_VIRT(COMM_REGION_BASE));

static int jailhouse_init(void)
{

#define OF_JAILHOUSE_NODE_NAME "/jailhouse"
#define OF_JAILHOUSE_COMM_REGION_PROP "comm_region"
#define OF_JAILHOUSE_PCI_REGION_PROP "pci_region"
#define OF_JAILHOUSE_VPCI_IRQ_BASE_PROP "irq"

    int ret = 0;
    uint32_t comm_region_prop[2];
    int node = of_get_node_by_path(OF_JAILHOUSE_NODE_NAME);
    if (node < 0) {
        printlk(LK_ERR, "Jailhouse node not found\n");
        ret = ERR_NOT_FOUND;
        return ret;
    }

    ret = of_get_int_array(node, OF_JAILHOUSE_COMM_REGION_PROP,
                                    comm_region_prop, 2);
    if (ret < 0) {
        printlk(LK_ERR, "Jailhouse node %s not found\n",
                                    OF_JAILHOUSE_COMM_REGION_PROP);

        comm_region_prop[0] = COMM_REGION_BASE;
        comm_region_prop[1] = 0x1000;
        printlk(LK_ERR, "Using default Jailhouse comm region %u@%x"
                " builtin definitions\n",
                comm_region_prop[0], comm_region_prop[1]);
    } else {
        comm_region =
            (struct jailhouse_comm_region *) JH_CFG_VIRT(comm_region_prop[0]);
    }

    ret = generic_arm64_map_regs(
                "comm region",
                (vaddr_t) comm_region,
                (paddr_t) comm_region_prop[0],
                (size_t) comm_region_prop[1]
          );

    if (ret) {
        printlk(LK_ERR, "Mapping Jailhouse comm region failed (%d)\n", ret);
        return ret;
    }

    if (comm_region->revision > 1) {
        irq_base = comm_region->vpci_irq_base + 32;
        g_pci_cfg_base = comm_region->pci_mmconfig_base;
        g_pci_cfg_size = PCI_CFG_SIZE;
    } else {
        uint32_t pci_region_prop[2];
        ret = of_get_int_array(node, OF_JAILHOUSE_PCI_REGION_PROP,
                                    pci_region_prop, 2);
        if (ret < 0) {
            printlk(LK_ERR, "Jailhouse property %s not found\n",
                                    OF_JAILHOUSE_PCI_REGION_PROP);

            printlk(LK_ERR, "Using default Jailhouse pci region %zu@%lx"
                " builtin definitions\n", g_pci_cfg_size, g_pci_cfg_base);

        } else {
            g_pci_cfg_base = pci_region_prop[0];
            g_pci_cfg_size = pci_region_prop[1];
        }

        ret = of_get_int_array(node, OF_JAILHOUSE_VPCI_IRQ_BASE_PROP,
                                    (uint32_t *) &irq_base, 1);
        if (ret < 0) {
            printlk(LK_ERR, "Jailhouse property %s not found\n",
                                    OF_JAILHOUSE_VPCI_IRQ_BASE_PROP);

            printlk(LK_ERR, "Using default Jailhouse IRQ base %d"
                    " builtin definition\n", irq_base);
        }
    }

    printlk(LK_NOTICE, "IVSHMEM: ABI revision: %d\n", comm_region->revision);
    printlk(LK_NOTICE, "IVSHMEM: VPCI region mapping: %zu@%lx\n",
                                            g_pci_cfg_size, g_pci_cfg_base);
    printlk(LK_NOTICE, "IVSHMEM: IRQ base: %d\n", irq_base);

    g_pci_cfg_virt = PCI_CFG_VIRT(g_pci_cfg_base);
    /* Map the configuration space */
    ret = generic_arm64_map_regs(
              "pci cfg",
              g_pci_cfg_virt,
              g_pci_cfg_base,
              g_pci_cfg_size
          );

    if (ret) {
        printlk(LK_ERR, "Mapping Jailhouse PCI region failed (%d)\n", ret);
        return ret;
    }


    return ret;
}

#define PCI_CFG_DEVS_MAX       (1 << 16)

u64 pci_read_config(u16 bdf, unsigned int addr, unsigned int size)
{
    u64 reg_addr = g_pci_cfg_virt | ((u32)bdf << 12) | (addr & 0xf8);
    switch (size) {
        case 1:
            return (u64) readb((u8 *)(reg_addr + (addr & 7)));
        case 2:
            return (u64) reads((u16 *)(reg_addr + (addr & 7)));
        case 4:
            return (u64) readl((u32 *)(reg_addr + (addr & 7)));
        case 8:
            return (((u64)readl((u32 *)(reg_addr + 4)) << 32)
                | readl((u32 *)(reg_addr)));

        default:
            return -1;
    }
}

void pci_write_config(u16 bdf, unsigned int addr, u64 value, unsigned int size)
{
    u64 reg_addr = g_pci_cfg_virt | ((u32)bdf << 12) | (addr & 0xf8);

    switch (size) {
        case 1:
            writeb(value, (u8 *)(reg_addr + (addr & 7)));
            break;
        case 2:
            writes(value, (u16 *)(reg_addr + (addr & 7)));
            break;
        case 4:
            writel(value, (u32 *)(reg_addr + (addr & 7)));
            break;
        case 8:
            writel((u32)(value >> 32), (u32 *)(reg_addr + 4));
            writel((u32)value, (u32 *)(reg_addr));
            break;
    }
}

static inline void pci_write8_config(u16 bdf, unsigned int addr, u8 value)
{
    pci_write_config(bdf, addr, value, 1);
}

static inline void pci_write16_config(u16 bdf, unsigned int addr, u16 value)
{
    pci_write_config(bdf, addr, value, 2);
}

static inline void pci_write32_config(u16 bdf, unsigned int addr, u32 value)
{
    pci_write_config(bdf, addr, value, 4);
}

static inline void pci_write64_config(u16 bdf, unsigned int addr, u64 value)
{
    pci_write_config(bdf, addr, value, 8);
}

static inline u8 pci_read8_config(u16 bdf, unsigned int addr)
{
    return pci_read_config(bdf, addr, 1);
}

static inline u16 pci_read16_config(u16 bdf, unsigned int addr)
{
    return pci_read_config(bdf, addr, 2);
}

static inline u32 pci_read32_config(u16 bdf, unsigned int addr)
{
    return pci_read_config(bdf, addr, 4);
}

static inline u64 pci_read64_config(u16 bdf, unsigned int addr)
{
    return pci_read_config(bdf, addr, 8);
}

int pci_find_device(u16 vendor, u16 device, u16 start_bdf)
{
    unsigned int bdf;
    u16 id;

    for (bdf = start_bdf; bdf < PCI_CFG_DEVS_MAX; bdf++) {
        id = pci_read16_config(bdf, PCI_CFG_VENDOR_ID);
        if (id == PCI_ID_ANY || (vendor != PCI_ID_ANY && vendor != id))
            continue;
        if (device == PCI_ID_ANY ||
                pci_read16_config(bdf, PCI_CFG_DEVICE_ID) == device)
            return bdf;
    }
    return -1;
}

int pci_find_cap(u16 bdf, u16 cap)
{
    u8 pos = PCI_CFG_CAP_PTR - 1;

    if (!(pci_read_config(bdf, PCI_CFG_STATUS, 2) & PCI_STS_CAPS))
        return -1;

    while (1) {
        pos = pci_read_config(bdf, pos + 1, 1);
        if (pos == 0)
            return -1;
        if (pci_read_config(bdf, pos, 1) == cap)
            return pos;
    }
}


static u64 __pci_get_bar_sz(u64 value)
{

    return ~(value & ~(0xf)) + 1;
}
static u64 pci_get_bar_sz(u16 bdf, u8 barn)
{
    u64 saved, sz;
    u32 addr = barn << 3;

    saved = pci_read64_config(bdf, PCI_CFG_BAR + addr);
    pci_write64_config(bdf, PCI_CFG_BAR + addr, 0xffffffffffffffffULL);
    sz = pci_read64_config(bdf, PCI_CFG_BAR + addr);
    pci_write64_config(bdf, PCI_CFG_BAR + addr, saved);

    return __pci_get_bar_sz(sz);
}

static int init_device_v1(struct ivshm_dev_data *d)
{

    status_t ret;
    unsigned long shmaddr, regaddr, regsz;
    d->rw_section_sz = pci_read64_config(d->bdf, IVSHM_CFG_SHMEM_SZ);
    shmaddr = pci_read64_config(d->bdf, IVSHM_CFG_SHMEM_PTR);
    d->rw_section = (u32 *) PCI_CFG_VIRT(shmaddr);

    printlk(LK_NOTICE, "IVSHMEM: R/W section %llu@%p\n",
                                        d->rw_section_sz, d->rw_section);
    regaddr =
        (((paddr_t)(shmaddr + d->rw_section_sz + PAGE_SIZE - 1)) & PAGE_MASK);

    d->registers = (void *) PCI_CFG_VIRT(regaddr);

    pci_write64_config(d->bdf, PCI_CFG_BAR, (u64)regaddr);
    regsz = pci_get_bar_sz(d->bdf, 2);
    printlk(LK_NOTICE, "IVSHMEM: bar0 is at phys:%lx virt:%p, sz:%lx\n",
            regaddr, d->registers, regsz);

    pci_write16_config(d->bdf, PCI_CFG_COMMAND, (PCI_CMD_MEM | PCI_CMD_MASTER));

    ret = vmm_alloc_physical(vmm_get_kernel_aspace(), "ivshmem",
                            d->rw_section_sz + PAGE_SIZE + regsz,
                            (void **) &d->rw_section, 0, shmaddr,
                            VMM_FLAG_VALLOC_SPECIFIC,
                            ARCH_MMU_FLAG_CACHED);

    d->id = readl(d->registers + IVSHM_V1_REG_IVPOS);
    assert(ret == 0);
    d->state_table = NULL;
    d->state_table_sz = 0;
    d->in_sections = NULL;
    d->out_section = NULL;
    d->out_section_sz = 0;
    return 0;
}

static void init_device_v2(struct ivshm_dev_data *d)
{
    unsigned long baseaddr, addr, size;
    int vndr_cap;
    u32 max_peers;
#define NAME_MAX_SZ 32
    char name[NAME_MAX_SZ];


    vndr_cap = pci_find_cap(d->bdf, PCI_CAP_VENDOR);
    if (vndr_cap < 0) {
        panic("IVSHMEM ERROR: missing vendor capability\n");
    }

    d->registers = (void *)(uintptr_t)(PCI_CFG_VIRT(BAR_BASE));
    pci_write_config(d->bdf, PCI_CFG_BAR, BAR_BASE, 4);
    printlk(LK_NOTICE, "IVSHMEM: bar0 is at %p/%x\n",
                                        d->registers, BAR_BASE);

    d->msix_table = (u32 *)(uintptr_t)(PCI_CFG_VIRT((BAR_BASE + PAGE_SIZE)));
    pci_write_config(d->bdf, PCI_CFG_BAR + 4,
        (unsigned long)(BAR_BASE + PAGE_SIZE), 4);
    printlk(LK_NOTICE, "IVSHMEM: bar1 is at %p/%lx\n",
                                        d->msix_table, BAR_BASE + PAGE_SIZE);

    pci_write_config(d->bdf, PCI_CFG_COMMAND,
             (PCI_CMD_MEM | PCI_CMD_MASTER), 2);

    snprintf(name, NAME_MAX_SZ, "ivshm reg %d", d->bdf);
    generic_arm64_map_regs(name, (vaddr_t) d->registers, BAR_BASE, PAGE_SIZE * 2);

    d->id = readl(d->registers + IVSHM_V2_REG_ID);
    printlk(LK_NOTICE, "IVSHMEM: ID is %d\n", d->id);

    max_peers = readl(d->registers + IVSHM_V2_REG_MAX_PEERS);
    printlk(LK_NOTICE, "IVSHMEM: max. peers is %d\n", max_peers);

    d->state_table_sz =
        pci_read_config(d->bdf, vndr_cap + IVSHMEM_CFG_STATE_TAB_SZ, 4);
    d->rw_section_sz =
        pci_read64_config(d->bdf, vndr_cap + IVSHMEM_CFG_RW_SECTION_SZ);
    d->out_section_sz =
        pci_read64_config(d->bdf, vndr_cap + IVSHMEM_CFG_OUT_SECTION_SZ);
    baseaddr = pci_read64_config(d->bdf, vndr_cap + IVSHMEM_CFG_ADDRESS);

    addr = PCI_CFG_VIRT(baseaddr);
    d->state_table = (u32 *)addr;

    addr += d->state_table_sz;
    d->rw_section = (u32 *)addr;

    addr += d->rw_section_sz;
    d->in_sections = (u32 *)addr;

    addr += d->id * d->out_section_sz;
    d->out_section = (u32 *)addr;

    printlk(LK_NOTICE, "IVSHMEM: state table: %u@%p\n",
                                        d->state_table_sz, d->state_table);
    printlk(LK_NOTICE, "IVSHMEM: R/W section: %llu@%p\n",
                                        d->rw_section_sz, d->rw_section);
    printlk(LK_NOTICE, "IVSHMEM: [NOT USED] input section: %llu@%p\n",
                                        d->out_section_sz, d->in_sections);
    printlk(LK_NOTICE, "IVSHMEM: [NOT USED] output section: %llu@%p\n",
                                        d->out_section_sz, d->out_section);

    size = d->state_table_sz + d->rw_section_sz +
        max_peers * d->out_section_sz;

    snprintf(name, NAME_MAX_SZ, "ivshmem  %d", d->bdf);
    vmm_alloc_physical(vmm_get_kernel_aspace(), name,
                            size, (void **)&d->state_table, 0, baseaddr,
                            VMM_FLAG_VALLOC_SPECIFIC,
                            ARCH_MMU_FLAG_CACHED);

}

static enum handler_return ivshm_irq_handler(void *arg)
{
    struct ivshm_dev_data *d = arg;
    printlk(LK_INFO, "%s:%d: IVSHMEM: irq - interrupt #%d\n",
            __PRETTY_FUNCTION__, __LINE__, irq_counter++);

    if (d->handler) {
        return d->handler(d->handler_arg);
    }
    return INT_RESCHEDULE;
}

static int ivshm_device_init(struct ivshm_dev_data *dev)
{
    int ret;
    u32 class_rev;
    u16 bdf = dev->bdf;
    uint32_t vid, pid;

    vid = ivshm_pci_id[comm_region->revision - 1].vid;
    pid = ivshm_pci_id[comm_region->revision - 1].pid;

    ret = pci_find_device(vid, pid, bdf);
    if (ret) {
        printlk(LK_ERR, "IVSHM device with bdf:%d\n", bdf);
        return ERR_NOT_FOUND;
    }

    printlk(LK_NOTICE, "Found %04x:%04x at %02x:%02x.%x\n",
            pci_read16_config(bdf, PCI_CFG_VENDOR_ID),
            pci_read16_config(bdf, PCI_CFG_DEVICE_ID),
            bdf >> 8, (bdf >> 3) & 0x1f, bdf & 0x7);

    class_rev = pci_read32_config(bdf, 0x8);
    if (class_rev != (PCI_DEV_CLASS_OTHER << 24 |
                      JAILHOUSE_SHMEM_PROTO_UNDEFINED << 8)) {
        printlk(LK_NOTICE, "class/revision %08x, not supported\n", class_rev);
        return ERR_NOT_SUPPORTED;
    }

    dev->revision = comm_region->revision;

    if (comm_region->revision == 2)
        init_device_v2(dev);
    else
        init_device_v1(dev);


    register_int_handler(dev->irq, ivshm_irq_handler, dev);
    unmask_interrupt(dev->irq);

    return 0;
}

static void ivshm_init(uint level)
{
    status_t ret;
    struct ivshm_dev_data *d = NULL;

    ret = jailhouse_init();
    ASSERT(ret == 0);

    d = &ivshm_devs[0];
    d->bdf = 0;
    d->irq = irq_base;
    ret = ivshm_device_init(d);
    assert(ret == 0);
}

LK_INIT_HOOK(ivshmem, ivshm_init, LK_INIT_LEVEL_VM + 1);

struct ivshm_dev_data *ivshm_register(int dev, int_handler handler, void *arg)
{
    struct ivshm_dev_data *data;
    if (dev >= MAX_NDEV) {
        dprintf(CRITICAL, "ERROR: Invalid ivshmem dev id (%d).\n", dev);
        return NULL;
    }

    data = &ivshm_devs[dev];
    if (data == NULL) {
        dprintf(CRITICAL, "IVSHMEM: ERROR: ivshmem device (%d) data not found.\n", dev);
        return NULL;
    }

    data->handler_arg = arg;
    data->handler = handler;

    return data;
}

struct ivshm_dev_data *ivshm_get_device(int dev)
{
    if (dev >= MAX_NDEV) {
        dprintf(CRITICAL, "ERROR: Invalid ivshmem dev id (%d).\n", dev);
        return NULL;
    }

    return &ivshm_devs[dev];
}

#if IVSHM_TEST
struct ivshm_test_data {
    u32 magic_value;
    struct ivshm_dev_data *dev;
};

static enum handler_return ivshm_test_handler(void *arg)
{
    struct ivshm_test_data *test_data = arg;
    struct ivshm_dev_data *d = test_data->dev;
    u32 value;

    value = readl(d->shmem);
    printf("ivshmem handler: Read %x from shmem\n", value);
    writel(test_data->magic_value, d->shmem);
    writel(1, d->registers + IVSHM_REG_DBELL);
    return INT_RESCHEDULE;

}

struct ivshm_test_data test_data;

static void ivshm_init_test(uint level)
{
    test_data.magic_value = 0xCAFEBABE;
    test_data.dev = ivshm_register(0, ivshm_test_handler, &test_data);
    assert(test_data.dev != NULL);

    writel(1, d->registers + IVSHM_REG_DBELL);
}

LK_INIT_HOOK(ivshm_test, ivshm_init_test, LK_INIT_LEVEL_VM + 2);
#endif /* IVSHM_TEST */
