#include "userspace_utils.h"

#include <iostream>

static const char * const memdev = "/dev/mem";
static int mem_fd = -1;
#define CANFD_ADDR_RANGE 4096

static void mem_open()
{
    if (mem_fd >= 0)
        return;

    mem_fd = open(memdev, O_RDWR|O_SYNC);
    if (mem_fd < 0) {
        err(1, "open memory device");
    }
}

static void *mem_map(unsigned long mem_start, unsigned long mem_length)
{
    unsigned long pagesize, mem_window_size;
    void *mm, *mem;

    //pagesize = getpagesize();
    pagesize = sysconf(_SC_PAGESIZE);

    mem_window_size = ((mem_start & (pagesize-1)) + mem_length + pagesize-1) & ~(pagesize-1);

    mm = mmap(NULL, mem_window_size, PROT_WRITE|PROT_READ,
              MAP_SHARED, mem_fd, mem_start & ~(pagesize-1));
    mem = (char*)mm + (mem_start & (pagesize-1));

    if (mm == MAP_FAILED) {
        err(1, "mmap");
        return NULL;
    }

    fprintf(stderr, "mmap 0x%lx -> %p\n",mem_start,mem);
    return mem;
}

unsigned ctu_can_fd_read8(struct ctucanfd_priv *priv, enum ctu_can_fd_regs reg) {
    return priv->read_reg(priv, (enum ctu_can_fd_regs)(reg & ~3)) >> (8 * (reg & 3));
}
unsigned ctu_can_fd_read16(struct ctucanfd_priv *priv, enum ctu_can_fd_regs reg) {
    return priv->read_reg(priv, (enum ctu_can_fd_regs)(reg & ~1)) >> (8 * (reg & 1));
}
/*
void ctu_can_fd_write8(struct ctucanfd_priv *priv, enum ctu_can_fd_regs reg, uint8_t val) {
    iowrite8(val, (uint8_t*)priv->mem_base + reg);
}
void ctu_can_fd_write16(struct ctucanfd_priv *priv, enum ctu_can_fd_regs reg, uint16_t val) {
    iowrite16(val, (uint8_t*)priv->mem_base + reg);
}*/

struct ctucanfd_priv* ctucanfd_init(uint32_t addr)
{
    mem_open();
    volatile void * const base = mem_map(addr, CANFD_ADDR_RANGE);

    struct ctucanfd_priv *priv = new ctucanfd_priv;
    memset(priv, 0, sizeof(*priv));

    priv->mem_base = base;
    priv->read_reg = ctu_can_fd_read32;
    priv->write_reg = ctu_can_fd_write32;

     // will leak memory, but who cares, this is just a prototype testing tool
    return priv;
}
