#include "ctu_can_fd_linux_defs.h"
#include "ctu_can_fd_hw.h"

#include <sys/mman.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <inttypes.h>
#include <err.h>

#define CANFD_ADDR_BASE    0x43C30000
#define CANFD_ADDR_RANGE   0x10000

static const char *memdev = "/dev/mem";
static int mem_fd = -1;

static void mem_open()
{
    mem_fd = open(memdev, O_RDWR|O_SYNC);
    if (mem_fd < 0) {
        err(1, "open memory device");
    }
}

void *mem_map(unsigned long mem_start, unsigned long mem_length)
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

int main(int argc, char *argv[])
{
    mem_open();
    volatile void * const base = mem_map(CANFD_ADDR_BASE, CANFD_ADDR_RANGE);
    struct ctucanfd_priv _p, *priv = &_p;

    priv->mem_base = base;

    union ctu_can_fd_device_id_version reg;
    reg.u32 = ctu_can_fd_read32(priv, CTU_CAN_FD_DEVICE_ID);

    printf("DevID: 0x%08x, should be 0x%08x\n", reg.s.device_id, CTU_CAN_FD_ID);

    if (!ctu_can_fd_check_access(priv))
        errx(1, "error: ctu_can_fd_check_access");

    u32 version = ctu_can_fd_get_version(priv);
    printf("Core version: %u\n", version);

    return 0;
}
