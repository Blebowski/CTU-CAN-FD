// SPDX-License-Identifier: GPL-2.0-or-later
/*******************************************************************************
 *
 * CTU CAN FD IP Core
 *
 * Copyright (C) 2015-2018 Ondrej Ille <ondrej.ille@gmail.com> FEE CTU
 * Copyright (C) 2018-2020 Ondrej Ille <ondrej.ille@gmail.com> self-funded
 * Copyright (C) 2018-2019 Martin Jerabek <martin.jerabek01@gmail.com> FEE CTU
 * Copyright (C) 2018-2020 Pavel Pisa <pisa@cmp.felk.cvut.cz> FEE CTU/self-funded
 *
 * Project advisors:
 *     Jiri Novak <jnovak@fel.cvut.cz>
 *     Pavel Pisa <pisa@cmp.felk.cvut.cz>
 *
 * Department of Measurement         (http://meas.fel.cvut.cz/)
 * Faculty of Electrical Engineering (http://www.fel.cvut.cz)
 * Czech Technical University        (http://www.cvut.cz/)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 ******************************************************************************/

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

unsigned ctu_can_fd_read8(struct ctucan_hw_priv *priv, enum ctu_can_fd_can_registers reg) {
    return priv->read_reg(priv, (enum ctu_can_fd_can_registers)(reg & ~3)) >> (8 * (reg & 3));
}
unsigned ctu_can_fd_read16(struct ctucan_hw_priv *priv, enum ctu_can_fd_can_registers reg) {
    return priv->read_reg(priv, (enum ctu_can_fd_can_registers)(reg & ~1)) >> (8 * (reg & 1));
}
/*
void ctu_can_fd_write8(struct ctucan_hw_priv *priv, enum ctu_can_fd_can_registers reg, uint8_t val) {
    iowrite8(val, (uint8_t*)priv->mem_base + reg);
}
void ctu_can_fd_write16(struct ctucan_hw_priv *priv, enum ctu_can_fd_can_registers reg, uint16_t val) {
    iowrite16(val, (uint8_t*)priv->mem_base + reg);
}*/

struct ctucan_hw_priv* ctucanfd_init(uint32_t addr)
{
    mem_open();
    volatile void * const base = mem_map(addr, CANFD_ADDR_RANGE);

    struct ctucan_hw_priv *priv = new ctucan_hw_priv;
    memset(priv, 0, sizeof(*priv));

    priv->mem_base = base;
    priv->read_reg = ctucan_hw_read32;
    priv->write_reg = ctucan_hw_write32;

     // will leak memory, but who cares, this is just a prototype testing tool
    return priv;
}
