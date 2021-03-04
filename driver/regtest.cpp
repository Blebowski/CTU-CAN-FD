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
#include <stdio.h>
#include <assert.h>
#include <stdint.h>
#include <iostream>


/*
    Adapted from APB unit test.
    Should check that the core is tied to CPU correctly and register access works.
    Usage: ./regtest -a 0xAAAAAAAA, where AAAAAAAA is the IP memory map begin
*/

uint32_t s_apb_prdata;
struct ctucan_hw_priv *priv;
unsigned error = 0;

void CHECK(uint32_t expected, const char *msg)
{
    if (s_apb_prdata != expected) {
        printf("%s: expected 0x%08x, got 0x%08x", msg, expected, s_apb_prdata);
        error++;
    }
}

void apb_write(uint32_t reg, uint32_t value, uint8_t be)
{
    enum {
        U8, U16, U32
    } access;
    int offset;

    switch (be)
    {
        case 0b0001: access = U8; offset = 0; break;
        case 0b0010: access = U8; offset = 1; break;
        case 0b0100: access = U8; offset = 2; break;
        case 0b1000: access = U8; offset = 3; break;

        case 0b0011: access = U16; offset = 0; break;
        case 0b1100: access = U16; offset = 2; break;

        case 0b1111: access = U32; offset = 0; break;
        default:
            assert(!"invalid byte enable");
    }
    uint8_t * addr = (uint8_t*)priv->mem_base + reg;
    switch (access)
    {
        case U8: iowrite8(value,   addr+offset); break;
        case U16: iowrite16(value, addr+offset); break;
        case U32: iowrite32(value, addr+offset); break;
    }
}

void apb_read(uint32_t reg)
{
    uint8_t * addr = (uint8_t*)priv->mem_base + reg;
    s_apb_prdata = ioread32(addr);
}

void apb_test_pattern(uint32_t reg, uint32_t data)
{
    apb_write(reg, data, 0b1111);
    apb_read(reg);
    if (s_apb_prdata != data) {
        printf("pattern 0x%08x mismatch: got 0x%08x", data, s_apb_prdata);
        error++;
    }
}


int main(int argc, char *argv[])
{
    uint32_t addr_base = 0;

    int c;
    char *e;
    const char *progname = argv[0];
    while ((c = getopt(argc, argv, "a:th")) != -1) {
        switch (c) {
            case 'a':
                addr_base = strtoul(optarg, &e, 0);
                if (*e != '\0')
                    err(1, "-i expects a number");
            break;
            case 'h':
                printf("Usage: %s [-a ifc_addr]\n\n",
                       progname
                );
                return 0;
        }
    }

    if (addr_base < 0x40000000 || addr_base > 0xBFFFFFFF) {
        std::cerr << "Err: ifc address must lie in AXI_GP0 or AXI_GP1 port range (on Zynq).\n";
        exit(1);
    }
    if (addr_base & 0xFFF) {
        std::cerr << "Err: ifc address must be aligned to at least 4096 bytes.\n";
        exit(1);
    }

    priv = ctucanfd_init(addr_base);

    union ctu_can_fd_device_id_version reg;
    reg.u32 = priv->read_reg(priv, CTU_CAN_FD_DEVICE_ID);

    printf("DevID: 0x%08x, should be 0x%08x\n", reg.s.device_id, CTU_CAN_FD_ID);



    apb_read(CTU_CAN_FD_DEVICE_ID);
    CHECK(0x0203CAFD, "CAN ID reg mismatch (just after HW reset)");

    apb_write(CTU_CAN_FD_BTR, 0xFFFFFFFF, 0b1111);
    apb_read(CTU_CAN_FD_DEVICE_ID);
    CHECK(0x0202CAFD, "CAN ID reg mismatch");

    apb_read(CTU_CAN_FD_BTR);
    CHECK(0xFFFFFFFF, "readback mismatch");

    apb_write(CTU_CAN_FD_BTR, 0x00000000, 0b0011);
    apb_read(CTU_CAN_FD_BTR);
    CHECK(0xFFFF0000, "write low word: readback mismatch");

    apb_write(CTU_CAN_FD_BTR, 0xFFFFFFFF, 0b1111);
    apb_write(CTU_CAN_FD_BTR, 0x00000000, 0b1100);
    apb_read(CTU_CAN_FD_BTR);
    CHECK(0x0000FFFF, "write high word: readback mismatch");

    apb_test_pattern(CTU_CAN_FD_BTR, 0xAAAAAAAA);
    apb_test_pattern(CTU_CAN_FD_BTR, 0x55555555);
    apb_test_pattern(CTU_CAN_FD_BTR, 0x92492492);
    apb_test_pattern(CTU_CAN_FD_BTR, 0x49249249);
    apb_test_pattern(CTU_CAN_FD_BTR, 0x24924924);

    apb_write(CTU_CAN_FD_BTR, 0x87654321, 0b1111);
    apb_read(CTU_CAN_FD_BTR);
    CHECK(0x87654321, "readback mismatch");
    apb_write(CTU_CAN_FD_BTR, 0x000055aa, 0b0011);
    apb_read(CTU_CAN_FD_BTR);
    CHECK(0x876555aa, "write low word: readback mismatch");


    if (error)
        printf("There were %u errors.\n", error);
    else
        printf("Success!\n");

    return error ? 1 : 0;
}
