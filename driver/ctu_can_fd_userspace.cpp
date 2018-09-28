/*******************************************************************************
 * 
 * CTU CAN FD IP Core
 * Copyright (C) 2015-2018
 * 
 * Authors:
 *     Ondrej Ille <ondrej.ille@gmail.com>
 *     Martin Jerabek <martin.jerabek01@gmail.com>
 * 
 * Project advisors: 
 * 	Jiri Novak <jnovak@fel.cvut.cz>
 * 	Pavel Pisa <pisa@cmp.felk.cvut.cz>
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
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 * 
*******************************************************************************/

#include "userspace_utils.h"

#include <iostream>


int main(int argc, char *argv[])
{
    uint32_t addr_base;
    unsigned ifc = 0;
    bool do_transmit = false;
    //bool do_showhelp = false;

    int c;
    char *e;
    const char *progname = argv[0];
    while ((c = getopt(argc, argv, "i:th")) != -1) {
        switch (c) {
            case 'i':
                ifc = strtoul(optarg, &e, 0);
                if (*e != '\0')
                    err(1, "-i expects a number");
            break;
            case 't': do_transmit = true; break;
            case 'h':
                printf("Usage: %s [-i ifc] [-t]\n\n"
                       "  -t: Transmit\n",
                       progname
                );
                return 0;
        }
    }

    static const uint32_t addrs[] = {0x43C30000, 0x43C70000};
    if (ifc >= 2) {
        std::cerr << "Err: ifc number must be 0 or 1.\n";
        exit(1);
    }
    addr_base = addrs[ifc];

    struct ctucanfd_priv *priv = ctucanfd_init(addr_base);
    int res;

    union ctu_can_fd_device_id_version reg;
    reg.u32 = priv->read_reg(priv, CTU_CAN_FD_DEVICE_ID);

    printf("DevID: 0x%08x, should be 0x%08x\n", reg.s.device_id, CTU_CAN_FD_ID);

    if (!ctu_can_fd_check_access(priv))
        errx(1, "error: ctu_can_fd_check_access");

    u32 version = ctu_can_fd_get_version(priv);
    printf("Core version: %u\n", version);

    //struct can_ctrlmode ctrlmode = {CAN_CTRLMODE_FD, CAN_CTRLMODE_FD};
    //ctu_can_fd_set_mode(priv, &ctrlmode);



    //printf("NOT RESETTING!\n");
    ctu_can_fd_reset(priv);

    {
        union ctu_can_fd_mode_command_status_settings mode;
        mode.u32 = priv->read_reg(priv, CTU_CAN_FD_MODE);

        if (mode.s.ena) {
            printf("Core is enabled but should be disabled!\n");
        }
    }

    struct net_device nd;
    nd.can.clock.freq = 100000000;

    struct can_bittiming nom_timing = {
        .bitrate = 1000000,
    };
    res = can_get_bittiming(&nd, &nom_timing,
                      &ctu_can_fd_bit_timing_max,
                      NULL,
                      0);
    if (res)
        err(res, "can_get_bittiming");
    printf("sample_point .%03d, tq %d, prop %d, seg1 %d, seg2 %d, sjw %d, brp %d, bitrate %d\n",
           nom_timing.sample_point,
           nom_timing.tq,
           nom_timing.prop_seg,
           nom_timing.phase_seg1,
           nom_timing.phase_seg2,
           nom_timing.sjw,
           nom_timing.brp,
           nom_timing.bitrate
    );

    //priv->write_reg(priv, CTU_CAN_FD_INT_MASK_CLR, 0xffff);
    //priv->write_reg(priv, CTU_CAN_FD_INT_ENA_SET, 0xffff);
    ctu_can_fd_set_nom_bittiming(priv, &nom_timing);
    //ctu_can_fd_rel_rx_buf(priv);
    //ctu_can_fd_set_ret_limit(priv, true, 1);
    //ctu_can_fd_set_ret_limit(priv, false, 0);
    //ctu_can_fd_abort_tx(priv);
    //ctu_can_fd_txt_set_abort(priv, CTU_CAN_FD_TXT_BUFFER_1);
    //ctu_can_fd_txt_set_empty(priv, CTU_CAN_FD_TXT_BUFFER_1);
    ctu_can_fd_enable(priv, true);
    usleep(10000);


    if (do_transmit) {
        struct canfd_frame txf;
        txf.can_id = 0x1FF;
        txf.flags = 0;
        //u8 d[] = {0xde, 0xad, 0xbe, 0xef};
        u8 d[] = {0xDE, 0xAD, 0xBE, 0xEF};
        memcpy(txf.data, d, sizeof(d));
        txf.len = sizeof(d);

        res = ctu_can_fd_insert_frame(priv, &txf, 0, CTU_CAN_FD_TXT_BUFFER_1, false);
        if (!res)
            printf("TX failed\n");
        ctu_can_fd_txt_set_rdy(priv, CTU_CAN_FD_TXT_BUFFER_1);
        return 0;
    }

    while (1) {
        u32 nrxf = ctu_can_fd_get_rx_frame_count(priv);//ctu_can_fd_get_rx_frame_ctr(priv);
        union ctu_can_fd_rx_mem_info reg;
        reg.u32 = ctu_can_fd_read32(priv, CTU_CAN_FD_RX_MEM_INFO);
        u32 rxsz = reg.s.rx_buff_size - reg.s.rx_mem_free;

        printf("%u RX frames, %u words", nrxf, rxsz);
        printf(", status 0x%02hhx", ctu_can_fd_read8(priv, CTU_CAN_FD_STATUS));
        printf(", settings 0x%02hhx", ctu_can_fd_read8(priv, CTU_CAN_FD_SETTINGS));
        printf(", INT_STAT 0x%04hhx", ctu_can_fd_read16(priv, CTU_CAN_FD_INT_STAT));
        printf(", CTU_CAN_FD_INT_ENA_SET 0x%04hx", priv->read_reg(priv, CTU_CAN_FD_INT_ENA_SET));
        printf(", CTU_CAN_FD_INT_MASK_SET 0x%04hx", priv->read_reg(priv, CTU_CAN_FD_INT_MASK_SET));
        printf(", CTU_CAN_FD_TX_STATUS 0x%04hx", priv->read_reg(priv, CTU_CAN_FD_TX_STATUS));
        //printf(", CTU_CAN_FD_ERR_CAPT 0x%02hhx", ctu_can_fd_read8(priv, CTU_CAN_FD_ERR_CAPT));


        printf("\n");
        /*
        while (rxsz--) {
            u32 data = priv->read_reg(priv, CTU_CAN_FD_RX_DATA);
            printf("  0x%08x\n", data);
        }
        */
        if (nrxf || rxsz) {
            struct canfd_frame cf;
            u64 ts;
            ctu_can_fd_read_rx_frame(priv, &cf, &ts);
            printf("%llu: #%x [%u]", ts, cf.can_id, cf.len);
            for (int i=0; i<cf.len; ++i)
                printf(" %02x", cf.data[i]);
            printf("\n");
        }

        usleep(1000000);
    }

    return 0;
}
