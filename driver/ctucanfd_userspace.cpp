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
#include <unistd.h>
#include <sys/types.h>
#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/*
/sys/devices/pci0000:00/0000:00:1c.4/0000:05:00.0
*/


const struct can_bittiming_const ctu_can_fd_bit_timing_max = {
	"ctu_can_fd",
	2,
	190,
	1,
	63,
	31,
	1,
	8,
	1
};

const struct can_bittiming_const ctu_can_fd_bit_timing_data_max = {
	"ctu_can_fd",
	2,
	94,
	1,
	31,
	31,
	1,
	2,
	1
};

typedef struct find_dir_chain {
    char *dir_name;
    DIR *dir;
    struct find_dir_chain *parent;
} find_dir_chain_t;

int pci_find_dev(char **dev_dir, const char *dir_name, unsigned vid, unsigned pid, int inst)
{
    unsigned vid_act, pid_act;
    find_dir_chain_t *dir_stack = NULL;
    find_dir_chain_t *dir_act = NULL;
    char *entry_name = NULL;
    FILE *f;
    struct dirent *entry;
    int level = 0;
    int sz1, sz2 = 0;
    char level0_prefix[] = "pci";
    int clean_rest = 0;

    *dev_dir = NULL;

    while(1) {
        if (dir_act != NULL) {
            while (((entry = readdir(dir_act->dir)) == NULL) || (clean_rest)) {
                closedir(dir_act->dir);
                free(dir_act->dir_name);
                dir_stack = dir_act->parent;
                free(dir_act);
                dir_act = dir_stack;
                level--;
                if (dir_stack == NULL)
                    break;
            }
            if (dir_act == NULL)
                break;
            if (entry->d_type != DT_DIR)
                continue;
            if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
                continue;
            dir_name = dir_stack->dir_name;
            entry_name = entry->d_name;
            if ((level <= 1) && strncmp(entry_name, level0_prefix, strlen(level0_prefix)))
                continue;
        }
        dir_act = (find_dir_chain_t *)malloc(sizeof(find_dir_chain_t));
        if (dir_act == NULL)
            err(1, "malloc failed");
        dir_act->parent = dir_stack;
        sz1 = strlen(dir_name);
        if (entry_name != NULL)
            sz2 = strlen(entry_name);
        dir_act->dir_name = (char *)malloc(sz1 + sz2 + 2);
        if (dir_act->dir_name == NULL)
            err(1, "malloc failed");
        memcpy(dir_act->dir_name, dir_name, sz1);
        if (entry_name != NULL) {
          dir_act->dir_name[sz1] = '/';
          memcpy(dir_act->dir_name + sz1 + 1, entry_name, sz2);
          dir_act->dir_name[sz1 + 1 + sz2] = 0;
        } else {
          dir_act->dir_name[sz1] = 0;
        }
        /*printf("level %d, name = %s\n", level, dir_act->dir_name);*/
        dir_stack = dir_act;
        dir_act->dir = opendir(dir_act->dir_name);
        if (dir_act->dir == NULL)
            err(1, "opendir failed");
        level++;
        if (level <= 2)
            continue;
        {
            int sz1 = strlen(dir_act->dir_name);
            char vid_fname[sz1 + strlen("/vendor") + 1];
            char pid_fname[sz1 + strlen("/device") + 1];
            memcpy(vid_fname, dir_act->dir_name, sz1);
            memcpy(pid_fname, dir_act->dir_name, sz1);
            strcpy(vid_fname + sz1, "/vendor");
            strcpy(pid_fname + sz1, "/device");
            f = fopen(vid_fname, "r");
            if (f == NULL)
                continue;
            vid_act = -1;
            fscanf(f, "%i", &vid_act);
            fclose(f);
            f = fopen(pid_fname, "r");
            if (f == NULL)
                continue;
            pid_act = -1;
            fscanf(f, "%i", &pid_act);
            fclose(f);
        }
        if (vid != vid_act)
            continue;
        if (pid != pid_act)
            continue;
        if (inst-- > 0)
            continue;
        *dev_dir = strdup(dir_act->dir_name);
        clean_rest = 1;
    }

    return clean_rest;
}

uintptr_t pci_find_bar(unsigned vid, unsigned pid, int inst, int barnr)
{
    FILE *f;
    char *dev_dir;
    int sz1;
    unsigned long long bar1_base;
    if (!pci_find_dev(&dev_dir, "/sys/devices", vid, pid, inst)) {
        return 0;
    }
    printf("found %s\n", dev_dir);
    sz1 = strlen(dev_dir);

    {
        char buff[200];
        char resource_fname[sz1 + strlen("/resource") + 1];
        memcpy(resource_fname, dev_dir, sz1);
        strcpy(resource_fname + sz1, "/resource");
        f = fopen(resource_fname, "r");
        if (f == NULL)
            return 0;
        while (barnr-- > 0)
            fgets(buff, sizeof(buff) - 1, f);
        fscanf(f, "%lli", &bar1_base);
        fclose(f);
    }
    return bar1_base;
}

static inline void
timespec_sub (struct timespec *diff, const struct timespec *left,
              const struct timespec *right)
{
  diff->tv_sec = left->tv_sec - right->tv_sec;
  diff->tv_nsec = left->tv_nsec - right->tv_nsec;

  if (diff->tv_nsec < 0)
    {
      --diff->tv_sec;
      diff->tv_nsec += 1000000000;
    }
}

int main(int argc, char *argv[])
{
    uintptr_t addr_base = 0;
    unsigned ifc = 0;
    bool do_transmit = false;
    int loop_cycle = 0;
    int gap = 1000;
    int bitrate = 1000000;
    int dbitrate = 0;
    int tx_can_id = 0x1ff;
    bool do_periodic_transmit = false;
    bool transmit_fdf = false;
    bool loopback_mode = false;
    bool test_read_speed = false;
    //bool do_showhelp = false;
    static uintptr_t addrs[] = {0x43C30000, 0x43C70000};

    int c;
    char *e;
    const char *progname = argv[0];
    while ((c = getopt(argc, argv, "i:a:g:b:B:I:fltThpr")) != -1) {
        switch (c) {
            case 'i':
                ifc = strtoul(optarg, &e, 0);
                if (*e != '\0')
                    err(1, "-i expects a number");
            break;

            case 'a':
                addr_base = strtoul(optarg, &e, 0);
                if (*e != '\0')
                    err(1, "-a expects a number");
            break;

            case 'g':
                gap = strtoul(optarg, &e, 0);
                if (*e != '\0')
                    err(1, "-g expects a number");
            break;

            case 'b':
                bitrate = strtoul(optarg, &e, 0);
                if (*e != '\0')
                    err(1, "-b expects a number");
            break;

            case 'B':
                dbitrate = strtoul(optarg, &e, 0);
                if (*e != '\0')
                    err(1, "-B expects a number");
            break;

            case 'I':
                tx_can_id = strtoul(optarg, &e, 0);
                if (*e != '\0')
                    err(1, "-I expects a number");
            break;

            case 'l': loopback_mode = true; break;
            case 't': do_transmit = true; break;
            case 'T': do_periodic_transmit = true; break;
            case 'f': transmit_fdf = true; break;
            case 'r': test_read_speed  = true; break;
            case 'p':
                addrs[0] = pci_find_bar(0x1172, 0xcafd, 0, 1);
                if (!addrs[0])
                    addrs[0] = pci_find_bar(0x1760, 0xff00, 0, 1);
                if (!addrs[0])
                    err(1, "-p PCI device not found");
                addrs[1] = addrs[0] + 0x4000;
            break;
            case 'h':
                printf("Usage: %s [-i ifc] [-a address] [-l] [-t] [-T]\n\n"
                       "  -t: Transmit\n",
                       progname
                );
                return 0;
        }
    }

    if (ifc >= 2) {
        std::cerr << "Err: ifc number must be 0 or 1.\n";
        exit(1);
    }
    if (addr_base == 0)
        addr_base = addrs[ifc];

    struct ctucan_hw_priv *priv = ctucanfd_init(addr_base);
    int res;

    union ctu_can_fd_device_id_version reg;
    reg.u32 = priv->read_reg(priv, CTU_CAN_FD_DEVICE_ID);

    printf("DevID: 0x%08x, should be 0x%08x\n", reg.s.device_id, CTU_CAN_FD_ID);

    if (!ctucan_hw_check_access(priv))
        errx(1, "error: ctucan_hw_check_access");

    u32 version = ctucan_hw_get_version(priv);
    printf("Core version: %u\n", version);

    //struct can_ctrlmode ctrlmode = {CAN_CTRLMODE_FD, CAN_CTRLMODE_FD};
    //ctucan_hw_set_mode(priv, &ctrlmode);

    if (test_read_speed) {
        struct timespec tic, tac, diff;
	int i;
	u32 dummy;
	(void)dummy;
        clock_gettime(CLOCK_MONOTONIC, &tic);
	for (i = 0; i < 1000 * 1000; i++) {
		dummy = ctucan_hw_read32(priv, CTU_CAN_FD_RX_DATA);
	}
        clock_gettime(CLOCK_MONOTONIC, &tac);

	timespec_sub(&diff, &tac, &tic);
	printf("%d reads takes %ld.%09ld s\n",
	       i, (long)diff.tv_sec, diff.tv_nsec);

	dummy = 0;
        clock_gettime(CLOCK_MONOTONIC, &tic);
	for (i = 0; i < 1000 * 1000; i++) {
		ctucan_hw_write32(priv, CTU_CAN_FD_FILTER_C_VAL, dummy);
	}
        clock_gettime(CLOCK_MONOTONIC, &tac);

	timespec_sub(&diff, &tac, &tic);
	printf("%d writes takes %ld.%09ld s\n",
	       i, (long)diff.tv_sec, diff.tv_nsec);

        return 0;
    }


    //printf("NOT RESETTING!\n");
    ctucan_hw_reset(priv);

    {
        union ctu_can_fd_mode_settings mode;
        mode.u32 = priv->read_reg(priv, CTU_CAN_FD_MODE);

        if (mode.s.ena) {
            printf("Core is enabled but should be disabled!\n");
        }
    }

    struct net_device nd;
    nd.can.clock.freq = 100000000;

    struct can_bittiming nom_timing = {
        .bitrate = bitrate,
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

    if (!dbitrate)
        dbitrate = 10 * bitrate;

    struct can_bittiming data_timing = {
        .bitrate = dbitrate,
    };
    res = can_get_bittiming(&nd, &data_timing,
                      &ctu_can_fd_bit_timing_data_max,
                      NULL,
                      0);
    if (res)
        err(res, "can_get_bittiming data");
    printf("data sample_point .%03d, tq %d, prop %d, seg1 %d, seg2 %d, sjw %d, brp %d, bitrate %d\n",
           data_timing.sample_point,
           data_timing.tq,
           data_timing.prop_seg,
           data_timing.phase_seg1,
           data_timing.phase_seg2,
           data_timing.sjw,
           data_timing.brp,
           data_timing.bitrate
    );

    //priv->write_reg(priv, CTU_CAN_FD_INT_MASK_CLR, 0xffff);
    //priv->write_reg(priv, CTU_CAN_FD_INT_ENA_SET, 0xffff);
    ctucan_hw_set_nom_bittiming(priv, &nom_timing);
    ctucan_hw_set_data_bittiming(priv, &data_timing);
    //ctucan_hw_rel_rx_buf(priv);
    //ctucan_hw_set_ret_limit(priv, true, 1);
    //ctucan_hw_set_ret_limit(priv, false, 0);
    //ctucan_hw_abort_tx(priv);
    //ctucan_hw_txt_set_abort(priv, CTU_CAN_FD_TXT_BUFFER_1);
    //ctucan_hw_txt_set_empty(priv, CTU_CAN_FD_TXT_BUFFER_1);

    if (loopback_mode) {
        struct can_ctrlmode mode = {0, 0};
	mode.mask  = CAN_CTRLMODE_LOOPBACK | CAN_CTRLMODE_PRESUME_ACK;
	mode.flags = CAN_CTRLMODE_LOOPBACK | CAN_CTRLMODE_PRESUME_ACK;
        ctucan_hw_set_mode(priv, &mode);
    }

    ctucan_hw_enable(priv, true);
    usleep(10000);

    printf("MODE=0x%02x\n", priv->read_reg(priv, CTU_CAN_FD_MODE));

    if (do_transmit) {
        struct canfd_frame txf;
        txf.can_id = tx_can_id;
        txf.flags = 0;
        //u8 d[] = {0xde, 0xad, 0xbe, 0xef};
        u8 d[] = {0xDE, 0xAD, 0xBE, 0xEF};
        memcpy(txf.data, d, sizeof(d));
        txf.len = sizeof(d);

        res = ctucan_hw_insert_frame(priv, &txf, 0, CTU_CAN_FD_TXT_BUFFER_1, false);
        if (!res)
            printf("TX failed\n");
        ctucan_hw_txt_set_rdy(priv, CTU_CAN_FD_TXT_BUFFER_1);
        return 0;
    }

    while (1) {
        u32 nrxf = ctucan_hw_get_rx_frame_count(priv);//ctucan_hw_get_rx_frame_ctr(priv);
        union ctu_can_fd_rx_mem_info reg;
        reg.u32 = ctucan_hw_read32(priv, CTU_CAN_FD_RX_MEM_INFO);
        u32 rxsz = reg.s.rx_buff_size - reg.s.rx_mem_free;
        union ctu_can_fd_status status = ctu_can_get_status(priv);
        union ctu_can_fd_err_capt_retr_ctr_alc err_capt_alc;
        union ctu_can_fd_int_stat int_stat = ctu_can_fd_int_sts(priv);
        ctucan_hw_int_clr(priv, int_stat);


        printf("%u RX frames, %u words", nrxf, rxsz);
        printf(", status 0x%08hx", status.u32);
        printf(", settings 0x%04hhx", ctu_can_fd_read16(priv, CTU_CAN_FD_SETTINGS));
        printf(", INT_STAT 0x%04hx", int_stat.u32);
        printf(", INT_ENA_SET 0x%04hx", priv->read_reg(priv, CTU_CAN_FD_INT_ENA_SET));
        printf(", INT_MASK_SET 0x%04hx", priv->read_reg(priv, CTU_CAN_FD_INT_MASK_SET));
        printf(", TX_STATUS 0x%04hx", priv->read_reg(priv, CTU_CAN_FD_TX_STATUS));
        //printf(", CTU_CAN_FD_ERR_CAPT 0x%02hhx", ctu_can_fd_read8(priv, CTU_CAN_FD_ERR_CAPT));
        printf(", TRV_DELAY 0x%0hx", priv->read_reg(priv, CTU_CAN_FD_TRV_DELAY));

        printf("\n");

        if (status.s.ewl) {
            err_capt_alc = ctu_can_fd_read_err_capt_alc(priv);
            printf("ERROR type %u pos %u ALC id_field %u bit %u\n",
                err_capt_alc.s.err_type, err_capt_alc.s.err_pos,
                err_capt_alc.s.alc_id_field, err_capt_alc.s.alc_bit);
	}

        /*
        while (rxsz--) {
            u32 data = priv->read_reg(priv, CTU_CAN_FD_RX_DATA);
            printf("  0x%08x\n", data);
        }
        */
        while (nrxf || rxsz) {
            struct canfd_frame cf;
            u64 ts;
            ctucan_hw_read_rx_frame(priv, &cf, &ts);
            printf("%llu: #%x [%u]", ts, cf.can_id, cf.len);
            for (int i=0; i<cf.len; ++i)
                printf(" %02x", cf.data[i]);
            printf("\n");
            rxsz = 0;
            nrxf = ctucan_hw_get_rx_frame_count(priv);
        }

        if (do_periodic_transmit && (loop_cycle & 1)) {
	    struct canfd_frame txf;
	    memset(&txf, 0, sizeof(txf));
            txf.can_id = tx_can_id;
            txf.flags = 0;

            if (transmit_fdf) {
                txf.flags |= CANFD_BRS;
                u8 dfd[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee};
                memcpy(txf.data, dfd, sizeof(dfd));
                txf.len = sizeof(dfd);
	    } else {
                u8 d[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0xEE};
                memcpy(txf.data, d, sizeof(d));
                txf.len = sizeof(d);
	    }

            res = ctucan_hw_insert_frame(priv, &txf, 0, CTU_CAN_FD_TXT_BUFFER_1, transmit_fdf);
            if (!res)
                printf("TX failed\n");
            ctucan_hw_txt_set_rdy(priv, CTU_CAN_FD_TXT_BUFFER_1);

        }

        usleep(1000 * gap);
        loop_cycle++;
    }

    return 0;
}
