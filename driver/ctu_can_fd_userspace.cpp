extern "C" {
#include "ctu_can_fd_linux_defs.h"
#include "ctu_can_fd_hw.h"
}

#undef abs
#include <sys/mman.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <inttypes.h>
#include <err.h>

//#include "clara.hpp"
#include <iostream>

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
/*
    using namespace clara;
    auto cli = //Opt(addr_base, "addr_base")
               //    ["-A"]["--addr_base"]("CAN FD registers base address")
               Opt(ifc, "ifc") ["-i"]("CAN FD interface number")
             | Opt(do_transmit)
                   ["-t"]("Do transmit")
             | Help(do_showhelp)
             ;
             std::cout << "IFC:" << ifc << std::endl;

    auto result = cli.parse(Args(argc, argv));

    if (!result) {
        std::cerr << "Error in command line: " << result.errorMessage() << std::endl;
        exit(1);
    }
    if (do_showhelp) {
        std::cout << cli;
        return 0;
    }
*/
    static const uint32_t addrs[] = {0x43C30000, 0x43C70000};
    if (ifc >= 2) {
        std::cerr << "Err: ifc number must be 0 or 1.\n";
        exit(1);
    }
    addr_base = addrs[ifc];

    mem_open();



    volatile void * const base = mem_map(addr_base, CANFD_ADDR_RANGE);
    struct ctucanfd_priv _p, *priv = &_p;
    int res;

    priv->mem_base = base;
    priv->read_reg = ctu_can_fd_read32;
    priv->write_reg = ctu_can_fd_write32;

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
