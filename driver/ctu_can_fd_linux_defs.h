/*******************************************************************************
 * 
 * CTU CAN FD IP Core
 * Copyright (C) 2015-2018 Ondrej Ille <ondrej.ille@gmail.com>
 * 
 * Project advisors and co-authors: 
 * 	Jiri Novak <jnovak@fel.cvut.cz>
 * 	Pavel Pisa <pisa@cmp.felk.cvut.cz>
 * 	Martin Jerabek <jerabma7@fel.cvut.cz>
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

/* Linux declarations used in the low-level driver. When using driver with
 * SocketCAN, this file is not necessary. When using the driver outside of
 * Linux, this file replaces common kernel headers. */

#ifndef __CTU_CAN_FD_LINUX_DEFS__
#define __CTU_CAN_FD_LINUX_DEFS__

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
//#include <linux/types.h>
#include <linux/socket.h>
#include <arpa/inet.h>
#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)
#define __iomem volatile

typedef uint8_t   __u8;
typedef uint16_t  __u16;
typedef uint32_t  __u32;
typedef uint64_t  __u64;

typedef int8_t   __s8;
typedef int16_t  __s16;
typedef int32_t  __s32;
typedef int64_t  __s64;

typedef __u8  u8;
typedef __u16 u16;
typedef __u32 u32;
typedef __u64 u64;

typedef __s8  s8;
typedef __s16 s16;
typedef __s32 s32;
typedef __s64 s64;

#ifndef __cplusplus
typedef _Bool			bool;
enum {
	false	= 0,
	true	= 1
};
#endif

#define __WARN_printf printf
#ifndef WARN
#define WARN(condition, format...) ({						\
int __ret_warn_on = !!(condition);				\
if (unlikely(__ret_warn_on))					\
    __WARN_printf(format);					\
    unlikely(__ret_warn_on);					\
})
#endif

/*
// from include/uapi/linux/types.h
#define __bitwise

typedef __u16 __bitwise __le16;
typedef __u16 __bitwise __be16;
typedef __u32 __bitwise __le32;
typedef __u32 __bitwise __be32;
typedef __u64 __bitwise __le64;
typedef __u64 __bitwise __be64;

typedef __u16 __bitwise __sum16;
typedef __u32 __bitwise __wsum;
*/

__attribute__((noinline))
static inline void iowrite32(u32 value, void *addr) {*(volatile u32*)addr = value;}
__attribute__((noinline))
static inline void iowrite16(u16 value, void *addr) {*(volatile u16*)addr = value;}
__attribute__((noinline))
static inline void iowrite8(u8 value, void *addr) {*(volatile u8*)addr = value;}

__attribute__((noinline))
static inline u32 ioread32(const void *addr) {return *(const volatile u32*)addr;}
__attribute__((noinline))
static inline u16 ioread16(const void *addr) {return *(const volatile u16*)addr;}
__attribute__((noinline))
static inline u8 ioread8(const void *addr) {return *(const volatile u8*)addr;}

static inline u32 cpu_to_be32(u32 v) {return htonl(v);}
static inline u32 be32_to_cpu(u32 v) {return ntohl(v);}

__attribute__((noinline))
static inline void iowrite32be(u32 value, void *addr) {*(volatile u32*)addr = cpu_to_be32(value);}
__attribute__((noinline))
static inline u32 ioread32be(const void *addr) {return be32_to_cpu(*(const volatile u32*)addr);}

/* CAN DLC to real data length conversion helpers */
u8 can_dlc2len(u8 can_dlc);
u8 can_len2dlc(u8 len);

/*
 * CAN bit-timing parameters
 *
 * For further information, please read chapter "8 BIT TIMING
 * REQUIREMENTS" of the "Bosch CAN Specification version 2.0"
 * at http://www.semiconductors.bosch.de/pdf/can2spec.pdf.
 */
struct can_bittiming {
	__u32 bitrate;		/* Bit-rate in bits/second */
	__u32 sample_point;	/* Sample point in one-tenth of a percent */
	__u32 tq;		/* Time quanta (TQ) in nanoseconds */
	__u32 prop_seg;		/* Propagation segment in TQs */
	__u32 phase_seg1;	/* Phase buffer segment 1 in TQs */
	__u32 phase_seg2;	/* Phase buffer segment 2 in TQs */
	__u32 sjw;		/* Synchronisation jump width in TQs */
	__u32 brp;		/* Bit-rate prescaler */
};

/*
 * CAN harware-dependent bit-timing constant
 *
 * Used for calculating and checking bit-timing parameters
 */
struct can_bittiming_const {
	char name[16];		/* Name of the CAN controller hardware */
	__u32 tseg1_min;	/* Time segement 1 = prop_seg + phase_seg1 */
	__u32 tseg1_max;
	__u32 tseg2_min;	/* Time segement 2 = phase_seg2 */
	__u32 tseg2_max;
	__u32 sjw_max;		/* Synchronisation jump width */
	__u32 brp_min;		/* Bit-rate prescaler */
	__u32 brp_max;
	__u32 brp_inc;
};

/*
 * CAN clock parameters
 */
struct can_clock {
	__u32 freq;		/* CAN system clock frequency in Hz */
};

/*
 * CAN operational and error states
 */
enum can_state {
	CAN_STATE_ERROR_ACTIVE = 0,	/* RX/TX error count < 96 */
	CAN_STATE_ERROR_WARNING,	/* RX/TX error count < 128 */
	CAN_STATE_ERROR_PASSIVE,	/* RX/TX error count < 256 */
	CAN_STATE_BUS_OFF,		/* RX/TX error count >= 256 */
	CAN_STATE_STOPPED,		/* Device is stopped */
	CAN_STATE_SLEEPING,		/* Device is sleeping */
	CAN_STATE_MAX
};

/*
 * CAN bus error counters
 */
struct can_berr_counter {
	__u16 txerr;
	__u16 rxerr;
};

/*
 * CAN controller mode
 */
struct can_ctrlmode {
	__u32 mask;
	__u32 flags;
};

#define CAN_CTRLMODE_LOOPBACK		0x01	/* Loopback mode */
#define CAN_CTRLMODE_LISTENONLY		0x02	/* Listen-only mode */
#define CAN_CTRLMODE_3_SAMPLES		0x04	/* Triple sampling mode */
#define CAN_CTRLMODE_ONE_SHOT		0x08	/* One-Shot mode */
#define CAN_CTRLMODE_BERR_REPORTING	0x10	/* Bus-error reporting */
#define CAN_CTRLMODE_FD			0x20	/* CAN FD mode */
#define CAN_CTRLMODE_PRESUME_ACK	0x40	/* Ignore missing CAN ACKs */
#define CAN_CTRLMODE_FD_NON_ISO		0x80	/* CAN FD in non-ISO mode */

/*
 * CAN device statistics
 */
struct can_device_stats {
	__u32 bus_error;	/* Bus errors */
	__u32 error_warning;	/* Changes to error warning state */
	__u32 error_passive;	/* Changes to error passive state */
	__u32 bus_off;		/* Changes to bus off state */
	__u32 arbitration_lost; /* Arbitration lost errors */
	__u32 restarts;		/* CAN controller re-starts */
};

/*
 * CAN netlink interface
 */
enum {
	IFLA_CAN_UNSPEC,
	IFLA_CAN_BITTIMING,
	IFLA_CAN_BITTIMING_CONST,
	IFLA_CAN_CLOCK,
	IFLA_CAN_STATE,
	IFLA_CAN_CTRLMODE,
	IFLA_CAN_RESTART_MS,
	IFLA_CAN_RESTART,
	IFLA_CAN_BERR_COUNTER,
	IFLA_CAN_DATA_BITTIMING,
	IFLA_CAN_DATA_BITTIMING_CONST,
	IFLA_CAN_TERMINATION,
	IFLA_CAN_TERMINATION_CONST,
	IFLA_CAN_BITRATE_CONST,
	IFLA_CAN_DATA_BITRATE_CONST,
	IFLA_CAN_BITRATE_MAX,
	__IFLA_CAN_MAX
};







/* SPDX-License-Identifier: ((GPL-2.0 WITH Linux-syscall-note) OR BSD-3-Clause) */
/*
 * linux/can.h
 *
 * Definitions for CAN network layer (socket addr / CAN frame / CAN filter)
 *
 * Authors: Oliver Hartkopp <oliver.hartkopp@volkswagen.de>
 *          Urs Thuermann   <urs.thuermann@volkswagen.de>
 * Copyright (c) 2002-2007 Volkswagen Group Electronic Research
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of Volkswagen nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * Alternatively, provided that this notice is retained in full, this
 * software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2, in which case the provisions of the
 * GPL apply INSTEAD OF those given above.
 *
 * The provided data structures and external interfaces from this code
 * are not restricted to be used by modules with a GPL compatible license.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#ifndef _UAPI_CAN_H
#define _UAPI_CAN_H




/* controller area network (CAN) kernel definitions */

/* special address description flags for the CAN_ID */
#define CAN_EFF_FLAG 0x80000000U /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000U /* remote transmission request */
#define CAN_ERR_FLAG 0x20000000U /* error message frame */

/* valid bits in CAN ID for frame formats */
#define CAN_SFF_MASK 0x000007FFU /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFU /* extended frame format (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFU /* omit EFF, RTR, ERR flags */

/*
 * Controller Area Network Identifier structure
 *
 * bit 0-28	: CAN identifier (11/29 bit)
 * bit 29	: error message frame flag (0 = data frame, 1 = error message)
 * bit 30	: remote transmission request flag (1 = rtr frame)
 * bit 31	: frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
 */
typedef __u32 canid_t;

#define CAN_SFF_ID_BITS		11
#define CAN_EFF_ID_BITS		29

/*
 * Controller Area Network Error Message Frame Mask structure
 *
 * bit 0-28	: error class mask (see include/linux/can/error.h)
 * bit 29-31	: set to zero
 */
typedef __u32 can_err_mask_t;

/* CAN payload length and DLC definitions according to ISO 11898-1 */
#define CAN_MAX_DLC 8
#define CAN_MAX_DLEN 8

/* CAN FD payload length and DLC definitions according to ISO 11898-7 */
#define CANFD_MAX_DLC 15
#define CANFD_MAX_DLEN 64

/**
 * struct can_frame - basic CAN frame structure
 * @can_id:  CAN ID of the frame and CAN_*_FLAG flags, see canid_t definition
 * @can_dlc: frame payload length in byte (0 .. 8) aka data length code
 *           N.B. the DLC field from ISO 11898-1 Chapter 8.4.2.3 has a 1:1
 *           mapping of the 'data length code' to the real payload length
 * @__pad:   padding
 * @__res0:  reserved / padding
 * @__res1:  reserved / padding
 * @data:    CAN frame payload (up to 8 byte)
 */
struct can_frame {
	canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	__u8    can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
	__u8    __pad;   /* padding */
	__u8    __res0;  /* reserved / padding */
	__u8    __res1;  /* reserved / padding */
	__u8    data[CAN_MAX_DLEN] __attribute__((aligned(8)));
};


/*
 * defined bits for canfd_frame.flags
 *
 * The use of struct canfd_frame implies the Extended Data Length (EDL) bit to
 * be set in the CAN frame bitstream on the wire. The EDL bit switch turns
 * the CAN controllers bitstream processor into the CAN FD mode which creates
 * two new options within the CAN FD frame specification:
 *
 * Bit Rate Switch - to indicate a second bitrate is/was used for the payload
 * Error State Indicator - represents the error state of the transmitting node
 *
 * As the CANFD_ESI bit is internally generated by the transmitting CAN
 * controller only the CANFD_BRS bit is relevant for real CAN controllers when
 * building a CAN FD frame for transmission. Setting the CANFD_ESI bit can make
 * sense for virtual CAN interfaces to test applications with echoed frames.
 */
#define CANFD_BRS 0x01 /* bit rate switch (second bitrate for payload data) */
#define CANFD_ESI 0x02 /* error state indicator of the transmitting node */

/**
 * struct canfd_frame - CAN flexible data rate frame structure
 * @can_id: CAN ID of the frame and CAN_*_FLAG flags, see canid_t definition
 * @len:    frame payload length in byte (0 .. CANFD_MAX_DLEN)
 * @flags:  additional flags for CAN FD
 * @__res0: reserved / padding
 * @__res1: reserved / padding
 * @data:   CAN FD frame payload (up to CANFD_MAX_DLEN byte)
 */
struct canfd_frame {
	canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	__u8    len;     /* frame payload length in byte */
	__u8    flags;   /* additional flags for CAN FD */
	__u8    __res0;  /* reserved / padding */
	__u8    __res1;  /* reserved / padding */
	__u8    data[CANFD_MAX_DLEN] __attribute__((aligned(8)));
};

#define CAN_MTU		(sizeof(struct can_frame))
#define CANFD_MTU	(sizeof(struct canfd_frame))

/* particular protocols of the protocol family PF_CAN */
#define CAN_RAW		1 /* RAW sockets */
#define CAN_BCM		2 /* Broadcast Manager */
#define CAN_TP16	3 /* VAG Transport Protocol v1.6 */
#define CAN_TP20	4 /* VAG Transport Protocol v2.0 */
#define CAN_MCNET	5 /* Bosch MCNet */
#define CAN_ISOTP	6 /* ISO 15765-2 Transport Protocol */
#define CAN_J1939	7 /* SAE J1939 */
#define CAN_NPROTO	8

#define SOL_CAN_BASE 100

/*
 * This typedef was introduced in Linux v3.1-rc2
 * (commit 6602a4b net: Make userland include of netlink.h more sane)
 * in <linux/socket.h>. It must be duplicated here to make the CAN
 * headers self-contained.
 */
typedef unsigned short __kernel_sa_family_t;

/**
 * struct sockaddr_can - the sockaddr structure for CAN sockets
 * @can_family:  address family number AF_CAN.
 * @can_ifindex: CAN network interface index.
 * @can_addr:    protocol specific address information
 */
struct sockaddr_can {
	__kernel_sa_family_t can_family;
	int         can_ifindex;
	union {
		/* transport protocol class address information (e.g. ISOTP) */
		struct {
			canid_t rx_id;
			canid_t tx_id;
		} tp;

		/* J1939 address information */
		struct {
			/* 8 byte name when using dynamic addressing */
			__u64 name;

			/* pgn:
			 * 8 bit: PS in PDU2 case, else 0
			 * 8 bit: PF
			 * 1 bit: DP
			 * 1 bit: reserved
			 */
			__u32 pgn;

			/* 1 byte address */
			__u8 addr;
		} j1939;

		/* reserved for future CAN protocols address information */
	} can_addr;
};

/**
 * struct can_filter - CAN ID based filter in can_register().
 * @can_id:   relevant bits of CAN ID which are not masked out.
 * @can_mask: CAN mask (see description)
 *
 * Description:
 * A filter matches, when
 *
 *          <received_can_id> & mask == can_id & mask
 *
 * The filter can be inverted (CAN_INV_FILTER bit set in can_id) or it can
 * filter for error message frames (CAN_ERR_FLAG bit set in mask).
 */
struct can_filter {
	canid_t can_id;
	canid_t can_mask;
};

#define CAN_INV_FILTER 0x20000000U /* to be set in can_filter.can_id */
#define CAN_RAW_FILTER_MAX 512 /* maximum number of can_filter set via setsockopt() */
#endif /* _UAPI_CAN_H */


struct can_priv {
    struct can_bittiming bittiming, data_bittiming;
    const struct can_bittiming_const *bittiming_const,
        *data_bittiming_const;
    //const u16 *termination_const;
    //unsigned int termination_const_cnt;
    //u16 termination;
    const u32 *bitrate_const;
    unsigned int bitrate_const_cnt;
    const u32 *data_bitrate_const;
    unsigned int data_bitrate_const_cnt;
    struct can_clock clock;
};

struct net_device {
    struct can_priv can;
};

#define netdev_priv(nd) (&((nd)->can))

int can_get_bittiming(struct net_device *dev, struct can_bittiming *bt,
                      const struct can_bittiming_const *btc,
                      const u32 *bitrate_const,
                      const unsigned int bitrate_const_cnt);

#define min(a, b) (a < b ? a : b)
#define max(a, b) (a > b ? a : b)
#define clamp(val, lo, hi)  min((typeof(val))max(val, lo), hi)

/**
 * do_div - returns 2 values: calculate remainder and update new dividend
 * @n: pointer to uint64_t dividend (will be updated)
 * @base: uint32_t divisor
 *
 * Summary:
 * ``uint32_t remainder = *n % base;``
 * ``*n = *n / base;``
 *
 * Return: (uint32_t)remainder
 *
 * NOTE: macro parameter @n is evaluated multiple times,
 * beware of side effects!
 */
# define do_div(n,base) ({					\
	uint32_t __base = (base);				\
	uint32_t __rem;						\
	__rem = ((uint64_t)(n)) % __base;			\
	(n) = ((uint64_t)(n)) / __base;				\
	__rem;							\
 })
/**
 * abs - return absolute value of an argument
 * @x: the value.  If it is unsigned type, it is converted to signed type first.
 *     char is treated as if it was signed (regardless of whether it really is)
 *     but the macro's return type is preserved as char.
 *
 * Return: an absolute value of x.
 */
#define abs(x)	__abs_choose_expr(x, long long,				\
		__abs_choose_expr(x, long,				\
		__abs_choose_expr(x, int,				\
		__abs_choose_expr(x, short,				\
		__abs_choose_expr(x, char,				\
		__builtin_choose_expr(					\
			__builtin_types_compatible_p(typeof(x), char),	\
			(char)({ signed char __x = (x); __x<0?-__x:__x; }), \
			((void)0)))))))

#define __abs_choose_expr(x, type, other) __builtin_choose_expr(	\
	__builtin_types_compatible_p(typeof(x),   signed type) ||	\
	__builtin_types_compatible_p(typeof(x), unsigned type),		\
	({ signed type __x = (x); __x < 0 ? -__x : __x; }), other)

#define netdev_warn(dev, format, ...) printf("%s" format, "netdev_warn: ", ##__VA_ARGS__);
#define netdev_err(dev, format, ...) printf("%s" format, "netdev_err: ", ##__VA_ARGS__);

#endif /* __CTU_CAN_FD_LINUX_DEFS__ */
