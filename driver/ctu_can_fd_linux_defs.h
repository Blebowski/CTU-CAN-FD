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

#endif
