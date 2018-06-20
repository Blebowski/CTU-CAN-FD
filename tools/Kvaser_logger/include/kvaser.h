/*******************************************************************************
 * 
 * Kvaser / Altera Logger of CAN FD frames.
 * 
 * Module: Kvaser access functions.
 *
 * Copyright (C) 2015-2018 Ondrej Ille <ondrej.ille@gmail.com>
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



/*
 * CAN Frame structure
 *
 * New structure is created instead of Linux structure. Linux uses different
 * structs for CAN and CAN FD frames. Single struct is more pragmatic in this
 * application
 */
struct kvalt_can_frame {
	unsigned int data_length;
	long id;
	uint8_t id_type; 		// Identifier type: 0 - BASE, 1 - EXTENDED
	uint8_t fr_type; 		// Frame type: 0 - CAN 2.0, 1 - CAN FD
	uint8_t brs;     		// Bit rate shift flag
	uint8_t rtr;     		// Remote transmission request flag
	unsigned char data[64];		// Data bytes
};


/* 
 * Initializes Kvaser device via "canlib" with following settings:
 * 	1. Channels 0 an 1 are opened.
 *	2. Bit-rate on both channels is set to 500 Kb/s Nominal, 2Mb Data. 80 % Sample point.
 */
int kvaser_init(void);


/* 
 * Prints CAN frame to standard output.
 * 
 * Arguments:
 *	frame	Pointer to frame that should be printed.
 */
void print_can_frame(struct kvalt_can_frame *frame);


/* 
 * Generate random CAN / CAN FD frame. Following constraints are applied:
 *	1. RTR flag is only for CAN Frames.
 *	2. BRS Flag is only for CAN FD Frames.
 *	3. If BRS is false and FD frame is generated. Data length is
 *	   mostly 16 bytes.
 * 
 * Arguments:
 *	frame	Pointer where generated frame will be returned.
 */
int generate_can_frame(struct kvalt_can_frame *frame);


/* 
 * Send CAN Frame via Kvaser.
 *
 * Arguments:
 *	frame	Pointer to frame which should be sent
 *	channel Kvaser channel to send the frame on (0, 1)
 */
int kvaser_send_frame(struct kvalt_can_frame *frame, int channel);


/* 
 * Read CAN Frame via Kvaser.
 *
 * Arguments:
 *	frame	Pointer to frame in which read frame will be stored
 *	channel Kvaser channel to read frame from (0, 1)
 */
int kvaser_read_frame(struct kvalt_can_frame *frame, int channel);

