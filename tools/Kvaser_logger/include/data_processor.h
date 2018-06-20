/*******************************************************************************
 * 
 * Kvaser / Altera Logger of CAN FD frames.
 * 
 * Module: Text processor for parsing Signal TAP II exported data and
 *	    generation of output files for VHDL testbenches.
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
 * Processes Signal TAP II exported file and stores logged bit sequence
 * to output file. Data are stored like so:
 * 	<CAN FRAME INFO> <BIT SEQUENCE>
 *
 * Bit sequence is coded like so:
 *	<NUM> <VAL>
 * where NUM reffers to number of consecutive ocurrences of VAL. 
 * E.g. sequence:
 *   4 0 5 1 3 0
 * Represents:
 *   000011111000
 *
 * Arguments:
 *	frame	Pointer to frame which should be sent
 *	channel Kvaser channel to send the frame on (0, 1)
 */
int process_stp_output(char *in_path, char *out_path, struct kvalt_can_frame *frame);


/* 
 * Erase file.
 *
 * Arguments:
 *	path	Path to file which should be erased.
 */
void erase_file(char *path);



