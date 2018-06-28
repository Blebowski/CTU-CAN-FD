/*******************************************************************************
 * 
 * Kvaser / Altera Logger of CAN FD frames.
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

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>

#include <pty.h>
#include <utmp.h>

#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <canlib.h>

#include <time.h>
#include <stdlib.h>

#include "include/stp.h"
#include "include/kvaser.h"
#include "include/data_processor.h"

/* 
 * Number of arguments for Kvalt logger: 
 * 	frame_count
 * 	stp_session
 *	instance
 *	signal_Set
 *	trigger
 *	path to output file
 */
#define ARG_CNT 6


void print_help(void)
{
	fprintf(stdout, "**************************\n");
	fprintf(stdout, "*** Kvaalt logger help ***\n");
	fprintf(stdout, "**************************\n");
	fprintf(stdout, "Usage: \n");
	fprintf(stdout, "	kvaalt_logger <frame_count> <stp_sesssion> "
			"<instance> <signal_set> <trigger> <out_file>\n\n");
	fprintf(stdout, "Example:\n");
	fprintf(stdout, "	kvaalt_logger 10 /DOKUMENTY/Skola/CVUT-FEL/"
				"CAN_FD_PCIe/FPGA_design"
        		        "/Quartus/CAN_FD_SoC/output_files/debug.stp "
				"\"signal_set: 2018/06/14 15:19:43  #0\" "
				"\"trigger: 2018/06/14 15:19:43  #1\" \n");
	fprintf(stdout, "\n");
}


int kvaalt_start_log(int num_frames, char *session, char *instance, 
			char *signals, char *trigger, char *outfile)
{
	struct kvalt_can_frame tx_frame;
	struct kvalt_can_frame rx_frame;

	char *export = "build/tmp_export";
	char *tmp_session = "build/tmp_session.stp";

	/* Randomize for generator */
	srand(time(NULL));

	/* Initialize Kvaser on both channels */
	if (kvaser_init()){
		fprintf(stderr, "Error in Kvaser intialization\n");
		return 1;
	}

	/* Start Altera Quartus Signal TAP II in separate shell */
	if (stp_init(QUARTUS_STP_PATH)){
		fprintf(stderr, "Error in Signal TAP II initialization\n");
		return 1;
	}

	/* Erase output file */
	erase_file(outfile);

	stp_create_temp_session(session, tmp_session);
	stp_start_session(tmp_session);
	
	/* Execute logging itself */
	for (int i = 0; i < num_frames; i++){
		generate_can_frame(&tx_frame);
		stp_run_aquisition(i, instance, signals, trigger);
		kvaser_send_frame(&tx_frame, 0);
		kvaser_read_frame(&rx_frame, 1);
		stp_wait_aquisition_end();
		stp_export_log(i, instance, signals, trigger, export);
		process_stp_output(export, outfile, &tx_frame);
		fprintf(stdout, "Iteration nr: %d\n", i + 1);
	}
	
	stp_close_session();

	/* Close Altera Quartus Signal Tap II */
	stp_exit();

	return 0;
}


int main(int argc, char *argv[])
{
	int result;

	if (argc != ARG_CNT + 1){
		print_help();
		exit(EXIT_FAILURE);
	}

	if (!strcmp(argv[1], "--help") || !strcmp(argv[1], "-h")){
		print_help();
		exit(EXIT_FAILURE);
	}

	fprintf(stdout, "\n");
	fprintf(stdout, "Starting Kvaser / Altera CAN Logger...\n");
	fprintf(stdout, "Number of frames: %s\n", argv[1]);

	result = kvaalt_start_log(atoi(argv[1]), argv[2], argv[3], argv[4],
					argv[5], argv[6]);
	exit(result);
}






