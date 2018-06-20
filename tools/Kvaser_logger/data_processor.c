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

#include "include/kvaser.h"
#include <regex.h> 


int parse_stp_output(FILE *stp, FILE *out)
{
	char line_buf[30];
	regex_t regex;
	int reg_res;
	int first_sample = 0;
	int eq_pos = 0;

	char prev_val = '0';
	int eq_count = 0;
	
	/* Parse sampled lines */
	reg_res = regcomp(&regex, "[ ]*[0-9]*.[0-9]> [0-9] = [0-9]*", 0);
	if (reg_res) {
		fprintf(stderr, "Unable to compile Regex\n");
		return EXIT_FAILURE;
	}

	fprintf(out, "Bit sequence: ");

	while (1) {
		fgets(line_buf, 30, stp);
		if (feof(stp))
			break;

		/* Evaluate Regex over the line */
		reg_res = regexec(&regex, line_buf, 0, NULL, 0);

		/* Process only matching Elements */
		if (reg_res == REG_NOMATCH)
			continue;

		/* Extract position of '=' in first line */
		if (first_sample == 0) {
			eq_pos = strcspn(line_buf, "=");
			first_sample = 1;
			continue;
		}

		/* Count and write when signal value changes */
		if (prev_val != line_buf[eq_pos + 2]){
			fprintf(out, "%d %c ", eq_count, prev_val);			
			eq_count = 0;
		} else {
			eq_count++;
		}
		prev_val = line_buf[eq_pos + 2];
	}
	fprintf(out, "%d %c \n", eq_count, prev_val);

	regfree(&regex);

	return EXIT_SUCCESS;
}


int store_frame_info(struct kvalt_can_frame *frame, FILE *out)
{
	if (frame->fr_type) {
		fprintf(out, "CAN FD  ");
	} else {
		fprintf(out, "CAN 2.0 ");
	}

	if (frame->id_type) {
		fprintf(out, "EXTENDED ");
	} else {
		fprintf(out, "BASE     ");
	}

	if (frame->rtr) {
		fprintf(out, "RTR ");
	} else {
		fprintf(out, "    ");
	}

	if (frame->brs) {
		fprintf(out, "BRS ");
	} else {
		fprintf(out, "    ");
	}

	fprintf(out, "Data length: %2d ", frame->data_length);
	fprintf(out, "ID: %9ld ", frame->id);

	fprintf(out, "Data: ");
	for (int i = 0; i < 64; i++)
		fprintf(out, "%02x ", frame->data[i]);

	return EXIT_SUCCESS;
}


int process_stp_output(char *in_path, char *out_path, struct kvalt_can_frame *frame)
{
	FILE *stp;
	FILE *out;
	char path_with_ext[200];

	memset(path_with_ext, '\0', sizeof(path_with_ext));
	strcpy(path_with_ext, "./");
	strcat(path_with_ext, in_path);
	strcat(path_with_ext, ".tbl");

	stp = fopen(path_with_ext, "r");

	if (stp == NULL){
		fprintf(stderr, "Could not open Signal TAP II Exported file!\n");
		fprintf(stderr, "Path: %s\n", path_with_ext);
		return EXIT_FAILURE;
	}

	out = fopen(out_path, "a");
	if (stp == NULL){
		fprintf(stderr, "Could not open output file!\n");
		fprintf(stderr, "Path: %s\n", out_path);
		return EXIT_FAILURE;
	}
	store_frame_info(frame, out);
	parse_stp_output(stp, out);

	fclose(stp);
	fclose(out);

	return EXIT_SUCCESS;
}


void erase_file(char *path)
{
	char cmd[100];
	memset(&cmd, '\0', sizeof(cmd));
	sprintf(cmd, "rm %s", path);
	system(cmd);
}











