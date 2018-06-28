/*******************************************************************************
 * 
 * Kvaser / Altera Logger of CAN FD frames.
 * 
 * Module: Kvaser access functions
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

/* Kvaser global variables */
static canHandle hnd_chn_1;
static canHandle hnd_chn_2;

static int kvalt_dlc[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};


static void check(char* id, canStatus stat)
{
	if (stat != canOK) {
		char buf[50];
		buf[0] = '\0';
		canGetErrorText(stat, buf, sizeof(buf));
		printf("%s: failed, stat=%d (%s)\n", id, (int)stat, buf);
	}
}


int kvaser_init(void)
{
	canStatus stat;

	/* Allow signals to interrupt syscalls */
	siginterrupt(SIGINT, 1);

	canInitializeLibrary();

	/* Open channel, set parameters and go on bus */
	hnd_chn_1 = canOpenChannel(0, canOPEN_CAN_FD | canOPEN_EXCLUSIVE | 
					canOPEN_REQUIRE_EXTENDED);
	hnd_chn_2 = canOpenChannel(1, canOPEN_CAN_FD | canOPEN_EXCLUSIVE | 
					canOPEN_REQUIRE_EXTENDED);
	if (hnd_chn_1 < 0) {
		fprintf(stdout, "canOpenChannel 0");
		return -1;
	}

	if (hnd_chn_2 < 0) {
		fprintf(stdout, "canOpenChannel 1");
		return -1;
	}

	/* Channel 1 */

	/* Set Nominal Bit-rate to 500 K */ 
	stat = canSetBusParams(hnd_chn_1, canFD_BITRATE_500K_80P, 0, 0, 0, 0, 0);
	check("canSetBusParams", stat);
	if (stat != canOK) {
		goto ErrorExit;
	}

	/* Set Data Bit-rate to 2 M */
	stat = canSetBusParamsFd(hnd_chn_1, canFD_BITRATE_2M_80P, 0, 0, 0);
	check("canSetBusParamsFd", stat);
	if (stat != canOK) {
		goto ErrorExit;
	}

	stat = canBusOn(hnd_chn_1);
	check("canBusOn", stat);
	if (stat != canOK) {
		goto ErrorExit;
	}
	
	/* Channel 2 */

	/* Set Nominal Bit-rate to 500 K */ 
	stat = canSetBusParams(hnd_chn_2, canFD_BITRATE_500K_80P, 0, 0, 0, 0, 0);
	check("canSetBusParams", stat);
	if (stat != canOK) {
		goto ErrorExit;
	}


	/* Set Data Bit-rate to 2 M */
	stat = canSetBusParamsFd(hnd_chn_2, canFD_BITRATE_2M_80P, 0, 0, 0);
	check("canSetBusParamsFd", stat);
	if (stat != canOK) {
		goto ErrorExit;
	}

	stat = canBusOn(hnd_chn_2);
	check("canBusOn", stat);
	if (stat != canOK) {
		goto ErrorExit;
	}

	return EXIT_SUCCESS;

ErrorExit:
  	stat = canBusOff(hnd_chn_1);
  	stat = canBusOff(hnd_chn_2);
  	check("canBusOff", stat);
  	stat = canClose(hnd_chn_1);
  	stat = canClose(hnd_chn_2);
  	check("canClose", stat);
  	stat = canUnloadLibrary();
  	check("canUnloadLibrary", stat);
	return 1;
}


void print_can_frame(struct kvalt_can_frame *frame)
{
	fprintf(stdout, "CAN Frame: \n");
	fprintf(stdout, "	Data length: %d\n", frame->data_length);
	fprintf(stdout, "	RTR: 0x%x\n", frame->rtr);
	fprintf(stdout, "	BRS: 0x%x\n", frame->brs);
	fprintf(stdout, "	Frame Type: 0x%x\n", frame->fr_type);
	fprintf(stdout, "	ID Type: 0x%x\n", frame->id_type);
	fprintf(stdout, "	ID: %lu \n", frame->id);
}


int generate_can_frame(struct kvalt_can_frame *frame)
{
	if (frame == NULL){
		fprintf(stderr, "Invalid CAN frame pointer\n");
		return EXIT_FAILURE;
	}

	int rval;

	rval = rand();
	frame->id_type = (uint8_t)(rval % 2);

	rval = rand();
	frame->fr_type = (uint8_t)(rval % 2);

	rval = rand();	
	if (frame->fr_type){
		frame->brs = (uint8_t)(rval % 2);
		frame->rtr = 0;
	} else {
		frame->rtr = (uint8_t)(rval % 2);
		frame->brs = 0;
	}

	rval = rand();
	if (frame->fr_type) {
		if (frame->brs)
			frame->data_length = kvalt_dlc[rval % 16];
		else
			frame->data_length = kvalt_dlc[rval % 11];
	} else {
		if (frame->rtr) {
			frame->data_length = 0;
		} else {
			frame->data_length = kvalt_dlc[rval % 8];
		}
	}

	rval = rand();
	if (frame->id_type) {
		frame->id = (rval % 536870912);
	} else {
		frame->id = (rval % 2048);
	}

	memset(frame->data, 0, sizeof(frame->data));

	for (int i = 0; i < frame->data_length; i++){
		rval = rand();
		frame->data[i] = (char)(rval % 0xFF);
	}

	return EXIT_SUCCESS;
}


int kvaser_send_frame(struct kvalt_can_frame *frame, int channel)
{
	unsigned int flag = 0U;
	canStatus stat;
	canHandle tmp_handle;

	if (frame->rtr)
		flag |= canMSG_RTR;

	if (frame->id_type)
		flag |= canMSG_EXT;
	else
		flag |= canMSG_STD;

	if (frame->fr_type)
		flag |= canFDMSG_FDF;
	
	if (frame->brs)
		flag |= canFDMSG_BRS;

	if (channel == 0) {
		tmp_handle = hnd_chn_1;
	} else if (channel == 1) {
		tmp_handle = hnd_chn_2;
	} else {
		fprintf(stderr, "Invalid Kvaser Channel!\n");
		return EXIT_FAILURE;
	}

	stat = canWrite(tmp_handle, frame->id, &(frame->data),
			frame->data_length, flag);
	check("canWrite", stat);
	if (stat != canOK) {
		goto ErrorExit;
	}

	stat = canWriteSync(tmp_handle, 1000);
	check("canWriteSync", stat);
	if (stat != canOK) {
		goto ErrorExit;
	}
	return EXIT_SUCCESS;

ErrorExit:
	fprintf(stderr, "Failed to send CAN Frame\n");
	return EXIT_FAILURE;
}


int kvaser_read_frame(struct kvalt_can_frame *frame, int channel)
{
	canStatus stat;
	canHandle tmp_handle;
	unsigned int flag;
	unsigned long time;

	if (channel == 0) {
		tmp_handle = hnd_chn_1;
	} else if (channel == 1) {
		tmp_handle = hnd_chn_2;
	} else {
		fprintf(stderr, "Invalid Kvaser Channel!\n");
		return EXIT_FAILURE;
	}
	stat = canReadWait(tmp_handle, &(frame->id), &(frame->data), &(frame->data_length), &flag, &time, 100);

	if (stat == canOK)
		return EXIT_SUCCESS;

	fprintf(stderr, "Error in reading CAN Frame");
	return EXIT_FAILURE;
}
