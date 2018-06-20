/*******************************************************************************
 * 
 * Kvaser / Altera Logger of CAN FD frames.
 * 
 * Module: Signal TAP II Logic Analyzer access functions.
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


/* Signal tap variables */
static char *stp_args[] = {"quartus_stp", "--shell", NULL};
static pid_t stp_pid;
int pipes[2];



int prepare_child_pipes(void)
{
	if (close(pipes[STDOUT_FILENO]) == -1) {
    		perror("CHILD: close write");
    		return EXIT_FAILURE;
 	}
	if (dup2(pipes[STDIN_FILENO], STDIN_FILENO) == -1) {
   		perror("CHILD: dup2 read STDIN_FILENO");
    		return EXIT_FAILURE;
	}
	if (close(pipes[STDIN_FILENO]) == -1) {
    		perror("CHILD: close read");
    		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}



int stp_init(char *stp_path)
{
	if (stp_path == NULL){
		fprintf(stderr, "Invalid path pointer!\n");	
		return EXIT_FAILURE;
	};

	if (pipe(pipes) == -1){
		fprintf(stderr, "Pipe error!\n");
		return EXIT_FAILURE;
	};

	stp_pid = fork();

	if (!stp_pid) {

		if (prepare_child_pipes())
			return EXIT_FAILURE;

		execv(stp_path, stp_args);
		fprintf(stderr, "Unable to start Signal TAP II!\n");
		exit(127);

	} else if (stp_pid > 0) {
		fprintf(stdout, "Started Signal TAP II. PID: %d\n", stp_pid);
	} else {
		fprintf(stdout, "Unable to start Signal TAP II! %d\n", stp_pid);
	}

	return EXIT_SUCCESS;
}


void stp_exit(void)
{
	fprintf(stdout, "Killing Signal Tap II analyzer...\n");
	fprintf(stdout, "\n");
	kill(stp_pid, SIGTERM);
	waitpid(stp_pid, 0, 0);
}


int stp_create_temp_session(char *in_session, char *out_session)
{
	char cmd[100];

	memset(&cmd, '\0', sizeof(cmd));
	sprintf(cmd, "cp %s %s", in_session, out_session);
	system(cmd);

	return EXIT_SUCCESS;
}


int stp_start_session(char *session)
{

	char cmd[300];
	int nbytes;

	memset(&cmd, '\0', sizeof(cmd));
	sprintf(cmd, "open_session -name %s\n", session);

	/* Sending command to signal tap to start session */
	fprintf(stdout, "Sending command to start Signal tap II session...\n");
	nbytes = write(pipes[STDOUT_FILENO], cmd, strlen(cmd));
	if (nbytes != strlen(cmd)){
		fprintf(stderr,"Session not started properly!\n");
		return EXIT_FAILURE;
	}
	usleep(1000000);
	fprintf(stdout, "Signal tap II session started...\n");
	
	return EXIT_SUCCESS;
}


int stp_run_aquisition(int log, char *instance, char *signals, char *trigger)
{
	char cmd[400];
	int nbytes;

	memset(&cmd, '\0', sizeof(cmd));
	sprintf(cmd, "run -instance \"%s\" -signal_set \"%s\" "
			"-trigger \"%s\" -data_log log_%d\n", instance, signals, trigger, log);

	nbytes = write(pipes[STDOUT_FILENO], cmd, strlen(cmd));
	if (nbytes != strlen(cmd)){
		fprintf(stderr,"Aquisition not started!\n");
		return EXIT_FAILURE;
	}
	usleep(200000);
	
	return EXIT_SUCCESS;
}


void stp_wait_aquisition_end(void)
{
	usleep(200000);
}


int stp_export_log(int log, char *instance, char *signals, char *trigger, char *dest)
{
	char cmd[200];
	int nbytes;

	memset(&cmd, '\0', sizeof(cmd));
	sprintf(cmd, "export_data_log -instance %s -signal_set \"%s\" "
		     "-trigger \"%s\" -data_log log_%d -filename %s -format tlb\n",
			instance, signals, trigger, log, dest);

	nbytes = write(pipes[STDOUT_FILENO], cmd, strlen(cmd));
	if (nbytes != strlen(cmd)){
		fprintf(stderr,"Data not exported properly!\n");
		return EXIT_FAILURE;
	}
	usleep(200000);
	
	return EXIT_SUCCESS;
}


int stp_close_session(void)
{
	char *cmd = "close_session\n";
	int nbytes;

	fprintf(stdout, "Closing session from Signal tap II...\n");
	nbytes = write(pipes[STDOUT_FILENO], cmd, strlen(cmd));
	if (nbytes != strlen(cmd)){
		fprintf(stderr,"Session not closed!\n");
		return EXIT_FAILURE;
	}
	usleep(1000000);

	return EXIT_SUCCESS;
}
