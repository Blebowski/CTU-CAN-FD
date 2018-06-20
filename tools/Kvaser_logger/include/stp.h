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


#define QUARTUS_STP_PATH "/opt/quartus/bin/quartus_stp"


/* 
 * Initializes Quartus Signal TAP II Logic analyzer.
 * Forks new process, connects input path and starts execution of command.
 * 
 * Arguments:
 *	stp_path	Path to Signal TAP II Analyzer binary.
 */
int stp_init(char *stp_path);


/* 
 * Terminates Quartus Signal TAP II Logic analyzer.
 */
void stp_exit(void);


/* 
 * Creates local copy of Signal TAP II session.
 * 
 * Arguments:
 *	in_session	Path to Signal TAP II Analyzer session (.stp file).
 *		  	This session will be duplicated.
 *	out_session	Path to duplicated session.
 */
int stp_create_temp_session(char *in_session, char *out_session);


/* 
 * Sends command to Signal TAP II Logic analyzer to initialize sesssion.
 * 
 * Arguments:
 *	session		Path to Signal TAP II Analyzer session (.stp file).
 */
int stp_start_session(char *session);


/* 
 * Sends command to Signal TAP II Logic analyzer to start aquisition.
 * 
 * Arguments:
 *	instance	Name of the instance within session
 *	signals		Name of signal set
 *	trigger		Name of trigger
 */
int stp_run_aquisition(int log, char *instance, char *signals, char *trigger);


/* 
 * Waits 500 ms. Signal TAP II Aquisition should be over by then.
 * 
 * Arguments:
 *	instance	Name of the instance within session
 *	signals		Name of signal set
 *	trigger		Name of trigger
 */
void stp_wait_aquisition_end(void);



/* 
 * Export logged data from Signal TAP II logic analyzer aquisition.
 * Exported data are not stored in the session anymore!
 * 
 * Arguments:
 *	instance	Name of the instance within session
 *	signals		Name of signal set
 *	trigger		Name of trigger
 *	dest		Destination where data should be stored.s
 */
int stp_export_log(int log, char *instance, char *signals, char *trigger, char *dest);


/* 
 * Closes Signal TAP II Logic analyzer session. Session is automatically
 * saved to .stp file. 
 *
 */
int stp_close_session(void);






















