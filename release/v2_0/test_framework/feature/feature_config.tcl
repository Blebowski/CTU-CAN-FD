################################################################################
## Author:      Ondrej Ille , Czech Technical University, FEL
## Project:     CAN FD IP Core Project
##
## 			Feature test configuration file
################################################################################

global FEATURE_FIFO

#quietly set FEATURE_FIFO [list [list forbid_fd 50] \
					   [list abort_transmittion 50] \
					   [list rtr_pref 25] \
					   [list tx_arb_time_tran 100] \
					   [list traf_measure 80] \
					   [list spec_mode 50] \
					   [list soft_reset 5] \
					   [list trv_delay 50] \
					   [list invalid_configs 30] \
					   [list retr_limit 60] \
					   [list overload 20] \
					   [list traf_measure 25]

quietly set FEATURE_FIFO [list [list trv_delay 10]]
				 
#[list interrupt 10] \
					   