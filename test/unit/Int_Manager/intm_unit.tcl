################################################################################
##
## CAN with Flexible Data-Rate IP Core 
##
## Copyright (C) 2015 Ondrej Ille <ondrej.ille@gmail.com>
##
## This program is free software; you can redistribute it and/or
## modify it under the terms of the GNU General Public License
## as published by the Free Software Foundation; either version 2
## of the License, or (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## The CAN protocol is developed by Robert Bosch GmbH and     
## protected by patents. Anybody who wants to implement this    
## IP core on silicon has to obtain a CAN protocol license
## from Bosch.
##  
## Description:
## 			Interrupt manager unit test handling script
################################################################################
global TCOMP

start_CAN_simulation "int_man_test_wrapper"

################################################################################
# Adding the waves
################################################################################

#Add common waves for each test entity
add_test_status_waves
add_system_waves

#Add circuit specific signals
add wave -noupdate -divider -height 20 "DUT inputs (generated)"
add wave -group "Interrupt sources" $TCOMP/error_valid \
									 $TCOMP/error_passive_changed \
									 $TCOMP/error_warning_limit \
									 $TCOMP/arbitration_lost \
									 $TCOMP/wake_up_valid \
									 $TCOMP/tx_finished \
									 $TCOMP/br_shifted \
									 $TCOMP/rx_message_disc \
									 $TCOMP/rec_message_valid \
									 $TCOMP/rx_full \
									 $TCOMP/loger_finished
#
add wave -group "Interrupt configuration" $TCOMP/drv_bus_err_int_ena \
										$TCOMP/drv_arb_lst_int_ena \
										$TCOMP/drv_err_pas_int_ena \
										$TCOMP/drv_wake_int_ena \
										$TCOMP/drv_dov_int_ena \
										$TCOMP/drv_err_war_int_ena \
										$TCOMP/drv_tx_int_ena \
										$TCOMP/drv_rx_int_ena \
										$TCOMP/drv_log_fin_int_ena \
										$TCOMP/drv_rx_full_int_ena \
										$TCOMP/drv_brs_int_ena

add wave $TCOMP/drv_int_vect_erase	

add wave -noupdate -divider -height 20 "DUT outputs"
add wave $TCOMP/int_out
add wave $TCOMP/int_vector

add wave -noupdate -divider -height 20 "Testbench internal signals"
add wave $TCOMP/int_ctr
add wave $TCOMP/int_test_ctr
add wave $TCOMP/int_test_mask
add wave $TCOMP/int_test_vector	 

add wave -noupdate -divider -height 20 "Internal DUT signals"								
add wave $TCOMP/int_man_comp/int_mask
add wave $TCOMP/int_man_comp/interrupt_active
add wave $TCOMP/int_man_comp/interrupt_counter
									 
################################################################################
# Execute the simulation, gather results
################################################################################
run_simulation
get_test_results

