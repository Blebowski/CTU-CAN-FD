################################################################################
##
## CAN with Flexible Data-Rate IP Core 
##
## Copyright (C) 2015 Ondrej Ille <ondrej.ille@gmail.com>
##
## Permission is hereby granted, free of charge, to any person obtaining a copy 
## of this software and associated documentation files (the "Software"), to deal
## in the Software without restriction, including without limitation the rights
## to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
## copies of the Software, and to permit persons to whom the Software is 
## furnished to do so, subject to the following conditions:
##
## The above copyright notice and this permission notice shall be included in 
## all copies or substantial portions of the Software.
##
## THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
## IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
## FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
## AUTHORS OR COPYRIGHTHOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
## LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
## FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
## IN THE SOFTWARE.
##
## The CAN protocol is developed by Robert Bosch GmbH and protected by patents. 
## Anybody who wants to implement this IP core on silicon has to obtain a CAN 
## protocol license from Bosch.
##
################################################################################

################################################################################
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

