################################################################################
## 
## CAN with Flexible Data-Rate IP Core 
## 
## Copyright (C) 2017 Ondrej Ille <ondrej.ille@gmail.com>
## 
## Project advisor: Jiri Novak <jnovak@fel.cvut.cz>
## Department of Measurement         (http://meas.fel.cvut.cz/)
## Faculty of Electrical Engineering (http://www.fel.cvut.cz)
## Czech Technical University        (http://www.cvut.cz/)
## 
## Permission is hereby granted, free of charge, to any person obtaining a copy 
## of this VHDL component and associated documentation files (the "Component"), 
## to deal in the Component without restriction, including without limitation 
## the rights to use, copy, modify, merge, publish, distribute, sublicense, 
## and/or sell copies of the Component, and to permit persons to whom the 
## Component is furnished to do so, subject to the following conditions:
## 
## The above copyright notice and this permission notice shall be included in 
## all copies or substantial portions of the Component.
## 
## THE COMPONENT IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
## IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
## FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
## AUTHORS OR COPYRIGHTHOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
## LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
## FROM, OUT OF OR IN CONNECTION WITH THE COMPONENT OR THE USE OR OTHER DEALINGS 
## IN THE COMPONENT.
## 
## The CAN protocol is developed by Robert Bosch GmbH and protected by patents. 
## Anybody who wants to implement this IP core on silicon has to obtain a CAN 
## protocol license from Bosch.
## 
################################################################################

################################################################################
## Description:
## 			Bit stuffing and destuffing unit test handling script
################################################################################
global TCOMP

start_CAN_simulation "event_logger_unit_test_wrapper"

################################################################################
# Adding the waves
################################################################################

#Add common waves for each test entity
add_test_status_waves
add_system_waves

#Add circuit specific signals
add wave -noupdate -divider -height 20 "DUT inputs (generated)"							 
add wave -label "Status bus (events occuring)" $TCOMP/stat_bus	
add wave -label "Synchronization edge" $TCOMP/sync_edge	
add wave -label "Data overrun" $TCOMP/data_overrun
add wave -label "Timestamp" $TCOMP/timestamp
add wave -label "Protocol state" $TCOMP/PC_state

add wave -noupdate -divider -height 20 "DUT outputs"
add wave -label "Logger finished" $TCOMP/loger_finished
add wave -label "Logger data output" -hexadecimal $TCOMP/loger_act_data
add wave -label "Write pointer" -unsigned $TCOMP/log_write_pointer
add wave -label "Read pointer" -unsigned $TCOMP/log_read_pointer
add wave -label "Logger size" -unsigned $TCOMP/log_size
add wave -label "State" $TCOMP/log_state_out

add wave -noupdate -divider -height 20 "Internal DUT signals"								
add wave -label "Memory content" $TCOMP/can_logger_comp/memory
add wave -label "Memory valid word" $TCOMP/can_logger_comp/memory_valid
add wave $TCOMP/can_logger_comp/event_inputs
add wave $TCOMP/can_logger_comp/event_edge
add wave $TCOMP/can_logger_comp/event_captured
add wave $TCOMP/can_logger_comp/harvest_pointer
add wave -group "Trigger settings" \
			$TCOMP/can_logger_comp/drv_trig_sof \
			$TCOMP/can_logger_comp/drv_trig_arb_lost \
			$TCOMP/can_logger_comp/drv_trig_rec_valid \
			$TCOMP/can_logger_comp/drv_trig_tran_valid \
			$TCOMP/can_logger_comp/drv_trig_ovl \
			$TCOMP/can_logger_comp/drv_trig_error \
			$TCOMP/can_logger_comp/drv_trig_brs \
			$TCOMP/can_logger_comp/drv_trig_user_write \
			$TCOMP/can_logger_comp/drv_trig_arb_start \
			$TCOMP/can_logger_comp/drv_trig_contr_start \
			$TCOMP/can_logger_comp/drv_trig_data_start \
			$TCOMP/can_logger_comp/drv_trig_crc_start \
			$TCOMP/can_logger_comp/drv_trig_ack_rec \
			$TCOMP/can_logger_comp/drv_trig_ack_n_rec \
			$TCOMP/can_logger_comp/drv_trig_ewl_reached \
			$TCOMP/can_logger_comp/drv_trig_erp_changed \
			$TCOMP/can_logger_comp/drv_trig_tran_start \
			$TCOMP/can_logger_comp/drv_trig_rec_start
			
add wave -group "Capture settings" \
			$TCOMP/can_logger_comp/drv_cap_sof \
			$TCOMP/can_logger_comp/drv_cap_arb_lost \
			$TCOMP/can_logger_comp/drv_cap_rec_valid \
			$TCOMP/can_logger_comp/drv_cap_tran_valid \
			$TCOMP/can_logger_comp/drv_cap_ovl \
			$TCOMP/can_logger_comp/drv_cap_error \
			$TCOMP/can_logger_comp/drv_cap_brs \
			$TCOMP/can_logger_comp/drv_cap_arb_start \
			$TCOMP/can_logger_comp/drv_cap_contr_start \
			$TCOMP/can_logger_comp/drv_cap_data_start \
			$TCOMP/can_logger_comp/drv_cap_crc_start \
			$TCOMP/can_logger_comp/drv_cap_ack_rec \
			$TCOMP/can_logger_comp/drv_cap_ack_n_rec \
			$TCOMP/can_logger_comp/drv_cap_ewl_reached \
			$TCOMP/can_logger_comp/drv_cap_erp_changed \
			$TCOMP/can_logger_comp/drv_cap_tran_start \
			$TCOMP/can_logger_comp/drv_cap_rec_start	

add wave -label "Event inputs" $TCOMP/evnt_inputs	
add wave -label "Event inputs edge" $TCOMP/evnt_inputs_edge	

add wave -noupdate -divider -height 20 "SW model"							 
add wave -label "Logger model memory" $TCOMP/log_mod_mem	
add wave -label "Logger pointer" $TCOMP/ev_logger_model_proc/log_ptr
add wave -label "Logger input edge" $TCOMP/evnt_inputs_edge
add wave -label "Logger inputs reg" $TCOMP/evnt_inputs_reg
add wave -label "Trigger settings" $TCOMP/drv_trig
add wave -label "Trigger inputs" $TCOMP/trig_inputs
									 
################################################################################
# Execute the simulation, gather results
################################################################################
run_simulation
get_test_results



