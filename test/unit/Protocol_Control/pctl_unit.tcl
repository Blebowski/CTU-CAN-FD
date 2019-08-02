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
## 			Protocol control unit test handling script
################################################################################
global TCOMP

start_CAN_simulation "protocol_control_unit_test_wrapper"

################################################################################
# Adding the waves
################################################################################

#Add common waves for each test entity
add_test_status_waves
add_system_waves

#Add circuit specific signals
add wave -noupdate -divider -height 20 "CAN bus levels"	
add wave -label "Protocol Control 1 TX" $TCOMP/tx_data_nbs_1
add wave -label "Protocol Control 2 TX" $TCOMP/tx_data_nbs_2
add wave -label "Bus level" -color "Cyan" $TCOMP/bus_level
add wave -label "TX Trigger" $TCOMP/tx_trigger_1
add wave -label "RX Trigger" $TCOMP/rx_trigger_1

add wave -noupdate -divider -height 20 "Protocol Control 1"	

add wave -group "TX Frame metadata" $TCOMP/tran_dlc_1 \
									$TCOMP/tran_is_rtr_1 \
									$TCOMP/tran_ident_type_1 \
									$TCOMP/tran_frame_type_1 \
									$TCOMP/tran_brs_1 \
									$TCOMP/txt_data_word_1 \
									$TCOMP/txt_buf_ptr_1 \

add wave -label "Frame transmitted OK" $TCOMP/tran_valid_1
add wave -label "Acknowledge received" $TCOMP/ack_received_1
add wave -label "RX Data" $TCOMP/ack_received_1
add wave -label "PC State 1" $TCOMP/protocol_control_inst_1/protocol_control_fsm_inst/curr_state

add wave -label "Control counter enable" $TCOMP/protocol_control_inst_1/control_counter_inst/ctrl_ctr_ena
add wave -label "Control counter Preload" $TCOMP/protocol_control_inst_1/control_counter_inst/ctrl_ctr_pload
add wave -label "Control counter Preload value" $TCOMP/protocol_control_inst_1/control_counter_inst/ctrl_ctr_pload_val
add wave -label "Control counter Value" $TCOMP/protocol_control_inst_1/control_counter_inst/ctrl_ctr_q

add wave -label "Load DLC" $TCOMP/protocol_control_inst_1/tx_shift_reg_inst/tx_load_dlc
add wave -label "Load TX shift register" $TCOMP/protocol_control_inst_1/tx_shift_reg_inst/tx_sr_pload
add wave -label "TX shift register status" $TCOMP/protocol_control_inst_1/tx_shift_reg_inst/tx_shift_reg_inst/reg_stat
add wave -label "TX shift register preload value" $TCOMP/protocol_control_inst_1/tx_shift_reg_inst/tx_sr_pload_val
add wave -label "Shift reg CE" $TCOMP/protocol_control_inst_1/tx_shift_reg_inst/tx_sr_ce
add wave -label "TX Shift ENA" $TCOMP/protocol_control_inst_1/tx_shift_reg_inst/tx_shift_ena
add wave -label "TX Trigger" $TCOMP/protocol_control_inst_1/tx_shift_reg_inst/tx_trigger

add wave -label "TX Trigger" $TCOMP/protocol_control_inst_1/bit_error_arb

add wave -label "TXT Buffer memory" $TCOMP/tran_word_1


add wave -noupdate -divider -height 20 "Protocol Control 2"
add wave -label "Frame Received OK" $TCOMP/rec_valid_2
add wave -label "PC State 2" $TCOMP/protocol_control_inst_2/protocol_control_fsm_inst/curr_state

add wave -label "Rec DLC" $TCOMP/protocol_control_inst_2/rec_dlc
add wave -label "Rec Frame type" $TCOMP/protocol_control_inst_2/rec_frame_type
add wave -label "Rec Ident type" $TCOMP/protocol_control_inst_2/rec_ident_type
add wave -label "Rec Ident" $TCOMP/protocol_control_inst_2/rec_ident
add wave -label "Rec BRS" $TCOMP/protocol_control_inst_2/rec_brs

add wave -label "RX Shift reg" $TCOMP/protocol_control_inst_1/rx_shift_reg_inst/rx_shift_reg_q
add wave -label "RX Data in" $TCOMP/protocol_control_inst_1/rx_shift_reg_inst/rx_data


add wave -group "Protocol configuration" $TCOMP/drv_can_fd_ena \
										$TCOMP/drv_bus_mon_ena \
										$TCOMP/drv_retr_lim_ena \
										$TCOMP/drv_retr_th \
										$TCOMP/drv_self_test_ena \
										$TCOMP/drv_ack_forb \
										$TCOMP/drv_ena \
										$TCOMP/drv_fd_type

add wave -noupdate -divider -height 20 "Testbench internal signals"
add wave -label "SW CAN frame" $TCOMP/test_proc/sw_seq
add wave -label "Received bit sequence" $TCOMP/test_proc/rec_seq
add wave -label "Expected frame length" $TCOMP/test_proc/seq_length
add wave -label "TX/RX data equal" $TCOMP/test_proc/out_frm
add wave -label "SW/Recieved sequence equal" $TCOMP/test_proc/out_seq
add wave -label "TXT Mem contents" $TCOMP/txtb_mem
add wave -label "TXT Mem pointer" $TCOMP/txtb_mem_ptr	
add wave -label "RX Mem contents" $TCOMP/rxb_mem
add wave -label "RX Mem pointer" $TCOMP/rxb_mem_ptr						
									 
################################################################################
# Execute the simulation, gather results
################################################################################
run_simulation
get_test_results


