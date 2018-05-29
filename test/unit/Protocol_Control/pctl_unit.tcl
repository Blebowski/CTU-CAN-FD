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
add wave -label "Protocol Control 1 TX" $TCOMP/data_tx_1
add wave -label "Protocol Control 2 TX" $TCOMP/data_tx_2
add wave -label "Bus level" -color "Cyan" $TCOMP/bus_level
add wave -label "TX Trigger" $TCOMP/tx_trig
add wave -label "RX Trigger" $TCOMP/rx_trig

add wave -noupdate -divider -height 20 "Protocol Control 1"	

add wave -group "TX Frame metadata" $TCOMP/tran_dlc_1 \
									$TCOMP/tran_is_rtr_1 \
									$TCOMP/tran_ident_type_1 \
									$TCOMP/tran_frame_type_1 \
									$TCOMP/tran_brs_1 \
									$TCOMP/txt_data_word_1 \
									$TCOMP/txt_buf_ptr_1 \

add wave -label "TX Frame Identifier BASE" $TCOMP/protocolControl_1_Comp/tran_ident_base_sr
add wave -label "TX Frame Identifier EXT" $TCOMP/protocolControl_1_Comp/tran_ident_ext_sr



add wave -label "Frame transmitted OK" $TCOMP/tran_valid_1
add wave -label "Acknowledge received" $TCOMP/ack_recieved_out_1
add wave -label "PC State" $TCOMP/pc_state_out_1
add wave -label "Operational State" $TCOMP/op_state_1
add wave -label "Stuff length" -unsigned $TCOMP/stuff_length_1


add wave -noupdate -divider -height 20 "Protocol Control 2"
add wave -label "Frame Received OK" $TCOMP/rec_valid_2
add wave -label "PC State" $TCOMP/pc_state_out_2
add wave -label "Operational State" $TCOMP/op_state_2
add wave -label "De-Stuff length" -unsigned $TCOMP/destuff_length_2

add wave -group "Protocol configuration" $TCOMP/drv_rtr_pref \
										$TCOMP/drv_can_fd_ena \
										$TCOMP/drv_bus_mon_ena \
										$TCOMP/drv_retr_lim_ena \
										$TCOMP/drv_retr_th \
										$TCOMP/drv_self_test_ena \
										$TCOMP/drv_abort_tran \
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


