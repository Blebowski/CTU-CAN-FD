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
## 			TX Arbitrator unit test handling script
################################################################################
global TCOMP

start_CAN_simulation "tx_arb_unit_test_wrapper"

################################################################################
# Adding the waves
################################################################################

#Add common waves for each test entity
add_test_status_waves
add_system_waves
add wave -label "Timestamp" -unsigned $TCOMP/timestamp

#Add circuit specific signals
add wave -noupdate -divider -height 20 "DUT TXT Buffers interface"
add wave -label "Buffer inputs" -unsigned $TCOMP/txt_buf_in
add wave -label "Buffers ready" -unsigned $TCOMP/txt_buf_ready
add wave -label "Buffers pointer" -unsigned $TCOMP/txtb_ptr
add wave -label "Stored buffer index" -unsigned $TCOMP/txt_hw_cmd_buf_index
add wave -label "Priorities" $TCOMP/txArbitrator_comp/txt_buf_prio

add wave -noupdate -divider -height 20 "DUT CAN Core interface"
add wave -label "Data output" -hexadecimal $TCOMP/tran_data_word_out
add wave -label "DLC" $TCOMP/tran_dlc_out
add wave -label "RTR" $TCOMP/tran_is_rtr
add wave -label "Identifier type" $TCOMP/tran_ident_type_out
add wave -label "Frame format" $TCOMP/tran_frame_type_out
add wave -label "BRS" $TCOMP/tran_brs_out
add wave -label "Output frame is valid" $TCOMP/tran_frame_valid_out
add wave -label "Hardware commands" $TCOMP/txt_hw_cmd
add wave -label "Pointer from CAN Core" $TCOMP/txtb_core_pointer

add wave -noupdate -divider -height 20 "Internal testbench signals"
add wave -label "Shadow memories" $TCOMP/shadow_mem
add wave -label "Model locked" $TCOMP/mod_locked
add wave -label "DLC output" $TCOMP/mod_dlc_out
add wave -label "RTR" $TCOMP/mod_is_rtr
add wave -label "Identifier type" $TCOMP/mod_ident_type_out
add wave -label "Frame type" $TCOMP/mod_frame_type_out
add wave -label "Frame valid" $TCOMP/mod_frame_valid_out
add wave -label "Stored index" $TCOMP/mod_buf_index
add wave -label "Combinational index" $TCOMP/high_prio_buf_index

add wave -noupdate -divider -height 20 "Internal DUT signals"
add wave -label "Metadata pointer" $TCOMP/txArbitrator_comp/txtb_pointer_meta
add wave -label "FSM" $TCOMP/txArbitrator_comp/tx_arb_fsm
add wave -label "Last txt buffer index" $TCOMP/txArbitrator_comp/last_txtb_index
add wave -label "Buffer available" $TCOMP/txArbitrator_comp/select_buf_avail
add wave -label "Selected buf index" $TCOMP/txArbitrator_comp/select_buf_index
add wave -label "Read timestamp" $TCOMP/txArbitrator_comp/txtb_timestamp
add wave -label "Timestamp valid" $TCOMP/txArbitrator_comp/timestamp_valid

									 
################################################################################
# Execute the simulation, gather results
################################################################################
run_simulation
get_test_results


