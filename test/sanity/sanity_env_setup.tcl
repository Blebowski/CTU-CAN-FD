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
## 			Feature environment waveform setup script
################################################################################
global TCOMP
global INST1
global CORE
global SILENT_SANITY

start_CAN_simulation "sanity_test_wrapper"
quietly set INST1 "can_inst_1"
quietly set CORE  "core_top_comp"

################################################################################
# Adding the waves
################################################################################

#Add common waves for each test entity
add_test_status_waves

# Sanity test was moved to the wrapper in order to create inputs in VHDL
# for AHDL simulation. Component path must be added to have correct path
# to internal signals
set WRCOMP $TCOMP
append WRCOMP "/i_st"

add wave $WRCOMP/error_ctr
add wave $WRCOMP/loop_ctr

#Add circuit specific signals
if { $SILENT_SANITY == "false" } {
	add wave -group "Test configuration" \
		-label "Configuration name" $WRCOMP/test_desc \
		-label "Topology" $WRCOMP/topology \
		-label "Bus Matrix" $WRCOMP/bus_matrix \
		-label "Clock tolerances (error)" $WRCOMP/epsilon_v \
		-label "Transciever delay" $WRCOMP/trv_del_v \
		-label "Noise width - mean (ns)" $WRCOMP/nw_mean \
		-label "Noise width - variance (ns)" $WRCOMP/nw_var \
		-label "Noise gap - mean (ns)" $WRCOMP/ng_mean \
		-label "Noise gap - variance (ns)" $WRCOMP/ng_var \
		-label "Bus timing" $WRCOMP/timing_config
} else {
	add wave -group "Test configuration" \
		-label "Configuration name" $WRCOMP/test_desc
}

add wave -label "Global Frame counter" $WRCOMP/overal_frame_counter
if { $SILENT_SANITY == "false" } {
	add wave -label "Node Frame counters" $WRCOMP/frame_counters
}

if { $SILENT_SANITY == "false" } {
	add wave -label "Timestamps" $WRCOMP/timestamp_v
}

add wave -label "Noise values" $WRCOMP/noise_reg
add wave -label "Noise force" -event $WRCOMP/noise_force

if { $SILENT_SANITY == "false" } {
	add wave -group "Test control and status signals" \
		-label "TX memories full" $WRCOMP/tx_full \
		-label "Wait" $WRCOMP/do_wait \
		-label "Config" $WRCOMP/do_config \
		-label "DO traffic" $WRCOMP/do_traffic \
		-label "Erase memories" $WRCOMP/do_erase_mems \
		-label "Generate noise" $WRCOMP/do_noise
}

if { $SILENT_SANITY == "false" } {
	add wave -label "Memory buses" $WRCOMP/mb_arr
}

add wave -label "CAN Tx" -expand $WRCOMP/can_tx_v
add wave -label "CAN Rx" -expand $WRCOMP/can_rx_v

add wave -group "Protocol states" \
	-label "Node 1" $WRCOMP/comp_gen__1/node_1_comp/core_top_comp/PC_State \
	-label "Node 2" $WRCOMP/comp_gen__2/node_1_comp/core_top_comp/PC_State \
	-label "Node 3" $WRCOMP/comp_gen__3/node_1_comp/core_top_comp/PC_State \
	-label "Node 4" $WRCOMP/comp_gen__4/node_1_comp/core_top_comp/PC_State

if { $SILENT_SANITY == "false" } {
add wave -group "Bit time state" \
	-label "Node 1" $WRCOMP/comp_gen__1/node_1_comp/bt_FSM_out \
	-label "Node 2" $WRCOMP/comp_gen__2/node_1_comp/bt_FSM_out \
	-label "Node 3" $WRCOMP/comp_gen__3/node_1_comp/bt_FSM_out \
	-label "Node 4" $WRCOMP/comp_gen__4/node_1_comp/bt_FSM_out
	
add wave -group "Trancieve triggers" \
	-label "Node 1" $WRCOMP/comp_gen__1/node_1_comp/core_top_comp/tran_trig \
	-label "Node 2" $WRCOMP/comp_gen__2/node_1_comp/core_top_comp/tran_trig \
	-label "Node 3" $WRCOMP/comp_gen__3/node_1_comp/core_top_comp/tran_trig \
	-label "Node 4" $WRCOMP/comp_gen__4/node_1_comp/core_top_comp/tran_trig
	
add wave -group "Recieve triggers" \
	-label "Node 1" $WRCOMP/comp_gen__1/node_1_comp/core_top_comp/rec_trig \
	-label "Node 2" $WRCOMP/comp_gen__2/node_1_comp/core_top_comp/rec_trig \
	-label "Node 3" $WRCOMP/comp_gen__3/node_1_comp/core_top_comp/rec_trig \
	-label "Node 4" $WRCOMP/comp_gen__4/node_1_comp/core_top_comp/rec_trig
	
add wave -group "Hard sync edge" \
	-label "Node 1" $WRCOMP/comp_gen__1/node_1_comp/hard_sync_edge_valid \
	-label "Node 2" $WRCOMP/comp_gen__2/node_1_comp/hard_sync_edge_valid \
	-label "Node 3" $WRCOMP/comp_gen__3/node_1_comp/hard_sync_edge_valid \
	-label "Node 4" $WRCOMP/comp_gen__4/node_1_comp/hard_sync_edge_valid

add wave -group "Synchronization type" \
	-label "Node 1" $WRCOMP/comp_gen__1/node_1_comp/core_top_comp/sync_control \
	-label "Node 2" $WRCOMP/comp_gen__2/node_1_comp/core_top_comp/sync_control \
	-label "Node 3" $WRCOMP/comp_gen__3/node_1_comp/core_top_comp/sync_control \
	-label "Node 4" $WRCOMP/comp_gen__4/node_1_comp/core_top_comp/sync_control

add wave -group "Sample control" \
	-label "Node 1" $WRCOMP/comp_gen__1/node_1_comp/core_top_comp/sp_control \
	-label "Node 2" $WRCOMP/comp_gen__2/node_1_comp/core_top_comp/sp_control \
	-label "Node 3" $WRCOMP/comp_gen__3/node_1_comp/core_top_comp/sp_control \
	-label "Node 4" $WRCOMP/comp_gen__4/node_1_comp/core_top_comp/sp_control
	
add wave -group "Time quantum start" \
	-label "Node 1" $WRCOMP/comp_gen__1/node_1_comp/prescaler_comp/tq_edge \
	-label "Node 2" $WRCOMP/comp_gen__2/node_1_comp/prescaler_comp/tq_edge \
	-label "Node 3" $WRCOMP/comp_gen__3/node_1_comp/prescaler_comp/tq_edge \
	-label "Node 4" $WRCOMP/comp_gen__4/node_1_comp/prescaler_comp/tq_edge
	
add wave -group "System clocks" \
	-label "Node 1" $WRCOMP/comp_gen__1/node_1_comp/clk_sys \
	-label "Node 2" $WRCOMP/comp_gen__2/node_1_comp/clk_sys \
	-label "Node 3" $WRCOMP/comp_gen__3/node_1_comp/clk_sys \
	-label "Node 4" $WRCOMP/comp_gen__4/node_1_comp/clk_sys
}

add wave -group "Error states" \
	-label "Node 1" $WRCOMP/comp_gen__1/node_1_comp/core_top_comp/error_state \
	-label "Node 2" $WRCOMP/comp_gen__2/node_1_comp/core_top_comp/error_state \
	-label "Node 3" $WRCOMP/comp_gen__3/node_1_comp/core_top_comp/error_state \
	-label "Node 4" $WRCOMP/comp_gen__4/node_1_comp/core_top_comp/error_state

add wave -group "Operation states" \
	-label "Node 1" $WRCOMP/comp_gen__1/node_1_comp/core_top_comp/OP_State \
	-label "Node 2" $WRCOMP/comp_gen__2/node_1_comp/core_top_comp/OP_State \
	-label "Node 3" $WRCOMP/comp_gen__3/node_1_comp/core_top_comp/OP_State \
	-label "Node 4" $WRCOMP/comp_gen__4/node_1_comp/core_top_comp/OP_State
	
add wave -group "Identifiers (Base part)" \
	-label "Node 1" $WRCOMP/comp_gen__1/node_1_comp/core_top_comp/tran_ident_base \
	-label "Node 2" $WRCOMP/comp_gen__2/node_1_comp/core_top_comp/tran_ident_base \
	-label "Node 3" $WRCOMP/comp_gen__3/node_1_comp/core_top_comp/tran_ident_base \
	-label "Node 4" $WRCOMP/comp_gen__4/node_1_comp/core_top_comp/tran_ident_base \

add wave -group "Identifiers (Extended part)" \
	-label "Node 1" $WRCOMP/comp_gen__1/node_1_comp/core_top_comp/tran_ident_ext \
	-label "Node 2" $WRCOMP/comp_gen__2/node_1_comp/core_top_comp/tran_ident_ext \
	-label "Node 3" $WRCOMP/comp_gen__3/node_1_comp/core_top_comp/tran_ident_ext \
	-label "Node 4" $WRCOMP/comp_gen__4/node_1_comp/core_top_comp/tran_ident_ext \

add wave -group "RTR flag" \
	-label "Node 1" $WRCOMP/comp_gen__1/node_1_comp/core_top_comp/tran_is_rtr \
	-label "Node 2" $WRCOMP/comp_gen__2/node_1_comp/core_top_comp/tran_is_rtr \
	-label "Node 3" $WRCOMP/comp_gen__3/node_1_comp/core_top_comp/tran_is_rtr \
	-label "Node 4" $WRCOMP/comp_gen__4/node_1_comp/core_top_comp/tran_is_rtr
	
add wave -group "Identifier type" \
	-label "Node 1" $WRCOMP/comp_gen__1/node_1_comp/core_top_comp/tran_ident_type \
	-label "Node 2" $WRCOMP/comp_gen__2/node_1_comp/core_top_comp/tran_ident_type \
	-label "Node 3" $WRCOMP/comp_gen__3/node_1_comp/core_top_comp/tran_ident_type \
	-label "Node 4" $WRCOMP/comp_gen__4/node_1_comp/core_top_comp/tran_ident_type
	
add wave -group "Frame type" \
	-label "Node 1" $WRCOMP/comp_gen__1/node_1_comp/core_top_comp/tran_frame_type \
	-label "Node 2" $WRCOMP/comp_gen__2/node_1_comp/core_top_comp/tran_frame_type \
	-label "Node 3" $WRCOMP/comp_gen__3/node_1_comp/core_top_comp/tran_frame_type \
	-label "Node 4" $WRCOMP/comp_gen__4/node_1_comp/core_top_comp/tran_frame_type

	
