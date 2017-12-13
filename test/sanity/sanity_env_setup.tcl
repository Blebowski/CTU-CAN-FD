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
add wave $TCOMP/error_ctr

#Add circuit specific signals
if { $SILENT_SANITY == "false" } {
	add wave -group "Test configuration" \
		-label "Configuration name" $TCOMP/test_desc \
		-label "Topology" $TCOMP/topology \
		-label "Bus Matrix" $TCOMP/bus_matrix \
		-label "Clock tolerances (error)" $TCOMP/epsilon_v \
		-label "Transciever delay" $TCOMP/trv_del_v \
		-label "Noise width - mean (ns)" $TCOMP/nw_mean \
		-label "Noise width - variance (ns)" $TCOMP/nw_var \
		-label "Noise gap - mean (ns)" $TCOMP/ng_mean \
		-label "Noise gap - variance (ns)" $TCOMP/ng_var \
		-label "Bus timing" $TCOMP/timing_config
} else {
	add wave -group "Test configuration" \
		-label "Configuration name" $TCOMP/test_desc
}

if { $SILENT_SANITY == "false" } {
	add wave -label "Timestamps" $TCOMP/timestamp_v
}

add wave -label "Noise values" $TCOMP/noise_reg
add wave -label "Noise force" -event $TCOMP/noise_force

if { $SILENT_SANITY == "false" } {
	add wave -group "Test control and status signals" \
		-label "TX memories full" $TCOMP/tx_full \
		-label "Wait" $TCOMP/do_wait \
		-label "Config" $TCOMP/do_config \
		-label "DO traffic" $TCOMP/do_traffic \
		-label "Erase memories" $TCOMP/do_erase_mems \
		-label "Generate noise" $TCOMP/do_noise
}

if { $SILENT_SANITY == "false" } {
	add wave -label "Memory buses" $TCOMP/mb_arr
}

add wave -label "CAN Tx" -expand $TCOMP/can_tx_v
add wave -label "CAN Rx" -expand $TCOMP/can_rx_v

add wave -group "Protocol states" \
	-label "Node 1" $TCOMP/comp_gen__1/node_1_comp/core_top_comp/PC_State \
	-label "Node 2" $TCOMP/comp_gen__2/node_1_comp/core_top_comp/PC_State \
	-label "Node 3" $TCOMP/comp_gen__3/node_1_comp/core_top_comp/PC_State \
	-label "Node 4" $TCOMP/comp_gen__4/node_1_comp/core_top_comp/PC_State

if { $SILENT_SANITY == "false" } {
add wave -group "Bit time state" \
	-label "Node 1" $TCOMP/comp_gen__1/node_1_comp/bt_FSM_out \
	-label "Node 2" $TCOMP/comp_gen__2/node_1_comp/bt_FSM_out \
	-label "Node 3" $TCOMP/comp_gen__3/node_1_comp/bt_FSM_out \
	-label "Node 4" $TCOMP/comp_gen__4/node_1_comp/bt_FSM_out
	
add wave -group "Trancieve triggers" \
	-label "Node 1" $TCOMP/comp_gen__1/node_1_comp/core_top_comp/tran_trig \
	-label "Node 2" $TCOMP/comp_gen__2/node_1_comp/core_top_comp/tran_trig \
	-label "Node 3" $TCOMP/comp_gen__3/node_1_comp/core_top_comp/tran_trig \
	-label "Node 4" $TCOMP/comp_gen__4/node_1_comp/core_top_comp/tran_trig
	
add wave -group "Recieve triggers" \
	-label "Node 1" $TCOMP/comp_gen__1/node_1_comp/core_top_comp/rec_trig \
	-label "Node 2" $TCOMP/comp_gen__2/node_1_comp/core_top_comp/rec_trig \
	-label "Node 3" $TCOMP/comp_gen__3/node_1_comp/core_top_comp/rec_trig \
	-label "Node 4" $TCOMP/comp_gen__4/node_1_comp/core_top_comp/rec_trig
	
add wave -group "Hard sync edge" \
	-label "Node 1" $TCOMP/comp_gen__1/node_1_comp/hard_sync_edge_valid \
	-label "Node 2" $TCOMP/comp_gen__2/node_1_comp/hard_sync_edge_valid \
	-label "Node 3" $TCOMP/comp_gen__3/node_1_comp/hard_sync_edge_valid \
	-label "Node 4" $TCOMP/comp_gen__4/node_1_comp/hard_sync_edge_valid

add wave -group "Synchronization type" \
	-label "Node 1" $TCOMP/comp_gen__1/node_1_comp/core_top_comp/sync_control \
	-label "Node 2" $TCOMP/comp_gen__2/node_1_comp/core_top_comp/sync_control \
	-label "Node 3" $TCOMP/comp_gen__3/node_1_comp/core_top_comp/sync_control \
	-label "Node 4" $TCOMP/comp_gen__4/node_1_comp/core_top_comp/sync_control

add wave -group "Sample control" \
	-label "Node 1" $TCOMP/comp_gen__1/node_1_comp/core_top_comp/sp_control \
	-label "Node 2" $TCOMP/comp_gen__2/node_1_comp/core_top_comp/sp_control \
	-label "Node 3" $TCOMP/comp_gen__3/node_1_comp/core_top_comp/sp_control \
	-label "Node 4" $TCOMP/comp_gen__4/node_1_comp/core_top_comp/sp_control
	
add wave -group "Time quantum start" \
	-label "Node 1" $TCOMP/comp_gen__1/node_1_comp/prescaler_comp/tq_edge \
	-label "Node 2" $TCOMP/comp_gen__2/node_1_comp/prescaler_comp/tq_edge \
	-label "Node 3" $TCOMP/comp_gen__3/node_1_comp/prescaler_comp/tq_edge \
	-label "Node 4" $TCOMP/comp_gen__4/node_1_comp/prescaler_comp/tq_edge
	
add wave -group "System clocks" \
	-label "Node 1" $TCOMP/comp_gen__1/node_1_comp/clk_sys \
	-label "Node 2" $TCOMP/comp_gen__2/node_1_comp/clk_sys \
	-label "Node 3" $TCOMP/comp_gen__3/node_1_comp/clk_sys \
	-label "Node 4" $TCOMP/comp_gen__4/node_1_comp/clk_sys
}

add wave -group "Error states" \
	-label "Node 1" $TCOMP/comp_gen__1/node_1_comp/core_top_comp/error_state \
	-label "Node 2" $TCOMP/comp_gen__2/node_1_comp/core_top_comp/error_state \
	-label "Node 3" $TCOMP/comp_gen__3/node_1_comp/core_top_comp/error_state \
	-label "Node 4" $TCOMP/comp_gen__4/node_1_comp/core_top_comp/error_state

add wave -group "Operation states" \
	-label "Node 1" $TCOMP/comp_gen__1/node_1_comp/core_top_comp/OP_State \
	-label "Node 2" $TCOMP/comp_gen__2/node_1_comp/core_top_comp/OP_State \
	-label "Node 3" $TCOMP/comp_gen__3/node_1_comp/core_top_comp/OP_State \
	-label "Node 4" $TCOMP/comp_gen__4/node_1_comp/core_top_comp/OP_State
	
add wave -group "Identifiers (Base part)" \
	-label "Node 1" $TCOMP/comp_gen__1/node_1_comp/core_top_comp/tran_ident_base \
	-label "Node 2" $TCOMP/comp_gen__2/node_1_comp/core_top_comp/tran_ident_base \
	-label "Node 3" $TCOMP/comp_gen__3/node_1_comp/core_top_comp/tran_ident_base \
	-label "Node 4" $TCOMP/comp_gen__4/node_1_comp/core_top_comp/tran_ident_base \

add wave -group "Identifiers (Extended part)" \
	-label "Node 1" $TCOMP/comp_gen__1/node_1_comp/core_top_comp/tran_ident_ext \
	-label "Node 2" $TCOMP/comp_gen__2/node_1_comp/core_top_comp/tran_ident_ext \
	-label "Node 3" $TCOMP/comp_gen__3/node_1_comp/core_top_comp/tran_ident_ext \
	-label "Node 4" $TCOMP/comp_gen__4/node_1_comp/core_top_comp/tran_ident_ext \

add wave -group "RTR flag" \
	-label "Node 1" $TCOMP/comp_gen__1/node_1_comp/core_top_comp/tran_is_rtr \
	-label "Node 2" $TCOMP/comp_gen__2/node_1_comp/core_top_comp/tran_is_rtr \
	-label "Node 3" $TCOMP/comp_gen__3/node_1_comp/core_top_comp/tran_is_rtr \
	-label "Node 4" $TCOMP/comp_gen__4/node_1_comp/core_top_comp/tran_is_rtr
	
add wave -group "Identifier type" \
	-label "Node 1" $TCOMP/comp_gen__1/node_1_comp/core_top_comp/tran_ident_type \
	-label "Node 2" $TCOMP/comp_gen__2/node_1_comp/core_top_comp/tran_ident_type \
	-label "Node 3" $TCOMP/comp_gen__3/node_1_comp/core_top_comp/tran_ident_type \
	-label "Node 4" $TCOMP/comp_gen__4/node_1_comp/core_top_comp/tran_ident_type
	
add wave -group "Frame type" \
	-label "Node 1" $TCOMP/comp_gen__1/node_1_comp/core_top_comp/tran_frame_type \
	-label "Node 2" $TCOMP/comp_gen__2/node_1_comp/core_top_comp/tran_frame_type \
	-label "Node 3" $TCOMP/comp_gen__3/node_1_comp/core_top_comp/tran_frame_type \
	-label "Node 4" $TCOMP/comp_gen__4/node_1_comp/core_top_comp/tran_frame_type
	
	