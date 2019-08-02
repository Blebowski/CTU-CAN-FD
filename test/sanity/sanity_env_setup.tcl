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
quietly set CORE  "can_core_inst"

################################################################################
# Adding the waves
################################################################################

#Add common waves for each test entity
add_test_status_waves

# Sanity test was moved to the wrapper in order to create inputs in VHDL
# for AHDL simulation. Component path must be added to have correct path
# to internal signals
set WRCOMP $TCOMP
append WRCOMP "/t_sanity"

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
        -label "Wait till end of integration" $WRCOMP/do_wait_integ_end \
        -label "Integration done" $WRCOMP/integ_done \
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
	-label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/protocol_control_inst/protocol_control_fsm_inst/curr_state \
	-label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/protocol_control_inst/protocol_control_fsm_inst/curr_state \
	-label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/protocol_control_inst/protocol_control_fsm_inst/curr_state \
	-label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/protocol_control_inst/protocol_control_fsm_inst/curr_state

add wave -group "Data Halt (Stuff bit) - Data input" \
    -label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/data_halt \
    -label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/data_halt \
    -label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/data_halt \
    -label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/data_halt \

add wave -group "Stuff counter" \
    -label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/bst_ctr \
    -label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/bst_ctr \
    -label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/bst_ctr \
    -label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/bst_ctr \

add wave -group "TX Load stuff count" \
    -label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/protocol_control_inst/tx_load_stuff_count \
    -label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/protocol_control_inst/tx_load_stuff_count \
    -label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/protocol_control_inst/tx_load_stuff_count \
    -label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/protocol_control_inst/tx_load_stuff_count \

add wave -group "TX Shift register status" \
    -label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/protocol_control_inst/tx_shift_reg_inst/tx_shift_reg_inst/reg_stat \
    -label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/protocol_control_inst/tx_shift_reg_inst/tx_shift_reg_inst/reg_stat \
    -label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/protocol_control_inst/tx_shift_reg_inst/tx_shift_reg_inst/reg_stat \
    -label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/protocol_control_inst/tx_shift_reg_inst/tx_shift_reg_inst/reg_stat \

add wave -group "CRC15" \
    -label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/crc_15 \
    -label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/crc_15 \
    -label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/crc_15 \
    -label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/crc_15 \

add wave -group "CRC17" \
    -label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/crc_17 \
    -label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/crc_17 \
    -label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/crc_17 \
    -label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/crc_17 \

add wave -group "CRC17 - Data input" \
    -label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/can_crc_inst/crc_17_21_data_in \
    -label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/can_crc_inst/crc_17_21_data_in \
    -label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/can_crc_inst/crc_17_21_data_in \
    -label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/can_crc_inst/crc_17_21_data_in \

add wave -group "CRC17 - Trigger" \
    -label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/can_crc_inst/crc_17_21_trigger \
    -label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/can_crc_inst/crc_17_21_trigger \
    -label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/can_crc_inst/crc_17_21_trigger \
    -label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/can_crc_inst/crc_17_21_trigger \

add wave -group "CRC21" \
    -label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/crc_21 \
    -label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/crc_21 \
    -label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/crc_21 \
    -label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/crc_21 \

add wave -group "CRC Calc from RX" \
    -label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/crc_calc_from_rx \
    -label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/crc_calc_from_rx \
    -label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/crc_calc_from_rx \
    -label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/crc_calc_from_rx \

add wave -group "CRC Check" \
    -label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/protocol_control_inst/crc_check \
    -label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/protocol_control_inst/crc_check \
    -label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/protocol_control_inst/crc_check \
    -label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/protocol_control_inst/crc_check \

add wave -group "CRC 17 OK" \
    -label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/protocol_control_inst/err_detector_inst/crc_17_ok \
    -label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/protocol_control_inst/err_detector_inst/crc_17_ok \
    -label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/protocol_control_inst/err_detector_inst/crc_17_ok \
    -label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/protocol_control_inst/err_detector_inst/crc_17_ok \

add wave -group "CRC source" \
    -label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/protocol_control_inst/err_detector_inst/crc_src \
    -label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/protocol_control_inst/err_detector_inst/crc_src \
    -label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/protocol_control_inst/err_detector_inst/crc_src \
    -label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/protocol_control_inst/err_detector_inst/crc_src \

add wave -group "RX CRC 17" \
    -label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/protocol_control_inst/err_detector_inst/rx_crc_17 \
    -label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/protocol_control_inst/err_detector_inst/rx_crc_17 \
    -label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/protocol_control_inst/err_detector_inst/rx_crc_17 \
    -label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/protocol_control_inst/err_detector_inst/rx_crc_17 \

add wave -group "TX Load CRC" \
    -label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/protocol_control_inst/tx_load_crc \
    -label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/protocol_control_inst/tx_load_crc \
    -label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/protocol_control_inst/tx_load_crc \
    -label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/protocol_control_inst/tx_load_crc \

add wave -group "Control counter" \
    -label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/protocol_control_inst/control_counter_inst/ctrl_ctr_q \
    -label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/protocol_control_inst/control_counter_inst/ctrl_ctr_q \
    -label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/protocol_control_inst/control_counter_inst/ctrl_ctr_q \
    -label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/protocol_control_inst/control_counter_inst/ctrl_ctr_q \

add wave -group "Error frame request" \
    -label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/protocol_control_inst/protocol_control_fsm_inst/err_frm_req \
    -label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/protocol_control_inst/protocol_control_fsm_inst/err_frm_req \
    -label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/protocol_control_inst/protocol_control_fsm_inst/err_frm_req \
    -label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/protocol_control_inst/protocol_control_fsm_inst/err_frm_req

add wave -group "Increment by 1" \
    -label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/fault_confinement_inst/inc_one \
    -label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/fault_confinement_inst/inc_one \
    -label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/fault_confinement_inst/inc_one \
    -label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/fault_confinement_inst/inc_one

add wave -group "Increment by 8" \
    -label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/fault_confinement_inst/inc_eight \
    -label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/fault_confinement_inst/inc_eight \
    -label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/fault_confinement_inst/inc_eight \
    -label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/fault_confinement_inst/inc_eight 
    
add wave -group "Decrement by 1" \
    -label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/fault_confinement_inst/dec_one \
    -label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/fault_confinement_inst/dec_one \
    -label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/fault_confinement_inst/dec_one \
    -label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/fault_confinement_inst/dec_one 

if { $SILENT_SANITY == "false" } {
add wave -group "Bit time state" \
	-label "Node 1" $WRCOMP/comp_gen__1/can_inst/bt_fsm \
	-label "Node 2" $WRCOMP/comp_gen__2/can_inst/bt_fsm \
	-label "Node 3" $WRCOMP/comp_gen__3/can_inst/bt_fsm \
	-label "Node 4" $WRCOMP/comp_gen__4/can_inst/bt_fsm
	
add wave -group "Trancieve triggers" \
	-label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/pc_tx_trigger \
	-label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/pc_tx_trigger \
	-label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/pc_tx_trigger \
	-label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/pc_tx_trigger
	
add wave -group "Recieve triggers" \
	-label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/pc_rx_trigger \
	-label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/pc_rx_trigger \
	-label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/pc_rx_trigger \
	-label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/pc_rx_trigger
	
add wave -group "Hard sync edge" \
	-label "Node 1" $WRCOMP/comp_gen__1/can_inst/prescaler_inst/h_sync_valid \
	-label "Node 2" $WRCOMP/comp_gen__2/can_inst/prescaler_inst/h_sync_valid \
	-label "Node 3" $WRCOMP/comp_gen__3/can_inst/prescaler_inst/h_sync_valid \
	-label "Node 4" $WRCOMP/comp_gen__4/can_inst/prescaler_inst/h_sync_valid

add wave -group "Resync edge" \
    -label "Node 1" $WRCOMP/comp_gen__1/can_inst/prescaler_inst/resync_edge_valid \
    -label "Node 2" $WRCOMP/comp_gen__2/can_inst/prescaler_inst/resync_edge_valid \
    -label "Node 3" $WRCOMP/comp_gen__3/can_inst/prescaler_inst/resync_edge_valid \
    -label "Node 4" $WRCOMP/comp_gen__4/can_inst/prescaler_inst/resync_edge_valid

add wave -group "Segment counter nominal" \
    -label "Node 1" $WRCOMP/comp_gen__1/can_inst/prescaler_inst/segm_counter_nbt \
    -label "Node 2" $WRCOMP/comp_gen__2/can_inst/prescaler_inst/segm_counter_nbt \
    -label "Node 3" $WRCOMP/comp_gen__3/can_inst/prescaler_inst/segm_counter_nbt \
    -label "Node 4" $WRCOMP/comp_gen__4/can_inst/prescaler_inst/segm_counter_nbt

add wave -group "Segment counter data" \
    -label "Node 1" $WRCOMP/comp_gen__1/can_inst/prescaler_inst/segm_counter_dbt \
    -label "Node 2" $WRCOMP/comp_gen__2/can_inst/prescaler_inst/segm_counter_dbt \
    -label "Node 3" $WRCOMP/comp_gen__3/can_inst/prescaler_inst/segm_counter_dbt \
    -label "Node 4" $WRCOMP/comp_gen__4/can_inst/prescaler_inst/segm_counter_dbt


add wave -group "Synchronization type" \
	-label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/sync_control \
	-label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/sync_control \
	-label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/sync_control \
	-label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/sync_control

add wave -group "Sample control" \
	-label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/sp_control_i \
	-label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/sp_control_i \
	-label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/sp_control_i \
	-label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/sp_control_i

add wave -group "System clocks" \
	-label "Node 1" $WRCOMP/comp_gen__1/can_inst/clk_sys \
	-label "Node 2" $WRCOMP/comp_gen__2/can_inst/clk_sys \
	-label "Node 3" $WRCOMP/comp_gen__3/can_inst/clk_sys \
	-label "Node 4" $WRCOMP/comp_gen__4/can_inst/clk_sys
}

add wave -group "Error states" \
	-label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/fault_confinement_inst/fault_confinement_fsm_inst/curr_state \
	-label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/fault_confinement_inst/fault_confinement_fsm_inst/curr_state \
	-label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/fault_confinement_inst/fault_confinement_fsm_inst/curr_state \
	-label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/fault_confinement_inst/fault_confinement_fsm_inst/curr_state

add wave -group "Operation states" \
	-label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/operation_control_inst/curr_state \
	-label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/operation_control_inst/curr_state \
	-label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/operation_control_inst/curr_state \
	-label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/operation_control_inst/curr_state
	
add wave -group "Identifier shift registers (Base part)" \
	-label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/PC_State_comp/tran_ident_base_sr \
	-label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/PC_State_comp/tran_ident_base_sr \
	-label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/PC_State_comp/tran_ident_base_sr \
	-label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/PC_State_comp/tran_ident_base_sr \

add wave -group "Identifiers shift registers (Extended part)" \
	-label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/PC_State_comp/tran_ident_ext_sr \
	-label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/PC_State_comp/tran_ident_ext_sr \
	-label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/PC_State_comp/tran_ident_ext_sr \
	-label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/PC_State_comp/tran_ident_ext_sr \

add wave -group "RTR flag" \
	-label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/tran_is_rtr \
	-label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/tran_is_rtr \
	-label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/tran_is_rtr \
	-label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/tran_is_rtr
	
add wave -group "Identifier type" \
	-label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/tran_ident_type \
	-label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/tran_ident_type \
	-label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/tran_ident_type \
	-label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/tran_ident_type
	
add wave -group "Frame type" \
	-label "Node 1" $WRCOMP/comp_gen__1/can_inst/can_core_inst/tran_frame_type \
	-label "Node 2" $WRCOMP/comp_gen__2/can_inst/can_core_inst/tran_frame_type \
	-label "Node 3" $WRCOMP/comp_gen__3/can_inst/can_core_inst/tran_frame_type \
	-label "Node 4" $WRCOMP/comp_gen__4/can_inst/can_core_inst/tran_frame_type

	
