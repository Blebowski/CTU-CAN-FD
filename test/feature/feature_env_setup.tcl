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
global SIG1
global SIG2
global CORE

start_CAN_simulation "feature_env_test_wrapper"
quietly set INST1 "g_inst(1)/can_inst"
quietly set SIG1 "p(1)"
quietly set SIG2 "p(2)"
quietly set CORE  "core_top_comp"

################################################################################
# Adding the waves
################################################################################

#Add common waves for each test entity
add_test_status_waves
add wave error_ctr
add wave -label "Name of test" $TCOMP/test_name

#Add circuit specific signals

add wave -noupdate -divider -height 20 "System and CAN bus"
add wave -label "Node 1 clock" $TCOMP/$SIG1.clk_sys
add wave -label "Node 2 clock" $TCOMP/$SIG2.clk_sys
add wave -label "Node 1 reset" $TCOMP/$SIG1.res_n
add wave -label "Node 2 reset" $TCOMP/$SIG2.res_n
add wave -label "Timestamp 1" $TCOMP/$SIG1.timestamp
add wave -label "Timestamp 2" $TCOMP/$SIG2.timestamp
add wave -label "Bus level" $TCOMP/bus_level
add wave -label "Inject to bus" $TCOMP/bl_force
add wave -label "Value to inject" $TCOMP/bl_inject


add wave -noupdate -divider -height 20 "Node 1"

add wave -label "Memory bus" "$TCOMP/mem_bus(1)"
add wave -label "CAN_TX" $TCOMP/$SIG1.can_tx
add wave -label "CAN_RX" $TCOMP/$SIG1.can_rx
add wave -label "Interrupt" $TCOMP/$SIG1.int
add wave -label "Transceiver delay" $TCOMP/$SIG1.tr_del
add wave -label "Driving bus" $TCOMP/$INST1/drv_bus
add wave -label "Status bus" $TCOMP/$INST1/stat_bus

add wave -group "RX Buffer (1)" \
	-label "RX_DATA" $TCOMP/$INST1/rx_read_buff \
	-label "Buffer size" -unsigned $TCOMP/$INST1/rx_buf_size \
	-label "Full" $TCOMP/$INST1/rx_full \
	-label "Empty" $TCOMP/$INST1/rx_empty \
	-label "Frames stored" -unsigned $TCOMP/$INST1/rx_message_count \
	-label "Free words" -unsigned $TCOMP/$INST1/rx_mem_free \
	-label "Read pointer" -unsigned $TCOMP/$INST1/rx_read_pointer_pos \
	-label "Write pointer" -unsigned $TCOMP/$INST1/rx_write_pointer_pos \
	-label "Data overrun" -unsigned $TCOMP/$INST1/rx_data_overrun

add wave -group "TXT Buffers (1)" \
	-label "FSM States" $TCOMP/$INST1/txtb_fsms \
    -label "HW Commands" $TCOMP/$INST1/txt_hw_cmd

add wave -group "Interrupt manager (1)" \
	-label "Interrupt vector" $TCOMP/$INST1/int_vector \
	-label "Interrupt enable" $TCOMP/$INST1/int_ena \
	-label "Interrupt mask" $TCOMP/$INST1/int_mask

add wave -group "Frame to transmit (1)" \
	-label "Frame is valid" $TCOMP/$INST1/tran_frame_valid_out \
	-label "data" -hexadecimal $TCOMP/$INST1/tran_data_out \
	-label "DLC" $TCOMP/$INST1/tran_dlc_out \
	-label "RTR" $TCOMP/$INST1/tran_is_rtr \
	-label "Identifier type" $TCOMP/$INST1/tran_ident_type_out \
	-label "Frame format" $TCOMP/$INST1/tran_frame_type_out \
	-label "BRS" $TCOMP/$INST1/tran_brs_out

add wave -group "Frame to recieve (1)" \
	-label "Frame is valid" $TCOMP/$INST1/rec_message_valid \
	-label "Data word" -hexadecimal $TCOMP/$INST1/rx_store_data_word \
	-label "Store Data word" -hexadecimal $TCOMP/$INST1/rx_store_data \
	-label "identifier" $TCOMP/$INST1/rec_ident_in \
	-label "DLC" $TCOMP/$INST1/rec_dlc_in \
	-label "RTR" $TCOMP/$INST1/rec_is_rtr \
	-label "Identifier type" $TCOMP/$INST1/rec_ident_type_in \
	-label "Frame format" $TCOMP/$INST1/rec_frame_type_in \
	-label "BRS" $TCOMP/$INST1/rec_brs \
	-label "ESI" $TCOMP/$INST1/rec_esi

add wave -group "Prescaler (1)" \
	-label "Time quantum (Nominal)" $TCOMP/$INST1/clk_tq_nbt \
	-label "Time quantum (Data)" $TCOMP/$INST1/clk_tq_dbt \
	-label "Sync (Nominal)" $TCOMP/$INST1/sync_nbt \
	-label "Sync (Data)" $TCOMP/$INST1/sync_dbt \
	-label "Sample (Nominal)" $TCOMP/$INST1/sample_nbt \
	-label "Sample (Data)" $TCOMP/$INST1/sample_dbt \
	-label "Sample type" $TCOMP/$INST1/sp_control \
	-label "Synchronization type" $TCOMP/$INST1/sync_control \
	-label "Bit time state" $TCOMP/$INST1/bt_fsm_out \
	-label "Hard synchronization" $TCOMP/$INST1/hard_sync_edge_valid

add wave -group "CAN Core (1)" \
	-label "Protocol state" $TCOMP/$INST1/$CORE/pc_state \
	-label "Operational state" $TCOMP/$INST1/$CORE/op_state \
	-label "Error state" $TCOMP/$INST1/$CORE/error_state \
	-label "TX error counter" $TCOMP/$INST1/$CORE/tx_counter_out \
	-label "RX error counter" $TCOMP/$INST1/$CORE/rx_counter_out \
	-label "Bit error" $TCOMP/$INST1/$CORE/bit_error_valid \
	-label "Stuff error" $TCOMP/$INST1/$CORE/stuff_error_valid \
	-label "Ack error" $TCOMP/$INST1/$CORE/ack_error \
	-label "CRC error" $TCOMP/$INST1/$CORE/crc_error \
	-label "Form error" $TCOMP/$INST1/$CORE/form_error \
	-label "Transmittion finished" $TCOMP/$INST1/$CORE/tran_valid \
	-label "Reception finished" $TCOMP/$INST1/$CORE/rec_valid \
	-label "CRC 15" -hexadecimal $TCOMP/$INST1/$CORE/crc15 \
	-label "CRC 17" -hexadecimal $TCOMP/$INST1/$CORE/crc17 \
	-label "CRC 21" -hexadecimal $TCOMP/$INST1/$CORE/crc21 \
	-label "Transmitt trigger" $TCOMP/$INST1/$CORE/tran_trig \
	-label "Recieve trigger" $TCOMP/$INST1/$CORE/rec_trig \
	-label "Insert stuff bit" $TCOMP/$INST1/$CORE/data_halt \
	-label "Bit is destuffed" $TCOMP/$INST1/$CORE/destuffed \
	-label "Stuffing length" $TCOMP/$INST1/$CORE/bs_length \
	-label "DeStuffing length" $TCOMP/$INST1/$CORE/bds_length \
	-label "Stuffing enabled" $TCOMP/$INST1/$CORE/bs_enable \
	-label "DeStuffing enabled" $TCOMP/$INST1/$CORE/bds_enable \
	-label "Fixed stuff" $TCOMP/$INST1/$CORE/fixed_stuff \
	-label "Fixed de-stuff" $TCOMP/$INST1/$CORE/fixed_destuff \
	-label "Stuff count grey coded" $TCOMP/$INST1/$CORE/PC_State_comp/stuff_count_grey \
	-label "stuff_parity" $TCOMP/$INST1/$CORE/PC_State_comp/stuff_parity \
	-label "CRC check" $TCOMP/$INST1/$CORE/PC_State_comp/crc_check \
	-label "Rx parity" $TCOMP/$INST1/$CORE/PC_State_comp/rx_parity \
	-label "Rx count grey" $TCOMP/$INST1/$CORE/PC_State_comp/rx_count_grey \
	-label "Rx CRC" -hexadecimal $TCOMP/$INST1/$CORE/PC_State_comp/rec_crc_r \
	-label "Retransmit limit ena" $TCOMP/$INST1/$CORE/PC_State_comp/drv_retr_lim_ena \
	-label "Retransmit count" $TCOMP/$INST1/$CORE/PC_State_comp/retr_count \
	-label "Retransmit limit" $TCOMP/$INST1/$CORE/PC_State_comp/drv_retr_th \
	-label "Stuff counter" $TCOMP/$INST1/$CORE/st_ctr_resolved
    

add wave -group "Bus sampling (1)" \
	-label "Measure transceiver delay" $TCOMP/$INST1/trv_delay_calib \
	-label "Transceiver delay" $TCOMP/$INST1/bus_sync_comp/trv_delay \
	-label "Measurment running" $TCOMP/$INST1/bus_sync_comp/trv_running \
	-label "Reset secondary sampling" $TCOMP/$INST1/ssp_reset \
	-label "Bit error secondary sampling" $TCOMP/$INST1/bit_error_sec_sam \
	-label "Tripple sampling registers" $TCOMP/$INST1/bus_sync_comp/trs_reg \



add wave -noupdate -divider -height 20 "Node 2"
quietly set INST1 "g_inst(2)/can_inst"

add wave -label "Memory bus" "$TCOMP/mem_bus(2)"
add wave -label "CAN_TX" $TCOMP/$SIG2.can_tx
add wave -label "CAN_RX" $TCOMP/$SIG2.can_rx
add wave -label "Interrupt" $TCOMP/$SIG2.int
add wave -label "Transceiver delay" $TCOMP/$SIG2.tr_del
add wave -label "Driving bus" $TCOMP/$INST1/drv_bus
add wave -label "Status bus" $TCOMP/$INST1/stat_bus

add wave -group "RX Buffer (2)" \
	-label "RX_DATA" $TCOMP/$INST1/rx_read_buff \
	-label "Buffer size" -unsigned $TCOMP/$INST1/rx_buf_size \
	-label "Full" $TCOMP/$INST1/rx_full \
	-label "Empty" $TCOMP/$INST1/rx_empty \
	-label "Frames stored" -unsigned $TCOMP/$INST1/rx_message_count \
	-label "Free words" -unsigned $TCOMP/$INST1/rx_mem_free \
	-label "Read pointer" -unsigned $TCOMP/$INST1/rx_read_pointer_pos \
	-label "Write pointer" -unsigned $TCOMP/$INST1/rx_write_pointer_pos \
	-label "Data overrun" -unsigned $TCOMP/$INST1/rx_data_overrun

add wave -group "TXT Buffers (2)" \
	-label "FSM States" $TCOMP/$INST1/txtb_fsms

add wave -group "Interrupt manager (2)" \
	-label "Interrupt vector" $TCOMP/$INST1/int_vector \
	-label "Interrupt enable" $TCOMP/$INST1/int_ena \
	-label "Interrupt mask" $TCOMP/$INST1/int_mask

add wave -group "Frame to transmit (2)" \
	-label "Frame is valid" $TCOMP/$INST1/tran_frame_valid_out \
	-label "data" -hexadecimal $TCOMP/$INST1/tran_data_out \
	-label "DLC" $TCOMP/$INST1/tran_dlc_out \
	-label "RTR" $TCOMP/$INST1/tran_is_rtr \
	-label "Identifier type" $TCOMP/$INST1/tran_ident_type_out \
	-label "Frame format" $TCOMP/$INST1/tran_frame_type_out \
	-label "BRS" $TCOMP/$INST1/tran_brs_out

add wave -group "Frame to recieve (2)" \
	-label "Frame is valid" $TCOMP/$INST1/rec_message_valid \
	-label "Data word" -hexadecimal $TCOMP/$INST1/rx_store_data_word \
	-label "Store Data word" -hexadecimal $TCOMP/$INST1/rx_store_data \
	-label "identifier" $TCOMP/$INST1/rec_ident_in \
	-label "DLC" $TCOMP/$INST1/rec_dlc_in \
	-label "RTR" $TCOMP/$INST1/rec_is_rtr \
	-label "Identifier type" $TCOMP/$INST1/rec_ident_type_in \
	-label "Frame format" $TCOMP/$INST1/rec_frame_type_in \
	-label "BRS" $TCOMP/$INST1/rec_brs \
	-label "ESI" $TCOMP/$INST1/rec_esi

add wave -group "Prescaler (2)" \
	-label "Time quantum (Nominal)" $TCOMP/$INST1/clk_tq_nbt \
	-label "Time quantum (Data)" $TCOMP/$INST1/clk_tq_dbt \
	-label "Sync (Nominal)" $TCOMP/$INST1/sync_nbt \
	-label "Sync (Data)" $TCOMP/$INST1/sync_dbt \
	-label "Sample (Nominal)" $TCOMP/$INST1/sample_nbt \
	-label "Sample (Data)" $TCOMP/$INST1/sample_dbt \
	-label "Sample type" $TCOMP/$INST1/sp_control \
	-label "Synchronization type" $TCOMP/$INST1/sync_control \
	-label "Bit time state" $TCOMP/$INST1/bt_fsm_out \
	-label "Hard synchronization" $TCOMP/$INST1/hard_sync_edge_valid

add wave -group "CAN Core (2)" \
	-label "Protocol state" $TCOMP/$INST1/$CORE/pc_state \
	-label "Operational state" $TCOMP/$INST1/$CORE/op_state \
	-label "Error state" $TCOMP/$INST1/$CORE/error_state \
	-label "TX error counter" $TCOMP/$INST1/$CORE/tx_counter_out \
	-label "RX error counter" $TCOMP/$INST1/$CORE/rx_counter_out \
	-label "Bit error" $TCOMP/$INST1/$CORE/bit_error_valid \
	-label "Stuff error" $TCOMP/$INST1/$CORE/stuff_error_valid \
	-label "Ack error" $TCOMP/$INST1/$CORE/ack_error \
	-label "CRC error" $TCOMP/$INST1/$CORE/crc_error \
	-label "Form error" $TCOMP/$INST1/$CORE/form_error \
	-label "Transmittion finished" $TCOMP/$INST1/$CORE/tran_valid \
	-label "Reception finished" $TCOMP/$INST1/$CORE/rec_valid \
	-label "CRC 15" -hexadecimal $TCOMP/$INST1/$CORE/crc15 \
	-label "CRC 17" -hexadecimal $TCOMP/$INST1/$CORE/crc17 \
	-label "CRC 21" -hexadecimal $TCOMP/$INST1/$CORE/crc21 \
	-label "Transmitt trigger" $TCOMP/$INST1/$CORE/tran_trig \
	-label "Recieve trigger" $TCOMP/$INST1/$CORE/rec_trig \
	-label "Insert stuff bit" $TCOMP/$INST1/$CORE/data_halt \
	-label "Bit is destuffed" $TCOMP/$INST1/$CORE/destuffed \
	-label "Stuffing length" $TCOMP/$INST1/$CORE/bs_length \
	-label "DeStuffing length" $TCOMP/$INST1/$CORE/bds_length \
	-label "Stuffing enabled" $TCOMP/$INST1/$CORE/bs_enable \
	-label "DeStuffing enabled" $TCOMP/$INST1/$CORE/bds_enable \
	-label "Fixed stuff" $TCOMP/$INST1/$CORE/fixed_stuff \
	-label "Fixed de-stuff" $TCOMP/$INST1/$CORE/fixed_destuff \
	-label "Stuff counter" $TCOMP/$INST1/$CORE/st_ctr_resolved

add wave -group "Bus sampling (2)" \
	-label "Measure transceiver delay" $TCOMP/$INST1/trv_delay_calib \
	-label "Transceiver delay" $TCOMP/$INST1/bus_sync_comp/trv_delay \
	-label "Measurment running" $TCOMP/$INST1/bus_sync_comp/trv_running \
	-label "Reset secondary sampling" $TCOMP/$INST1/ssp_reset \
	-label "Bit error secondary sampling" $TCOMP/$INST1/bit_error_sec_sam \
	-label "Tripple sampling registers" $TCOMP/$INST1/bus_sync_comp/trs_reg \
