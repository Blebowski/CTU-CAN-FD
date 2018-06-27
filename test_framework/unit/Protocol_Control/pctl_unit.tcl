################################################################################
## Author:      Ondrej Ille , Czech Technical University, FEL
## Project:     CAN FD IP Core Project
##
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

add wave -noupdate -divider -height 20 "Protocol Control 1 inputs"	
add wave -label "Transmitted frame" $TCOMP/tx_frame
add wave -label "Frame transmitted OK" $TCOMP/tran_valid_1
add wave -label "Acknowledge received" $TCOMP/ack_recieved_out_1
add wave -label "PC State" $TCOMP/pc_state_out_1
add wave -label "Operational State" $TCOMP/op_state_1
add wave -label "Stuff length" -unsigned $TCOMP/stuff_length_1


add wave -noupdate -divider -height 20 "Protocol Control 2 outputs"
add wave -label "Received frame" $TCOMP/rx_frame
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
add wave -label "Frame on CAN bus" $TCOMP/test_proc/rec_seq
add wave -label "Expected frame length" $TCOMP/test_proc/seq_length
add wave -label "TX/RX data equal" $TCOMP/test_proc/out_frm
add wave -label "SW/Recieved sequence equal" $TCOMP/test_proc/out_seq						
									 
################################################################################
# Execute the simulation, gather results
################################################################################
run_simulation
get_test_results


