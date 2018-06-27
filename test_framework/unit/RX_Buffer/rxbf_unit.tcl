
################################################################################
## Author:      Ondrej Ille , Czech Technical University, FEL
## Project:     CAN FD IP Core Project
##
## 			RX Buffers unit test handling script
################################################################################
global TCOMP

start_CAN_simulation "rx_buf_unit_test_wrapper"

################################################################################
# Adding the waves
################################################################################

#Add common waves for each test entity
add_test_status_waves
add_system_waves

#Add circuit specific signals
add wave -noupdate -divider -height 20 "DUT inputs (generated)"
add wave -label "CAN Frame to store into RX Buffer" $TCOMP/input_frame
add wave -label "Timestamp" -unsigned $TCOMP/timestamp
add wave -label "Erase Rx buffer" $TCOMP/rx_buffer_comp/drv_erase_rx
add wave -label "Read word" $TCOMP/rx_buffer_comp/drv_read_start

add wave -noupdate -divider -height 20 "RX Buffer status signals"
add wave -label "Buffer full" $TCOMP/rx_full_b
add wave -label "Buffer empty" $TCOMP/rx_empty_b
add wave -label "Stored frames" -unsigned $TCOMP/rx_message_count_b
add wave -label "Free 32bit words" -unsigned $TCOMP/rx_mem_free_b
add wave -label "Read pointer" -unsigned $TCOMP/rx_read_pointer_pos_b
add wave -label "Write pointer" -unsigned $TCOMP/rx_write_pointer_pos_b
add wave -label "Frame discarded" $TCOMP/rx_message_disc_b
add wave -label "Output word" -hexadecimal $TCOMP/rx_read_buff_b

add wave -noupdate -divider -height 20 "Testbench internal signals"
add wave -label "Input memory full" $TCOMP/in_mem_full
add wave -label "Output memory full" $TCOMP/out_mem_full
add wave -label "Input memory" -hexadecimal $TCOMP/in_mem
add wave -label "Output memory" -hexadecimal $TCOMP/out_mem	
add wave -label "Input mem pointer" -unsigned $TCOMP/in_pointer
add wave -label "Output mem pointer" -unsigned $TCOMP/out_pointer	
add wave -label "???InMem=OutMem???" $TCOMP/cons_check/cons_res

add wave -noupdate -divider -height 20 "Internal DUT signals"
add wave -label "RX Buffer content" $TCOMP/rx_Buffer_comp/memory
add wave -label "RX Buffer memory row valid" $TCOMP/rx_Buffer_comp/memory_valid


								 
################################################################################
# Execute the simulation, gather results
################################################################################
run_simulation
get_test_results

