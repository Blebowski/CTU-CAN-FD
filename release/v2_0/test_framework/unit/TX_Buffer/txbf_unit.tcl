################################################################################
## Author:      Ondrej Ille , Czech Technical University, FEL
## Project:     CAN FD IP Core Project
##
## 			Bit stuffing and destuffing unit test handling script
################################################################################
global TCOMP

start_CAN_simulation "tx_buf_unit_test_wrapper"

################################################################################
# Adding the waves
################################################################################

#Add common waves for each test entity
add_test_status_waves
add_system_waves

#Add circuit specific signals
add wave -label "Common Data input" -hexadecimal $TCOMP/tran_data_in

add wave -noupdate -divider -height 20 "DUT signals - Buffer 1"	
add wave -label "Buffer empty" -hexadecimal $TCOMP/txt_empty_1							
add wave -label "Discard content" $TCOMP/txt_disc_1	
add wave -label "Buffer output" -hexadecimal $TCOMP/txt_buffer_out_1
add wave -label "Discard content" $TCOMP/txt_disc_1
add wave -label "Store data" $TCOMP/drv_store_txt_1
add wave -label "Supports FD size" $TCOMP/txt_buf_comp_1/useFDsize

add wave -noupdate -divider -height 20 "DUT signals - Buffer 2"
add wave -label "Buffer empty" -hexadecimal $TCOMP/txt_empty_2							
add wave -label "Discard content" $TCOMP/txt_disc_2	
add wave -label "Buffer output" -hexadecimal $TCOMP/txt_buffer_out_2
add wave -label "Discard content" $TCOMP/txt_disc_2
add wave -label "Store data" $TCOMP/drv_store_txt_2
add wave -label "Supports FD size" $TCOMP/txt_buf_comp_2/useFDsize

add wave -noupdate -divider -height 20 "Internal testbench signals"								
add wave -label "Expected output Buffer 1" -hexadecimal $TCOMP/small_mem
add wave -label "Expected output Buffer 2" -hexadecimal $TCOMP/big_mem
									 
################################################################################
# Execute the simulation, gather results
################################################################################
run_simulation
get_test_results


