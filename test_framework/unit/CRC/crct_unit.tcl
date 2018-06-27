################################################################################
## Author:      Ondrej Ille , Czech Technical University, FEL
## Project:     CAN FD IP Core Project
##
## 			CRC unit test handling script
################################################################################
global TCOMP

start_CAN_simulation "crc_unit_test_wrapper"

################################################################################
# Adding the waves
################################################################################

#Add common waves for each test entity
add_test_status_waves
add_system_waves

#Add circuit specific signals
add wave -noupdate -divider -height 20 "CRC polynomials"
add wave -hexadecimal $TCOMP/CRC_comp/crc15_pol
add wave -hexadecimal $TCOMP/CRC_comp/crc17_pol
add wave -hexadecimal $TCOMP/CRC_comp/crc21_pol

add wave -noupdate -divider -height 20 "DUT inputs (generated)"
add wave $TCOMP/enable
add wave $TCOMP/drv_fd_type
add wave $TCOMP/trig
add wave $TCOMP/data_in

add wave -noupdate -divider -height 20 "DUT outputs"
add wave $TCOMP/crc15
add wave $TCOMP/crc17
add wave $TCOMP/crc21

add wave -noupdate -divider -height 20 "Testbench internal signals"
add wave -label "Length of bit sequence" $TCOMP/test_proc/gen_length 
add wave -label "SW CRC15" $TCOMP/test_proc/crc_15_mod 
add wave -label "SW CRC17" $TCOMP/test_proc/crc_17_mod 
add wave -label "SW CRC21" $TCOMP/test_proc/crc_21_mod 
add wave -label "CRC15 Mismatch" $TCOMP/test_proc/c15_mism 
add wave -label "CRC17 Mismatch" $TCOMP/test_proc/c17_mism 
add wave -label "CRC21 Mismatch" $TCOMP/test_proc/c21_mism 

add wave -noupdate -divider -height 20 "Internal DUT signals"								
add wave $TCOMP/CRC_comp/crc15_cycle/crc15_nxt
add wave $TCOMP/CRC_comp/crc17_cycle/crc17_nxt
add wave $TCOMP/CRC_comp/crc21_cycle/crc21_nxt
									 
################################################################################
# Execute the simulation, gather results
################################################################################
run_simulation
get_test_results


