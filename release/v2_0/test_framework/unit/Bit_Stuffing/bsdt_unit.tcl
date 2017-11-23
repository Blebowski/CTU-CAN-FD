################################################################################
## Author:      Ondrej Ille , Czech Technical University, FEL
## Project:     CAN FD IP Core Project
##
## 			Bit stuffing and destuffing unit test handling script
################################################################################
global TCOMP

start_CAN_simulation "bit_stuffing_unit_test_wrapper"

################################################################################
# Adding the waves
################################################################################

#Add common waves for each test entity
add_test_status_waves
add_system_waves

#Add circuit specific signals
add wave -noupdate -divider -height 20 "DUT inputs (generated)"
add wave -group "Triggering signals" $TCOMP/tx_trig \
									 $TCOMP/bs_trig \
									 $TCOMP/bd_trig \
									 $TCOMP/rx_trig
									 
add wave -label "Data before bit stuffing" $TCOMP/tx_data								
add wave -label "Data before bit De-stuffing" $TCOMP/joined_data								
add wave -label "Bit stuffing enabled" $TCOMP/bs_enable
add wave -label "Bit De-stuffing enabled" $TCOMP/bd_enable
add wave -label "Fixed bit stuffing method" $TCOMP/fixed_stuff
add wave -label "Length of bit stuffing rule" -unsigned $TCOMP/bs_length
add wave -label "Length of bit De-stuffing rule" -unsigned $TCOMP/bd_length

add wave -noupdate -divider -height 20 "DUT outputs"
add wave -label "Bit stuffed" $TCOMP/data_halt
add wave -label "Bit De-stuffed" $TCOMP/destuffed
add wave -label "Stuff error" $TCOMP/stuff_error
add wave -label "Data after bit stuffing" $TCOMP/stuffed_data
add wave -label "Data after bit De-stuffing" $TCOMP/rx_data
add wave -label "Stuff count mod 8" -unsigned $TCOMP/bs_ctr
add wave -label "De-Stuff count mod 8" -unsigned $TCOMP/bd_ctr

add wave -noupdate -divider -height 20 "Internal DUT signals"								
add wave $TCOMP/bitstufcomp/same_bits
add wave $TCOMP/bitstufcomp/prev_bit
add wave $TCOMP/bitdestcomp/same_bits
add wave $TCOMP/bitdestcomp/prev_val
									 
################################################################################
# Execute the simulation, gather results
################################################################################
run_simulation
get_test_results


