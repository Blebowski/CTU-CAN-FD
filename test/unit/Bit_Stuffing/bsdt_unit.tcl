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
## Description:
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


