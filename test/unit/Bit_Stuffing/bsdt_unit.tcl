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
add wave -group "Triggering signals" $TCOMP/tx_trig_ack \
									 $TCOMP/bs_trig \
									 $TCOMP/bd_trig \
									 $TCOMP/rx_trig_ack


add wave -noupdate -divider -height 20 "DUT settings"
add wave -label "Fixed bit stuffing method" $TCOMP/fixed_stuff
# Note that only one length is enough since both circuits are set to same length!
# The same with enable!
add wave -label "Length of bit stuffing/destuffing rule" -unsigned $TCOMP/bs_length
add wave -label "Bit stuffing/destuffing enabled" $TCOMP/bs_enable

add wave -noupdate -divider -height 20 "Bit Stuffing"
add wave -label "Pre bit stuffing" $TCOMP/tx_data
add wave -label "Post bit stuffing: DUT" $TCOMP/stuffed_data
add wave -label "Post bit stuffing: MODEL" $TCOMP/exp_stuffed
add wave -label "Bit stuffed: DUT" $TCOMP/data_halt
add wave -label "Bit stuffed: MODEL" $TCOMP/should_be_stuffed
add wave -label "Stuff count mod 8" -unsigned $TCOMP/bs_ctr

add wave -noupdate -divider -height 20 "Bit De-Stuffing"
add wave -label "Pre bit De-stuffing" $TCOMP/joined_data
add wave -label "Post bit De-stuffing" $TCOMP/rx_data
add wave -label "Inserted error data" -unsigned $TCOMP/err_data									 
add wave -label "Bit De-stuffed" $TCOMP/destuffed
add wave -label "Stuff error" $TCOMP/stuff_error
add wave -label "De-Stuff count mod 8" -unsigned $TCOMP/bd_ctr

add wave -noupdate -divider -height 20 "Testbench internals"
add wave -label "Step settings" -unsigned $TCOMP/set
add wave -label "NBS pointer" -unsigned $TCOMP/nbs_index
add wave -label "WBS pointer" -unsigned $TCOMP/wbs_index




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


