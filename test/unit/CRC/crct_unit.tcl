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
add wave -hexadecimal $TCOMP/can_crc_comp/crc15_pol
add wave -hexadecimal $TCOMP/can_crc_comp/crc17_pol
add wave -hexadecimal $TCOMP/can_crc_comp/crc21_pol

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
add wave $TCOMP/can_crc_comp/crc_calc_15_comp/crc_nxt
add wave $TCOMP/can_crc_comp/crc_calc_15_comp/crc_shift
add wave $TCOMP/can_crc_comp/crc_calc_15_comp/crc_shift_n_xor
add wave $TCOMP/can_crc_comp/crc_calc_15_comp/crc_nxt_val

add wave $TCOMP/can_crc_comp/crc_calc_17_comp/crc_nxt
add wave $TCOMP/can_crc_comp/crc_calc_17_comp/crc_shift
add wave $TCOMP/can_crc_comp/crc_calc_17_comp/crc_shift_n_xor
add wave $TCOMP/can_crc_comp/crc_calc_17_comp/crc_nxt_val

add wave $TCOMP/can_crc_comp/crc_calc_21_comp/crc_nxt
add wave $TCOMP/can_crc_comp/crc_calc_21_comp/crc_shift
add wave $TCOMP/can_crc_comp/crc_calc_21_comp/crc_shift_n_xor
add wave $TCOMP/can_crc_comp/crc_calc_21_comp/crc_nxt_val

################################################################################
# Execute the simulation, gather results
################################################################################
run_simulation
get_test_results


