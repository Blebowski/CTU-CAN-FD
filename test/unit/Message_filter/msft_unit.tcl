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
## 			Message filter unit test handling script
################################################################################
global TCOMP

start_CAN_simulation "mess_filt_unit_test_wrapper"

################################################################################
# Adding the waves
################################################################################

#Add common waves for each test entity
add_test_status_waves
add_system_waves

#Add message filter specific components
add wave -noupdate -divider -height 20 "Stimuli inputs"
add wave -expand -height 17 -label "Input frame" $TCOMP/frame_info  
add wave -expand -height 17 -label "Filter settings" $TCOMP/drv_settings

#Circuit component			
set CCOMP "frame_filters_comp"  
add wave -noupdate -divider -height 20 "Circuit state"
add wave -label "Filter A valid" $TCOMP/$CCOMP/int_filter_a_valid
add wave -label "Filter B valid" $TCOMP/$CCOMP/int_filter_b_valid
add wave -label "Filter C valid" $TCOMP/$CCOMP/int_filter_c_valid
add wave -label "Filter Range valid" $TCOMP/$CCOMP/int_filter_ran_valid

add wave -noupdate -divider -height 20 "Bit Filter A"
add wave -label "Filter A mask" $TCOMP/$CCOMP/drv_filter_A_mask
add wave -label "Filter A bits" $TCOMP/$CCOMP/drv_filter_A_bits
add wave -label "Filter A enable" $TCOMP/$CCOMP/filter_A_enable

add wave -noupdate -divider -height 20 "Bit Filter B"
add wave -label "Filter B mask" $TCOMP/$CCOMP/drv_filter_B_mask
add wave -label "Filter B bits" $TCOMP/$CCOMP/drv_filter_B_bits
add wave -label "Filter B enable" $TCOMP/$CCOMP/filter_B_enable

add wave -noupdate -divider -height 20 "Bit Filter C"
add wave -label "Filter C mask" $TCOMP/$CCOMP/drv_filter_C_mask
add wave -label "Filter C bits" $TCOMP/$CCOMP/drv_filter_C_bits
add wave -label "Filter C enable" $TCOMP/$CCOMP/filter_C_enable

add wave -noupdate -divider -height 20 "Range Filter"
add wave -label "Filter Range High threshold" $TCOMP/$CCOMP/drv_filter_ran_hi_th
add wave -label "Filter Range Low threshold" $TCOMP/$CCOMP/drv_filter_ran_lo_th
add wave -label "Filter Range enable" $TCOMP/$CCOMP/filter_range_enable

add wave -noupdate -divider -height 20 "Circuit outputs"
add wave -label "Output valid" $TCOMP/out_ident_valid

################################################################################
# Execute the simulation, gather results
################################################################################
run_simulation
get_test_results
