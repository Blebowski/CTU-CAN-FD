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
add wave -expand -height 17 $TCOMP/frame_info  
add wave -expand -height 17 $TCOMP/drv_settings

#Circuit component			
set CCOMP "messageFilter_comp"  
add wave -noupdate -divider -height 20 "Circuit state"
add wave $TCOMP/$CCOMP/int_filter_a_valid
add wave $TCOMP/$CCOMP/int_filter_b_valid
add wave $TCOMP/$CCOMP/int_filter_c_valid
add wave $TCOMP/$CCOMP/int_filter_ran_valid

add wave -noupdate -divider -height 20 "Circuit outputs"
add wave $TCOMP/out_ident_valid

################################################################################
# Execute the simulation, gather results
################################################################################
run_simulation
get_test_results
