################################################################################
##
## CAN with Flexible Data-Rate IP Core 
##
## Copyright (C) 2015 Ondrej Ille <ondrej.ille@gmail.com>
##
## This program is free software; you can redistribute it and/or
## modify it under the terms of the GNU General Public License
## as published by the Free Software Foundation; either version 2
## of the License, or (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## The CAN protocol is developed by Robert Bosch GmbH and     
## protected by patents. Anybody who wants to implement this    
## IP core on silicon has to obtain a CAN protocol license
## from Bosch.
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
