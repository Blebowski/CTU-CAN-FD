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

start_CAN_simulation "Fault_confinement_unit_test_wrapper"

################################################################################
# Adding the waves
################################################################################

# Add common waves for each test entity
add_test_status_waves
add_system_waves

# Driving bus signals
add wave -decimal -label "Error warning limit" $TCOMP/drv_ewl
add wave -decimal -label "Error passive" $TCOMP/drv_erp
add wave -decimal -label "Counter preset value" $TCOMP/drv_ctr_val
add wave -label "Counter preset target" $TCOMP/drv_ctr_sel

# Increment / Decrement error counters!
add wave -label "Increment one" $TCOMP/inc_one
add wave -label "Increment eight" $TCOMP/inc_eight
add wave -label "Decrement one" $TCOMP/dec_one

# Add error counter related values
add wave -noupdate -divider -height 20 "Error (Fault confinement) counters"
add wave -decimal -label "TX Error counter" $TCOMP/tx_counter_out
add wave -decimal -label "RX Error counter" $TCOMP/rx_counter_out
add wave -decimal -label "Nominal Error counter" $TCOMP/err_counter_norm_out
add wave -decimal -label "FD Error counter" $TCOMP/err_counter_fd_out
add wave -decimal -label "Fault conf. state" $TCOMP/error_state_out

# Testbench internal signals
add wave -noupdate -divider -height 20 "Testbench modeled counters"
add wave -label "TX Error counter" $TCOMP/tx_err_model
add wave -label "RX Error counter" $TCOMP/rx_err_model
add wave -label "Nominal Error counter" $TCOMP/norm_err_model
add wave -label "FD Error counter" $TCOMP/fd_err_model
add wave -decimal -label "Fault conf. state" $TCOMP/fc_model

									 
################################################################################
# Execute the simulation, gather results
################################################################################
run_simulation
get_test_results


