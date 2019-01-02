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
## 			Interrupt manager unit test handling script
################################################################################
global TCOMP

start_CAN_simulation "int_man_test_wrapper"

################################################################################
# Adding the waves
################################################################################

#Add common waves for each test entity
add_test_status_waves
add_system_waves

#Add circuit specific signals
add wave -noupdate -divider -height 20 "DUT inputs (generated)"
add wave -group "Interrupt sources" $TCOMP/error_valid \
									 $TCOMP/error_passive_changed \
									 $TCOMP/error_warning_limit \
									 $TCOMP/arbitration_lost \
									 $TCOMP/tx_finished \
									 $TCOMP/br_shifted \
									 $TCOMP/rx_message_disc \
									 $TCOMP/rec_message_valid \
									 $TCOMP/rx_full \
									 $TCOMP/loger_finished \
									 $TCOMP/rx_empty \
									 $TCOMP/txt_hw_cmd_int

# Commands from user registers
add wave -label "Interrupt clear" $TCOMP/drv_int_clear;
add wave -label "Interrupt Enable set" $TCOMP/drv_int_ena_set;
add wave -label "Interrupt Enable clear" $TCOMP/drv_int_ena_clear;
add wave -label "Interrupt Mask set" $TCOMP/drv_int_mask_set;
add wave -label "Interrupt Mask clear" $TCOMP/drv_int_mask_clear;

# Interrupt manager outputs
add wave -noupdate -divider -height 20 "DUT outputs"
add wave -label "Interrupt output" $TCOMP/int_out
add wave -label "Interrupt vector" $TCOMP/int_vector
add wave -label "Interrupt Enable" $TCOMP/int_ena
add wave -label "Interrupt Mask" $TCOMP/int_mask

add wave -noupdate -divider -height 20 "Testbench internal signals"
add wave $TCOMP/int_status_exp
add wave $TCOMP/int_ena_exp
add wave $TCOMP/int_mask_exp

									 
################################################################################
# Execute the simulation, gather results
################################################################################
run_simulation
get_test_results

