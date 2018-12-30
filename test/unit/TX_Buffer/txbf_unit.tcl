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
## 	Bit stuffing and destuffing unit test handling script
################################################################################
global TCOMP

start_CAN_simulation "tx_buf_unit_test_wrapper"

################################################################################
# Adding the waves
################################################################################

#Add common waves for each test entity
add_test_status_waves
add_system_waves

#Add circuit specific signals
add wave -noupdate -divider -height 20 "DUT memory access"
add wave -label "Data input" -hexadecimal $TCOMP/tran_data
add wave -label "Address input" -hexadecimal $TCOMP/tran_addr
add wave -label "Chip select" $TCOMP/tran_cs

add wave -noupdate -divider -height 20 "DUT control signals"
add wave -label "SW commands" $TCOMP/txt_sw_cmd			
add wave -label "SW command index" $TCOMP/txt_sw_buf_cmd_index
add wave -label "HW commands" $TCOMP/txt_hw_cmd

add wave -noupdate -divider -height 20 "DUT status signals"
add wave -label "Buffer state" $TCOMP/txtb_state
add wave -label "Buffer ready" $TCOMP/txt_buf_ready
add wave -label "Buffer output" -hexadecimal $TCOMP/txt_word
add wave -label "Buffer adress from CAN Core" $TCOMP/txt_addr

add wave -label "TX Buffer memory" $TCOMP/txt_buffer_comp/txt_buf_RAM/ram_memory

add wave -label "TX Buffer FSM" $TCOMP/txt_buffer_comp/txt_buffer_fsm_comp/buf_fsm

add wave -noupdate -divider -height 20 "Internal testbench signals"
add wave -label "Shadow memory" -hexadecimal $TCOMP/shadow_mem
									 
################################################################################
# Execute the simulation, gather results
################################################################################
run_simulation
get_test_results


