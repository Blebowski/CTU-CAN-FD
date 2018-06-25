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
## 			Feature environment waveform setup script
################################################################################
global TCOMP

################################################################################
# Adding the waves
################################################################################

#Add common waves for each test entity
#add_test_status_waves
#add_system_waves

#TODO: Will be replace by calls above. So far disabled to allow manual debug!
add wave run
add wave loop_ctr
add wave errors
add wave status

add wave -noupdate -divider -height 20 "System signals"
add wave -label "Clock" clk_sys
add wave -label "Timestamp" timestamp

add wave -label "Memory bus" mem_bus

add wave -noupdate -divider -height 20 "Bit generator"
add wave -label "Run" bit_gen_run
add wave -label "Done" bit_gen_done
add wave -label "Sequence" bit_sequence
add wave -label "Tick" bit_gen_comp/tick
add wave -label "Counter" bit_gen_comp/counter
add wave -label "Index" bit_gen_comp/bit_gen/index

add wave -noupdate -divider -height 20 "CAN Bus"
add wave -label "Generator output" bit_gen_out
add wave -label "CAN TX" can_tx
add wave -label "CAN RX" can_rx

add wave -noupdate -divider -height 20 "CAN Node"
add wave -label "Protocol control" CAN_inst/core_top_comp/PC_State
add wave -label "Bit timing state" CAN_inst/bt_FSM_out


