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
add_test_status_waves
add_system_waves

add wave -noupdate -divider -height 20 "System signals"
add wave -label "Clock" $TCOMP/clk_sys
add wave -label "Timestamp" $TCOMP/timestamp

add wave -label "Memory bus" $TCOMP/mem_bus

add wave -noupdate -divider -height 20 "Bit generator"
add wave -label "Run" $TCOMP/bit_gen_run
add wave -label "Done" $TCOMP/bit_gen_done
add wave -label "Sequence" $TCOMP/bit_sequence
add wave -label "Tick" $TCOMP/bit_gen_comp/tick
add wave -label "Counter" $TCOMP/bit_gen_comp/counter
add wave -label "Index" $TCOMP/bit_gen_comp/bit_gen/index

add wave -noupdate -divider -height 20 "CAN Bus"
add wave -label "Generator output" $TCOMP/bit_gen_out
add wave -label "CAN TX" $TCOMP/can_tx
add wave -label "CAN RX" $TCOMP/can_rx

add wave -noupdate -divider -height 20 "CAN Node"
add wave -label "Protocol control" $TCOMP/CAN_inst/core_top_comp/PC_State
add wave -label "Bit timing state" $TCOMP/CAN_inst/bt_FSM_out


