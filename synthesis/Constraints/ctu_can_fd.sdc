################################################################################
## 
## CTU CAN FD IP Core 
## Copyright (C) 2021-present Ondrej Ille
## 
## Permission is hereby granted, free of charge, to any person obtaining a copy
## of this VHDL component and associated documentation files (the "Component"),
## to use, copy, modify, merge, publish, distribute the Component for
## educational, research, evaluation, self-interest purposes. Using the
## Component for commercial purposes is forbidden unless previously agreed with
## Copyright holder.
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
## -------------------------------------------------------------------------------
## 
## CTU CAN FD IP Core 
## Copyright (C) 2015-2020 MIT License
## 
## Authors:
##     Ondrej Ille <ondrej.ille@gmail.com>
##     Martin Jerabek <martin.jerabek01@gmail.com>
## 
## Project advisors: 
## 	Jiri Novak <jnovak@fel.cvut.cz>
## 	Pavel Pisa <pisa@cmp.felk.cvut.cz>
## 
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

set SYS_CLK_PERIOD 10.0

# Basic clock for whole system
create_clock -period $SYS_CLK_PERIOD -name SYS_CLK -waveform { 0.000 5.000 } [get_ports *clk*]

# Asynchronous input from CAN bus
set_false_path -from [get_ports can_rx]

# Reset is defined as asynchronous, it is internally synced!
set_false_path -from [get_ports res_n]

# Only for ASICs, synchronous to clk_sys
set_input_delay -clock SYS_CLK [expr $SYS_CLK_PERIOD/2] [get_ports scan_enable]

# This is for TB only, in design this can be left unconnected!
set_false_path -to [get_ports test_probe*]

# Set IO delays to half of cycle to get realistic logic inside of CTU CAN FD
set_input_delay -clock SYS_CLK [expr $SYS_CLK_PERIOD/2] [get_ports timestamp*]
set_input_delay -clock SYS_CLK [expr $SYS_CLK_PERIOD/2] [get_ports data_in*]
set_input_delay -clock SYS_CLK [expr $SYS_CLK_PERIOD/2] [get_ports adress*]
set_input_delay -clock SYS_CLK [expr $SYS_CLK_PERIOD/2] [get_ports scs]
set_input_delay -clock SYS_CLK [expr $SYS_CLK_PERIOD/2] [get_ports swr]
set_input_delay -clock SYS_CLK [expr $SYS_CLK_PERIOD/2] [get_ports srd]
set_input_delay -clock SYS_CLK [expr $SYS_CLK_PERIOD/2] [get_ports sbe*]

set_output_delay -clock SYS_CLK [expr $SYS_CLK_PERIOD/2] [get_ports data_out*]
set_output_delay -clock SYS_CLK [expr $SYS_CLK_PERIOD/2] [get_ports can_tx]
set_output_delay -clock SYS_CLK [expr $SYS_CLK_PERIOD/2] [get_ports int]
set_output_delay -clock SYS_CLK [expr $SYS_CLK_PERIOD/2] [get_ports res_n_out]


