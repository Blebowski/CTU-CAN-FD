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


#*******************************************************************************
# Xilinx constraints
#*******************************************************************************

# Create 100 MHz clock
create_clock -name {clk_sys} -period 10.000 -waveform { 0.000 5.000 } [get_nets {clk_sys}]

# Reset synchroniser attirbutes
set rst_sync_chain_1 [get_cells {*rst_sync_comp/rst_n*}]
set rst_sync_chain_2 [get_cells {*rst_sync_comp/rff*}]

set_property ASYNC_REG true $rs_sync_chain_1
set_property ASYNC_REG true $rs_sync_chain_2


# CAN RX line synchroniser attirbutes
set CAN_rx_chain_1 [get_cells {*bus_sync_comp/sync_Chain_1*}]
set CAN_rx_chain_2 [get_cells {*bus_sync_comp/sync_Chain_2*}]

set_property ASYNC_REG true $CAN_rx_chain_1
set_property ASYNC_REG true $CAN_rx_chain_2

