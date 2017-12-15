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
## 			Bit stuffing and destuffing unit test handling script
################################################################################
global TCOMP

start_CAN_simulation "bus_sync_test_wrapper"

################################################################################
# Adding the waves
################################################################################

#Add common waves for each test entity
add_test_status_waves
add_system_waves

#Add circuit specific signals
add wave -noupdate -divider -height 20 "DUT inputs (generated)"
add wave -label "Sample (NOMINAL,Data)" $TCOMP/sample_nbt
add wave -label "Sync" $TCOMP/tx_trig
add wave -label "Sample type" $TCOMP/sp_control
add wave -label "Start transciever delay measurment" $TCOMP/trv_delay_calib
add wave -label "SYNC+PROP+PH1" -unsigned $TCOMP/seg1
add wave -label "PH2" -unsigned $TCOMP/seg2

add wave -noupdate -divider -height 20 "DUT outputs"
add wave -label "Synchronization edge" $TCOMP/sync_edge
add wave -label "Secondary sample point" $TCOMP/sample_sec_out
add wave -label "Transciever delay (measured)" -unsigned $TCOMP/trv_delay_out
add wave -label "Bit error" $TCOMP/bit_error

add wave -noupdate -divider -height 20 "DUT internal signals"
add wave -label "Tripple sampling majority" $TCOMP/bus_sync_comp/trs_majority
add wave -label "Valid R->D edge on RX" $TCOMP/bus_sync_comp/edge_rx_det
add wave -label "Valid R->D edge on TX" $TCOMP/bus_sync_comp/edge_tx_det
add wave -label "Delay measurment running" $TCOMP/bus_sync_comp/trv_running
add wave $TCOMP/bus_sync_comp/trv_to_restart

add wave -noupdate -divider -height 20 "Internal testbench signals"								
add wave -label "Transciever delay (real)" -unsigned $TCOMP/tran_del
add wave -label "Delay simulation shift register" -unsigned $TCOMP/tran_del_sr
add wave -label "Only recessive now genetrated on TX" $TCOMP/generate_ones
												 
################################################################################
# Execute the simulation, gather results
################################################################################
run_simulation
get_test_results


