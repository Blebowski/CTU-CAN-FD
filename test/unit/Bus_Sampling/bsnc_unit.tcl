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


