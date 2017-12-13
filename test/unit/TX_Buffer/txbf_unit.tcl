################################################################################
##
## CAN with Flexible Data-Rate IP Core 
##
## Copyright (C) 2015 Ondrej Ille <ondrej.ille@gmail.com>
##
## Permission is hereby granted, free of charge, to any person obtaining a copy 
## of this software and associated documentation files (the "Software"), to deal
## in the Software without restriction, including without limitation the rights
## to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
## copies of the Software, and to permit persons to whom the Software is 
## furnished to do so, subject to the following conditions:
##
## The above copyright notice and this permission notice shall be included in 
## all copies or substantial portions of the Software.
##
## THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
## IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
## FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
## AUTHORS OR COPYRIGHTHOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
## LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
## FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
## IN THE SOFTWARE.
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
add wave -label "Common Data input" -hexadecimal $TCOMP/tran_data_in

add wave -noupdate -divider -height 20 "DUT signals - Buffer 1"	
add wave -label "Buffer empty" -hexadecimal $TCOMP/txt_empty_1							
add wave -label "Discard content" $TCOMP/txt_disc_1	
add wave -label "Buffer output" -hexadecimal $TCOMP/txt_buffer_out_1
add wave -label "Discard content" $TCOMP/txt_disc_1
add wave -label "Store data" $TCOMP/drv_store_txt_1
add wave -label "Supports FD size" $TCOMP/txt_buf_comp_1/useFDsize

add wave -noupdate -divider -height 20 "DUT signals - Buffer 2"
add wave -label "Buffer empty" -hexadecimal $TCOMP/txt_empty_2							
add wave -label "Discard content" $TCOMP/txt_disc_2	
add wave -label "Buffer output" -hexadecimal $TCOMP/txt_buffer_out_2
add wave -label "Discard content" $TCOMP/txt_disc_2
add wave -label "Store data" $TCOMP/drv_store_txt_2
add wave -label "Supports FD size" $TCOMP/txt_buf_comp_2/useFDsize

add wave -noupdate -divider -height 20 "Internal testbench signals"								
add wave -label "Expected output Buffer 1" -hexadecimal $TCOMP/small_mem
add wave -label "Expected output Buffer 2" -hexadecimal $TCOMP/big_mem
									 
################################################################################
# Execute the simulation, gather results
################################################################################
run_simulation
get_test_results


