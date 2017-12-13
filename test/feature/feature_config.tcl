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
## 			Feature test configuration file
################################################################################

global FEATURE_FIFO

#quietly set FEATURE_FIFO [list [list forbid_fd 50] \
					   [list abort_transmittion 50] \
					   [list rtr_pref 25] \
					   [list tx_arb_time_tran 100] \
					   [list traf_measure 80] \
					   [list spec_mode 50] \
					   [list soft_reset 5] \
					   [list trv_delay 50] \
					   [list invalid_configs 30] \
					   [list retr_limit 60] \
					   [list overload 20] \
					   [list traf_measure 25]

quietly set FEATURE_FIFO [list [list trv_delay 10]]
				 
#[list interrupt 10] \
					   