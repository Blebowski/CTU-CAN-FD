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
					   