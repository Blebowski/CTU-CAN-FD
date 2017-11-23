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
###############################################################################


##########################################################
#Adds the basic waveforms which are common for all tests
##########################################################
proc add_common_waves {} {
	add wave -noupdate -divider -height 20 "Test details"
	add wave status
	add wave run
	add wave errors
	add wave test_comp/iterations
	add wave test_comp/log_level
	add wave test_comp/error_beh
}
