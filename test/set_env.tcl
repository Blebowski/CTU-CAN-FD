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
## 			Environment variables initialization script
################################################################################

################################################################################
#Test component
#Each separate unit test component should be called test component
################################################################################
quietly set TCOMP "test_comp"

################################################################################
# Configures the generic parameters for CAN test wrapper entity
# Temporary definitions will be moved to the calling script
################################################################################
puts "Setting up default test parameters"
puts "Number of iterations:"
set ITERATIONS 200
puts ""
puts "Test behaviour during error:"
set ERR_BEH  go_on
puts ""
puts "Logging level:"
set LOG_LEVEL warning_l
puts ""
puts "Error tolerance:"
set ERR_TOL  0
puts ""

################################################################################
# Default break behaviour
# When multiple test sequences are executed automatically then following
# variable has to be set to FALSE
################################################################################
puts "Stall script exuction at the end of test:"
set WAIT_ON_END false

################################################################################
# Introduce the global variables for test results
################################################################################
quietly set TEST_RESULT ""
quietly set ERROR_COUNT 0


################################################################################
# Introduce the global variables for test results
################################################################################
quietly set FEATURE_FIFO []

