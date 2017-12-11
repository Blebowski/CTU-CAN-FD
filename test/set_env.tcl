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

