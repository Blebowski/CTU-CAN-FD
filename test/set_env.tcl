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

