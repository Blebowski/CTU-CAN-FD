################################################################################
## Author:      Ondrej Ille , Czech Technical University, FEL
## Project:     CAN FD IP Core Project
##
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
set ITERATIONS 25000
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

