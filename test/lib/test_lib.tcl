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
## 			Test library for CAN IP Core TCL functions
################################################################################


################################################################################
#Adds the basic waveforms which are common for all tests
################################################################################
proc add_test_status_waves {} {
	#Import used variable
	global TCOMP
	
	add wave -noupdate -divider -height 20 "Test details"
	#add wave status
	#add wave run
	#add wave errors
	add wave $TCOMP/iterations
	add wave $TCOMP/log_level
	add wave $TCOMP/error_beh
	add wave $TCOMP/loop_ctr
}

################################################################################
#Adds the Async reset signal and system clock
################################################################################
proc add_system_waves {} {
	global TCOMP
	
	add wave -noupdate -divider -height 20 System
	add wave $TCOMP/res_n
	add wave $TCOMP/clk_sys
}

################################################################################
# Starts the CAN simulation
# Entity can_test_wrapper is simulated. Arhitecture for simulation is given
# as parameter
################################################################################
proc start_CAN_simulation {test_wrapper} {
	#Import global variables
	global ERR_BEH
	global ITERATIONS
	global LOG_LEVEL
	global ERR_TOL
	
	#Quit simulation if any simulation is running
	quit -sim
       
	#Start the new entity simulation
	vsim -t fs -noglitch -gerror_beh=$ERR_BEH -giterations=$ITERATIONS \
		 -glog_level=$LOG_LEVEL -gerror_tol=$ERR_TOL \
		 work.can_test_wrapper($test_wrapper)
		 
	restart -f
	delete wave -r *
}

################################################################################
# Run the simulation
# after the end of simulation waits if WAIT_ON_END
# is 'true' otherwise it executes the script further
################################################################################
proc run_simulation {} {
	#Import global variables
	global WAIT_ON_END
	global NumericStdNoWarnings
	
	# Set what happedns when simulation is interrupted
	if	{$WAIT_ON_END == true} {
		onbreak { 
		}
	} else {
		onbreak { resume }
	}

	# Avoid warnings at time 0
	set NumericStdNoWarnings 1
	run 0 ns
	set NumericStdNoWarnings 0	

	#Run the simulation
	run -all
}


################################################################################
# Executes first TCL script it finds in given directory.
# Given path must be a directory
################################################################################
proc exec_TCL_from_path {path} {
	if { [file exists $path] && [file isdir $path] } {
		
		#Find TCL script in the directory
		set test_script [ glob -directory $path *.tcl ]
		
		#Execute the test if existing
		if { [info exists test_script] } {
			do $test_script
		} else {
			puts "Given directory does not contain any TCL script!"
			puts "To run the test create TCL script which executes it!"
		}
	} else {
		puts "Directory ${path} does not exist!"
		puts "Please type in valid directory name which contains TCL script!"
	}
}


################################################################################
# Executes all TCL scripts in all subfolders of given folder
# This proc is used for execution of all tests of given type 
# (e.g. test unit all)
################################################################################
proc exec_all_TCL_from_path {path} {
	
	#Find all subdirectories of this directory
	set test_dir_list [ glob -types d -directory $path *]
	
	#Import the globals
	global WAIT_ON_END
	global TEST_RESULT
	global ERROR_COUNT
	
	#More test will be run, we cant break after one!
	set WAIT_ON_END false
	
	set report_list []
	
	#Iterate trough all the tests
	foreach test $test_dir_list {
		exec_TCL_from_path $test
		
		#Store the report part
		set test_name [lindex [file split $test] end]
		lappend report_list "Test ${test_name}, ${ERROR_COUNT} errors, test ${TEST_RESULT}"
	}
	
	#Now print the results
	puts "Result summary of all test run from:"
	puts "$path"
	puts ""
	foreach test_res $report_list {
		puts $test_res
	}
	puts ""
}







