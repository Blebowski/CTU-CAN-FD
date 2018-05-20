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
	add wave status
	add wave run
	add wave errors
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
# Obtains the test results from the simulator, by
# extracting the value of status signals into global variables TEST_RESULT
# and ERROR_COUNT
################################################################################
proc get_test_results {} {
	global TEST_RESULT
	global ERROR_COUNT
	global TCOMP
	
	set TEST_RESULT [exa {/can_test_wrapper/status}]
	set ERROR_COUNT [exa {/can_test_wrapper/errors}]
}

################################################################################
# Obtains the test results from the simulator, by
# extracting the value of status signals into global variables
################################################################################
proc get_test_results_feature {} {
	global TEST_RESULT
	global ERROR_COUNT
	global TCOMP
	
	set TEST_RESULT [exa {/can_test_wrapper/status}]
	set ERROR_COUNT [exa {/can_test_wrapper/error_ctr}]
}

################################################################################
# Prints help into command line
################################################################################
proc print_help {} {	
	puts "CANTest framework has following commands available:"
	puts "	help			to print this help menu"
	puts "	exit			to exit the test framework"
	puts "	test <test> 	to run the tests"
	puts " "
	
	puts "CANTest framework contains following types of tests:"
	puts "	Unit tests"
	puts "	Feature tests"
	puts "	Sanity tests"
	puts " "
	
	puts "Unit tests can be used via following commands:"
	puts "	test unit <test_name>	To execute <test_name> unit test"
	puts "	test unit all           TO execute all existing unit tests"
	puts " "
	
	puts "Feature tests can be used via following commands:"
	puts "	test feature start			To start test environment of feature test"
	puts "	test feature run        	To start feature test execution"
	puts "	test feature print_config   To print configuration of feature test"
	puts " "
	
	puts "Sanity tests can be used via following commands:"
	puts "	test sanity start			To start test environment of sanity test"
	puts "	test sanity run        		To start sanity test execution"
	puts "	test sanity start silent    To start sanity test with only basic waveform "
	puts "								This option is recomended to save harddisk space"
	puts "								when running long sanity simulations (e.g more than 5 hours)"
	puts " "
	
	puts "Feature and sanity tests have separate configuration files (TCL) in "
	puts "their corresponding directories. See the CAN FD IP Core documentation"
	puts "to get the details about test configuration files"
	puts " "
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



################################################################################
################################################################################
#  Feature tests routines
################################################################################
################################################################################

################################################################################
# Starts simulation of feature test environment
################################################################################
proc start_feature_FIFO {} {
	global BASE_TEST
	do "${BASE_TEST}/feature/feature_env_setup.tcl"
}

################################################################################
# Runs the feature test simulation.
# Feature config script is executed and FEATURE_FIFO is loaded.
# In loop each config is executed and FEATURE_FIFO is sliced until all tests
# are executed
################################################################################
proc run_feature_FIFO {} {
	global FEATURE_FIFO
	global TCOMP
	global TEST_RESULT
	global ERROR_COUNT
	global BASE_TEST
	
	set feat_res []
	
	#Run the config script which sets up the feature list
	#(ORIGINAL Feature FIFO)
	do "${BASE_TEST}/feature/feature_config.tcl"
	
	#Iterate over FIFO entries
	while { [llength $FEATURE_FIFO] > 0 } {
		set actual [lindex $FEATURE_FIFO 0] 
		set act_name [lindex $actual 0]
		set act_iter [lindex $actual 1]
		
		#Set the testbench stimulus
		force $TCOMP/iterations $act_iter	
		set fmt_name [format "%20s" $act_name]
		force $TCOMP/test_name "${fmt_name}"
		
		#Run the testbench and fetch results
		run_simulation
		get_test_results_feature
		
		#Erase the actual test from FIFO
		set FEATURE_FIFO [lreplace $FEATURE_FIFO 0 0]
		
		#TODO: Here gather the result of the test!
		lappend feat_res [list $act_name $act_iter $ERROR_COUNT $TEST_RESULT]
	}
	
	puts "Feature test run results:"
	foreach actual $feat_res {
		puts $actual
	} 
}

proc show_feature_FIFO {} {
	global BASE_TEST
	global FEATURE_FIFO
	
	do "${BASE_TEST}/feature/feature_config.tcl"
	puts $FEATURE_FIFO
}


################################################################################
################################################################################
#  Sanity test routines
################################################################################
################################################################################

################################################################################
# Set the bus_matrix signal of sanity test environment
################################################################################
proc force_bm {a b c d e f g h i j k l m n o p} {
	global TCOMP
	
	force $TCOMP/bus_matrix(1,1) $a
	force $TCOMP/bus_matrix(1,2) $b
	force $TCOMP/bus_matrix(1,3) $c
	force $TCOMP/bus_matrix(1,4) $d
	
	force $TCOMP/bus_matrix(2,1) $e
	force $TCOMP/bus_matrix(2,2) $f
	force $TCOMP/bus_matrix(2,3) $g
	force $TCOMP/bus_matrix(2,4) $h
	
	force $TCOMP/bus_matrix(3,1) $i
	force $TCOMP/bus_matrix(3,2) $j
	force $TCOMP/bus_matrix(3,3) $k
	force $TCOMP/bus_matrix(3,4) $l
	
	force $TCOMP/bus_matrix(4,1) $m
	force $TCOMP/bus_matrix(4,2) $n
	force $TCOMP/bus_matrix(4,3) $o
	force $TCOMP/bus_matrix(4,4) $p
}

################################################################################
# Start sanity test simulation
################################################################################
proc start_sanity {} {
	global BASE_TEST
	do "${BASE_TEST}/sanity/sanity_env_setup.tcl"
}

################################################################################
# Run sanity test
# First SAN_CFG is loaded and each configuration is executed until all configs
# are exectued.
################################################################################
proc run_sanity {} {
	global SAN_CFG
	global TCOMP
	global TEST_RESULT
	global ERROR_COUNT
	global BASE_TEST
	
	set san_res []
	
	#Run the config script which sets up the list with sanity
	# configurations
	do "${BASE_TEST}/sanity/sanity_config.tcl"
	
	#Iterate over the configurations
	while { [llength $SAN_CFG] > 0 } {
		set actual [lindex $SAN_CFG 0]
		
		##################################
		#Aliases for configuration values
		##################################
		set topology [lindex $actual 0]
		
		#Bus lengths configuration
		set l1 [lindex $actual 1]
		set l2 [lindex $actual 2]
		set l3 [lindex $actual 3]
		set l4 [lindex $actual 4]
		set l5 [lindex $actual 5]
		set l6 [lindex $actual 6]
		
		#Transciever delay
		set del [list [lindex $actual 7] [lindex $actual 8]\
						[lindex $actual 9] [lindex $actual 10]]
		
		#Oscillator tollerance
		set osc [list [lindex $actual 11] [lindex $actual 12]\
						[lindex $actual 13] [lindex $actual 14]]
		
		#Noise parameters
		set nw_mean [lindex $actual 15]
		set nw_var [lindex $actual 16]
		set ng_mean [lindex $actual 17]
		set ng_var [lindex $actual 18]
		
		#Bit time config
		set brp_nbt [lindex $actual 19]
		set brp_dbt [lindex $actual 20]
		set prop_nbt [lindex $actual 21]
		set ph1_nbt [lindex $actual 22]
		set ph2_nbt [lindex $actual 23]
		set sjw_nbt [lindex $actual 24]
		set prop_dbt [lindex $actual 25]
		set ph1_dbt [lindex $actual 26]
		set ph2_dbt [lindex $actual 27]
		set sjw_dbt [lindex $actual 28]
		
		#Name and iterations amount
		set act_name [lindex $actual 29]
		set act_iter [lindex $actual 30]
		
		##################################
		# Now force the values to the test
		##################################
		
		#Bus matrix based on topology
		if { $topology == "bus"} {
			force_bm 0 $l1 [expr "${l1}+${l2}"] [expr "${l1}+${l2}+${l3}"]\
					$l1	0 $l2 [expr "${l2}+${l3}"]\
					[expr "${l1}+${l2}"] $l2 0 $l3\
					[expr "${l1}+${l2}+${l3}"] [expr "${l2}+${l3}"] $l3 0
		} elseif { $topology == "star" } {
			force_bm 0 [expr "${l1}+${l2}"] [expr "${l1}+${l3}"] [expr "${l1}+${l4}"]\
					[expr "${l1}+${l2}"] 0 [expr "${l2}+${l3}"] [expr "${l2}+${l4}"]\
					[expr "${l1}+${l3}"] [expr "${l2}+${l3}"] 0 [expr "${l3}+${l4}"]\
					[expr "${l1}+${l4}"] [expr "${l2}+${l4}"] [expr "${l3}+${l4}"] 0
		} elseif { $topology == "tree" } {
			force_bm 0 [expr "${l1}+${l2}"] [expr "${l1}+${l3}+${l5}"] [expr "${l1}+${l4}+${l5}"]\
					[expr "${l1}+${l2}"] 0 [expr "${l2}+${l3}+${l5}"] [expr "${l2}+${l4}+${l5}"]\
					[expr "${l1}+${l3}+${l5}"] [expr "${l2}+${l3}+${l5}"] 0 [expr "${l3}+${l4}"]\
					[expr "${l1}+${l4}+${l5}"] [expr "${l2}+${l4}+${l5}"] [expr "${l3}+${l4}"] 0
		} elseif { $topology == "ring" } {
			# TODO: Ring topology with fucking min functions
		} elseif { $topology == "custom" } {
			force_bm 0 $l1 $l2 $l3
					$l1 0 $l4 $l5
					$l2 $l4 0 $l6
					$l3 $l6 $l6 0
		} else {
			puts "Invalid bus topology! Assuming bus topology..."
			force_bm 0 $l1 [expr "${l1}+${l2}"] [expr "${l1}+${l2}+${l3}"]\
					$l1	0 $l2 [expr "${l2}+${l3}"]\
					[expr "${l1}+${l2}"] $l2 0 $l3\
					[expr "${l1}+${l2}+${l3}"] [expr "${l2}+${l3}"] $l3 0
		}
		
		#Set the testbench stimulus
		force $TCOMP/iterations $act_iter	
		set fmt_name [format "%-50s" $act_name]
		force $TCOMP/i_st/test_desc "${fmt_name}"
		
		set fmt_topology [format "%-50s" $topology]
		force $TCOMP/topology "${fmt_topology}"
		
		#Transciever delays
		set index 0
		foreach elem $del {
			incr index
			force $TCOMP/trv_del_v($index) $elem
		}
		
		#Oscilator tolerances
		set index 0
		foreach elem $osc {
			incr index
			force $TCOMP/epsilon_v($index) $elem
		}
		
		#Noise parameters
		force $TCOMP/nw_mean $nw_mean
		force $TCOMP/nw_var $nw_var
		force $TCOMP/ng_mean $ng_mean
		force $TCOMP/ng_var $ng_var
		
		#Bit time settings
		force $TCOMP/timing_config.tq_nbt $brp_nbt
		force $TCOMP/timing_config.tq_dbt $brp_dbt
		force $TCOMP/timing_config.prop_nbt $prop_nbt
		force $TCOMP/timing_config.ph1_nbt $ph1_nbt
		force $TCOMP/timing_config.ph2_nbt $ph2_nbt
		force $TCOMP/timing_config.sjw_nbt $sjw_nbt
		force $TCOMP/timing_config.prop_dbt $prop_dbt
		force $TCOMP/timing_config.ph1_dbt $ph1_dbt
		force $TCOMP/timing_config.ph2_dbt $ph2_dbt
		force $TCOMP/timing_config.sjw_dbt $sjw_dbt
		
		#Run the testbench and fetch results
		run_simulation
		get_test_results_feature
		
		#Erase the actual test from test config
		set SAN_CFG [lreplace $SAN_CFG 0 0]
		
		#TODO: Here gather the result of the test!
		lappend san_res [list $act_name "Iterations:" $act_iter "Errors:" $ERROR_COUNT "Result:" $TEST_RESULT]	
	}
	
	puts "Sanity test run results:"
	foreach actual $san_res {
		puts $actual
	} 
}





