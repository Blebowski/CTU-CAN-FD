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
## 			CAN FD IP Core testbench TCL framework for automatic 
##			test execution
##
################################################################################

puts "----------------------------------"
puts "--Starting CANTest TCL framework--"
puts "----------------------------------"

#Check if environment variables are existing
quietly set exist_var [info exist ITERATIONS]

# IP Core standalone relative location
quietly set BASE_DIR "../"
quietly set BASE_TEST "../test"

# Test platform relative location
#quietly set BASE_DIR "../../../CAN_FD_IP_Core/"
#quietly set BASE_TEST "../../../CAN_FD_IP_Core/test"

# Create the environment if not yet existant
if { $exist_var == 0 } {
	puts "Enviroment variables not found -> Setting up environment"
	puts ""
	do [file join $BASE_TEST set_env.tcl]
}

#Include the library functions
source [file join $BASE_TEST lib/test_lib.tcl]

puts ""
puts "Welcome in CAN FD IP Core TCL test framework"
puts "use: 'help' command to obtain list of available commands"
puts ""

quietly set FRAMEWORK_QUIT false

# Test parser loop
while { $FRAMEWORK_QUIT == false } {
	set arg1 ""
	set arg2 ""
	set arg3 ""
	set arg4 ""
	set arg5 ""
	scan [gets stdin] "%s %s %s %s %s" arg1 arg2 arg3 arg4 arg5
	
	if { $arg1 == "exit" } {
		quietly set FRAMEWORK_QUIT true
	} elseif { $arg1 == "help" } {
		print_help
	} elseif { $arg1 == "test" } {
		if { $arg2 == "unit" } {
			if { $arg3 == "all" } {
				exec_all_TCL_from_path [file join $BASE_TEST unit ]
			} else {
				exec_TCL_from_path [file join $BASE_TEST unit $arg3]
			}
		} elseif { $arg2 == "sanity" } {
			if { $arg3 == "run" } {	
				run_sanity
			} elseif { $arg3 == "start" } {
				
				quietly set SILENT_SANITY "false"
				if { $arg4 == "silent" } {
					quietly set SILENT_SANITY "true" 
				}
				start_sanity
			} else {
				puts "Unknown command! Type: 'help' to obtain list of commands!"
			}	
		} elseif { $arg2 == "feature" } {
			if { $arg3 == "start" } {	
				start_feature_FIFO
			} elseif { $arg3 == "run" } {
				run_feature_FIFO
			} elseif { $arg3 == "print_config" } {
				show_feature_FIFO
			} else {
				puts "Unknown command! Type: 'help' to obtain list of commands!"
			}
		} else {
			puts "Unknown command! Type: 'help' to obtain list of commands!"
		}
	} else {
	  puts "Unknown command! Type: 'help' to obtain list of commands!"
	}
}
