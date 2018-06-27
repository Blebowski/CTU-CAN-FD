################################################################################
## Author:      Ondrej Ille , Czech Technical University, FEL
## Project:     CAN FD IP Core Project
##
## 			CAN FD IP Core testbench TCL framework for automatic 
##			test execution
##
################################################################################

puts "----------------------------------"
puts "--Starting CANTest TCL framework--"
puts "----------------------------------"

#Check if environment variables are existing
quietly set exist_var [info exist ITERATIONS]
quietly set BASE_DIR "../CAN_FD_Leto2015/"
quietly set BASE_TEST "../CAN_FD_Leto2015/Testing_scripts"

# Create the environment if not yet existant
if { $exist_var == 0 } {
	puts "Enviroment variables not found -> Setting up environment"
	puts ""
	do [file join $BASE_DIR Testing_scripts/set_env.tcl]
}

#Include the library functions
source [file join $BASE_DIR Testing_scripts/lib/test_lib.tcl]

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
