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
## 		Quartus TCL script for automation of core resource requirements.
##      Execute the script in Quartus project: Benchmark_project located
##      in synthesis/Quartus.
################################################################################

load_package flow
load_package report

set CONFIG_COUNT 7
set i 1
set results [list]

## Name of CAN_top_level entity parameters
 set PARAM_NAMES [ list "dummy" \
                                 "use_logger" \
							     "rx_buffer_size" \
							     "use_sync" \
							     "ID" \
							     "sup_filtA" \
							     "sup_filtB" \
							     "sup_filtC" \
							     "sup_range" \
							     "logger_size"
					     ]
							     
## List of synthesis configurations
 set CFG_LIST [ list          [ list "Minimal configuration" \
							        false 32 true 1\
						            false false false false 8
							  ] \
							  [ list "Medium configuration" \
							        false 256 true 1\
						            false false true true 8
							  ] \
							  [ list "Full configuration" \
							        false 4096 true 1\
						            true true true true 8
							  ] \
							  [ list "Full configuration + Small logger" \
							        true 4096 true 1\
						            true true true true 8
							  ] \
							  [ list "Full configuration + Big logger" \
							        true 4096 true 1\
						            true true true true 64
							  ] 
			]
			

foreach config $CFG_LIST {
   set act_cfg [lindex $config 0]
   puts "Configuration name: ${act_cfg}"
   
   # Set configuration to top level entity and compile
   foreach par_name $PARAM_NAMES par_val $config {
        set_parameter -entity "CAN_Wrapper" -name $par_name $par_val
   }
   execute_flow -compile
   
   # Load report and get the results
   load_report

	# Load timing Analysis of 85 ° Slow
	set panel_name {TimeQuest Timing Analyzer||Slow 1100mV 85C Model||Slow 1100mV 85C Model Fmax Summary}
	set panel_id    [get_report_panel_id $panel_name]
	set max_freq [get_report_panel_data -row 1 -col 0 -id $panel_id]

	# Load FPGA resource usage
	set aluts [get_fitter_resource_usage -alut -used]
	set aregs  [get_fitter_resource_usage -reg -used]
	set alms  [get_fitter_resource_usage -alm -used]
	set mbits [get_fitter_resource_usage -mem_bit -used]
    
	lappend results [list $act_cfg [list "LUTs" $aluts] \
									[list "REGs" $aregs] \
									[list "ALMs" $alms] \
									[list "Max. Freq" $max_freq] \
									[list "Mbits" $mbits]
					]

	foreach cfg_res $results {
		puts $cfg_res
	}
	unload_report
}  

puts " "
puts "CAN FD Benchmark results:"

foreach cfg_res $results {
	puts $cfg_res
}
