################################################################################
## Author:      Ondrej Ille , Czech Technical University, FEL
## Project:     CAN FD IP Core Project
##
## 			Message filter unit test handling script
################################################################################
global TCOMP

start_CAN_simulation "mess_filt_unit_test_wrapper"

################################################################################
# Adding the waves
################################################################################

#Add common waves for each test entity
add_test_status_waves
add_system_waves

#Add message filter specific components
add wave -noupdate -divider -height 20 "Stimuli inputs"
add wave -expand -height 17 $TCOMP/frame_info  
add wave -expand -height 17 $TCOMP/drv_settings

#Circuit component			
set CCOMP "messageFilter_comp"  
add wave -noupdate -divider -height 20 "Circuit state"
add wave $TCOMP/$CCOMP/int_filter_a_valid
add wave $TCOMP/$CCOMP/int_filter_b_valid
add wave $TCOMP/$CCOMP/int_filter_c_valid
add wave $TCOMP/$CCOMP/int_filter_ran_valid

add wave -noupdate -divider -height 20 "Circuit outputs"
add wave $TCOMP/out_ident_valid

################################################################################
# Execute the simulation, gather results
################################################################################
run_simulation
get_test_results
