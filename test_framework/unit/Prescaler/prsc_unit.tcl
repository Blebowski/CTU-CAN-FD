################################################################################
## Author:      Ondrej Ille , Czech Technical University, FEL
## Project:     CAN FD IP Core Project
##
## 			Bit stuffing and destuffing unit test handling script
################################################################################
global TCOMP

start_CAN_simulation "presc_unit_test_wrapper"

################################################################################
# Adding the waves
################################################################################

#Add common waves for each test entity
add_test_status_waves
add_system_waves
		
#Add circuit specific signals
add wave -noupdate -divider -height 20 "DUT inputs (generated)"	
add wave -group "Bit time settings" \
		   -label "Time quanta (Nominal)" -unsigned $TCOMP/drv_tq_nbt \
		   -label "PROP_SEG (Nominal)" -unsigned $TCOMP/drv_prs_nbt \
		   -label "PH1_SEG (Nominal)" -unsigned $TCOMP/drv_ph1_nbt \
		   -label "PH2_SEG (Nominal)" -unsigned $TCOMP/drv_ph2_nbt \
		   -label "Synchron. jump width (Nominal)" -unsigned $TCOMP/drv_sjw_nbt \
		   -label "Time quanta (Data)" -unsigned $TCOMP/drv_tq_dbt \
		   -label "PROP_SEG (Data)" -unsigned $TCOMP/drv_prs_dbt \
		   -label "PH1_SEG (Data)" -unsigned $TCOMP/drv_ph1_dbt \
		   -label "PH2_SEG (Data)" -unsigned $TCOMP/drv_ph2_dbt \
		   -label "Synchron. jump width (Data)" -unsigned $TCOMP/drv_sjw_dbt
		   
add wave -label "Synchronization edge" $TCOMP/sync_edge
add wave -label "Sample control" -unsigned $TCOMP/sp_control
add wave -label "Synchronization control" -unsigned $TCOMP/sync_control

add wave -noupdate -divider -height 20 "DUT outputs"
add wave -group "Triggerring signals" \
		        -label "SYNC (Nominal)" $TCOMP/sync_nbt \
		        -label "SYNC del.1 (Nominal)" $TCOMP/sync_nbt_del_1 \
				-label "SAMPLE (Nominal)" $TCOMP/sample_nbt \
				-label "SAMPLE del.1 (Nominal)" $TCOMP/sample_nbt_del_1 \
				-label "SAMPLE del.2 (Nominal)" $TCOMP/sample_nbt_del_2 \
		        -label "SYNC (Data)" $TCOMP/sync_dbt \
		        -label "SYNC del.1 (Data)" $TCOMP/sync_dbt_del_1 \
				-label "SAMPLE (Data)" $TCOMP/sample_dbt \
				-label "SAMPLE del.1 (Data)" $TCOMP/sample_dbt_del_1 \
				-label "SAMPLE del.2 (Data)" $TCOMP/sample_dbt_del_2
				
add wave -label "Bit time state" $TCOMP/bt_fsm_out
add wave -label "Hard sync appeared" $TCOMP/hard_sync_edge_valid

add wave -noupdate -divider -height 20 "Error counters"
add wave -label "Inform. proc. time corrupted" \
			-unsigned $TCOMP/ipt_err_ctr		
				
add wave -label "Coherency checks failed" -unsigned $TCOMP/coh_err_ctr
add wave -label "Sync signal missed" -unsigned $TCOMP/sync_seq_err_ctr
add wave -label "Sample signal missed" -unsigned $TCOMP/sample_seq_err_ctr
				
add wave -noupdate -divider -height 20 "Internal DUT signals"								
add wave -label "FSM preset" $TCOMP/prescaler_comp/fsm_preset
add wave -label "Time quantum start" $TCOMP/prescaler_comp/tq_edge
add wave -label "Bit time counter" -unsigned $TCOMP/prescaler_comp/bt_counter
add wave -label "PH1 (after sync.)" -unsigned $TCOMP/prescaler_comp/ph1_real
add wave -label "PH2 (after sync.)" -unsigned $TCOMP/prescaler_comp/ph2_real
								
									 
################################################################################
# Execute the simulation, gather results
################################################################################
run_simulation
get_test_results


