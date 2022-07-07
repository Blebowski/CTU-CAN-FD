source synth_core.tcl
source benchmark_configs.tcl

load_rtl
read_xdc ../../Constraints/ctu_can_fd.sdc

# Enable preset clear arcs right away to allow synthesis use this timing information!
#config_timing_analysis -enable_preset_clear_arcs true

# Run through all design configurations
global DESIGN_CONFIGS

set cfg [lindex $DESIGN_CONFIGS 1]
set cfg_name [dict get $cfg name]

run_synth $cfg_name
write_outputs $cfg_name




