
###################################################################################################
## Procedures
###################################################################################################

proc load_rtl {} {
    global PROJ_ROOT
    read_vhdl -library ctu_can_fd_rtl -verbose [glob $PROJ_ROOT/src/*/*/**.vhd]
    read_vhdl -library ctu_can_fd_rtl -verbose [glob $PROJ_ROOT/src/*/**.vhd]
    read_vhdl -library ctu_can_fd_rtl -verbose [glob $PROJ_ROOT/src/*.vhd]
}

proc form_generics {cfg_name} {
    global DESIGN_CONFIGS
    set CMD ""
    foreach CFG $DESIGN_CONFIGS {
        if {[dict get $CFG name] == $cfg_name} {
            dict for {PARAM VALUE} [dict get $CFG generics] {
                append CMD "-generic ${PARAM}=${VALUE} "
            }
        }
    }
    return "${CMD}"
}

proc run_synth {cfg_name} {
    global TOP
    global PART
    
    puts "Running synthesis of design configuration: ${cfg_name} ..."
    set GENERICS [form_generics $cfg_name]
    set CMD "synth_design -top ${TOP} -part ${PART} ${GENERICS}"    
    eval $CMD
    
    # Do optimize to get also better results
    opt_design -resynth_seq_area
}

proc write_outputs {cfg_name} {
    global TOP

    exec rm -rf $cfg_name
    exec mkdir $cfg_name
    
    config_timing_analysis -enable_preset_clear_arcs true
    #udpate_timing

    report_timing_summary > $cfg_name/timing_summary.rpt
    report_utilization > $cfg_name/utilization.rpt
    report_utilization -hierarchical -hierarchical_percentages > $cfg_name/utilization_hierarchy.rpt

    write_vhdl -include_xilinx_libs -mode funcsim -verbose $cfg_name/$TOP.vhd
    write_verilog -include_xilinx_libs -mode funcsim $cfg_name/$TOP.v
    write_sdf -mode timesim -process_corner slow $cfg_name/$TOP\_slow.sdf
    write_sdf -mode timesim -process_corner fast $cfg_name/$TOP\_fast.sdf
    write_xdc $cfg_name/$TOP.xdc
    
    exec cp vivado.log $cfg_name
}


###################################################################################################
## Main part of the script
###################################################################################################

source benchmark_configs.tcl

load_rtl
read_xdc ../../Constraints/ctu_can_fd.sdc

# Run through all design configurations
global DESIGN_CONFIGS
foreach cfg $DESIGN_CONFIGS {
    set cfg_name [dict get $cfg name]
    #puts "${cfg_name}"
    run_synth $cfg_name
    write_outputs $cfg_name
}



