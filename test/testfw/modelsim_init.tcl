global TCOMP
# set TCOMP tb/i_test

proc start_CAN_simulation {test_wrapper} {
}

proc run_simulation {} {
    vunit_run
}

proc get_test_results_feature {} {
}

proc get_test_results {} {
}

rename add _add
proc safe_add {args} {
    if {[catch {_add {*}$args} fid]} {
        puts stderr $fid
    }
}

global IgnoreAddWaveErrors
set IgnoreAddWaveErrors 0

proc add {args} {
    global IgnoreAddWaveErrors
    if {$IgnoreAddWaveErrors} {
        safe_add {*}$args
    } else {
        _add {*}$args
    }
}

proc sim_restart {} {
    vunit_restart
    vunit_user_init
}

rename vunit_help _vunit_help
proc vunit_help {} {
    _vunit_help
    puts "sim_restart:"
    puts "  - recompile, add waves, restart (a.k.a vunit_restart; vunit_user_init)"
}
