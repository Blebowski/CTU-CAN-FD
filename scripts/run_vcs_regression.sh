#!/bin/bash

# First run for clean-up
ts_sim_run.py --recompile --no-sim-out --clear --clear-logs tb_rtl_test_nightly \*

ts_sim_run.py --recompile --no-sim-out tb_rtl_test_fast_asic \*
ts_sim_run.py --recompile --no-sim-out tb_rtl_test_fast_fpga \*

ts_sim_run.py --recompile --no-sim-out tb_rtl_test_compliance_full_max \*
ts_sim_run.py --recompile --no-sim-out tb_rtl_test_compliance_full_typ \*
ts_sim_run.py --recompile --no-sim-out tb_rtl_test_compliance_full_min \*
