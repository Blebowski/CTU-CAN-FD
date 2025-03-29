#!/bin/bash

cd $TS_REPO_ROOT/test

export NUM_CPUS=10

# First run for clean-up
rm -rf vunit_out
VUNIT_SIMULATOR=nvc ./run.py tb_rtl_test_nightly -p $NUM_CPUS

VUNIT_SIMULATOR=nvc ./run.py tb_rtl_test_fast_asic -p $NUM_CPUS
VUNIT_SIMULATOR=nvc ./run.py tb_rtl_test_fast_fpga -p $NUM_CPUS

VUNIT_SIMULATOR=nvc ./run.py tb_rtl_test_compliance_full_max -p $NUM_CPUS
VUNIT_SIMULATOR=nvc ./run.py tb_rtl_test_compliance_full_typ -p $NUM_CPUS
VUNIT_SIMULATOR=nvc ./run.py tb_rtl_test_compliance_full_min -p $NUM_CPUS
