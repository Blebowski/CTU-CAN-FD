#!/bin/bash

ts_sim_coverage.py -o merged_fast sim_tb_rtl_test_fast_* --clear
ts_sim_coverage.py -o merged_nightly sim_tb_rtl_test_nightly_rx_status_rx_buffer_size_4096_* sim_tb_rtl_test_nightly_* --clear
ts_sim_coverage.py -o merged_compliance sim_tb_rtl_test_compliance_full_* --clear

rm -rf merged_nightly
rm -rf merged_compliance
rm -rf merged_fast

mkdir merged_nightly
mkdir merged_compliance
mkdir merged_fast

mv ../coverage_output/merged_fast.vdb merged_fast/simv.vdb
mv ../coverage_output/merged_nightly.vdb merged_nightly/simv.vdb
mv ../coverage_output/merged_compliance.vdb merged_compliance/simv.vdb

cp sim_tb_rtl_test_nightly_alc_ide_*/_ts_flow_reference_elaboration_directory merged_fast
cp sim_tb_rtl_test_nightly_alc_ide_*/_ts_flow_reference_elaboration_directory merged_nightly
cp sim_tb_rtl_test_nightly_alc_ide_*/_ts_flow_reference_elaboration_directory merged_compliance