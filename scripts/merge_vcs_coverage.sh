#!/bin/bash

ts_sim_coverage.py -o merged_fast sim_tb_rtl_test_fast_* --clear
ts_sim_coverage.py -o merged_nightly sim_tb_rtl_test_nightly_rx_status_rx_buffer_size_4096_* sim_tb_rtl_test_nightly_* --clear
ts_sim_coverage.py -o merged_compliance_min sim_tb_rtl_test_compliance_full_min* --clear
ts_sim_coverage.py -o merged_compliance_typ sim_tb_rtl_test_compliance_full_typ* --clear
ts_sim_coverage.py -o merged_compliance_max sim_tb_rtl_test_compliance_full_max* --clear

rm -rf merged_nightly
rm -rf merged_compliance_min
rm -rf merged_compliance_typ
rm -rf merged_compliance_max
rm -rf merged_fast

mkdir merged_nightly
mkdir merged_compliance_min
mkdir merged_compliance_typ
mkdir merged_compliance_max
mkdir merged_fast

mv ../coverage_output/merged_fast.vdb merged_fast/simv.vdb
mv ../coverage_output/merged_nightly.vdb merged_nightly/simv.vdb
mv ../coverage_output/merged_compliance_min.vdb merged_compliance_min/simv.vdb
mv ../coverage_output/merged_compliance_typ.vdb merged_compliance_typ/simv.vdb
mv ../coverage_output/merged_compliance_max.vdb merged_compliance_max/simv.vdb

cp sim_tb_rtl_test_nightly_alc_ide_*/_ts_flow_reference_elaboration_directory merged_fast
cp sim_tb_rtl_test_nightly_alc_ide_*/_ts_flow_reference_elaboration_directory merged_nightly
cp sim_tb_rtl_test_nightly_alc_ide_*/_ts_flow_reference_elaboration_directory merged_compliance_min
cp sim_tb_rtl_test_nightly_alc_ide_*/_ts_flow_reference_elaboration_directory merged_compliance_typ
cp sim_tb_rtl_test_nightly_alc_ide_*/_ts_flow_reference_elaboration_directory merged_compliance_max

ts_sim_coverage.py merged_compliance_max merged_compliance_min merged_compliance_typ merged_fast merged_nightly -o merged_total --clear
