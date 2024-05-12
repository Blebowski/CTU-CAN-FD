#!/bin/bash
#
# sanitize_compliance_test.sh <compliance_test_name> <compliance_test_name> ...
#

if [ "$#" -eq 0 ]
then
  echo "No test name passed as argument!"
  echo "Launch as:"
  echo "    sanitize_compliance_test.sh <compliance_test_name> <compliance_test_name> ..."
  exit 1
fi

cd $TS_REPO_ROOT

export VALGRIND_LOG_DIR="test/valgrind_results"
export SANITIZE_SIM_TARGET="tb_rtl_test_compliance_full_typ"

rm -rf $VALGRIND_LOG_DIR
mkdir -p $VALGRIND_LOG_DIR
rm -rf sim/sim_logs/*
rm -rf sim/elab_logs/*

echo "#########################################################################"
echo "Tests to sanitize:"
for TEST in "$@"
do
    echo "  $TEST"
done
echo "#########################################################################"


echo "#########################################################################"
echo "Rebuilding compliance library..."
echo "#########################################################################"

cd test/main_tb/iso-16845-compliance-tests
rm -rf build
./build.sh
cd ../../..





echo "#########################################################################"
echo "#########################################################################"
echo "Sanitizing Debug build of libVCS_VHPI_COSIM_LIB.so ..."
echo "#########################################################################"
echo "#########################################################################"
export LD_LIBRARY_PATH=`pwd`/test/main_tb/iso-16845-compliance-tests/build/Debug/src/cosimulation

for TEST in "$@"
do
    echo "#########################################################################"
    echo "Sanitizing test: $TEST..."
    echo "#########################################################################"

    echo "#########################################################################"
    echo "Running reference simulation..."
    echo "#########################################################################"

    # Capture both stdout and stderr to "ts_hw_scripts.log"
    ts_sim_run.py --recompile --clear --seed 0 -vv $SANITIZE_SIM_TARGET $TEST 2>&1 | tee ts_hw_scripts.log

    # Parse out the exact simulation command used to simulate
    export SIM_CMD=`cat ts_hw_scripts.log | grep -E "\[INFO\] (.)+simv(.)+"`

    # Remove colors
    export SIM_CMD=`echo $SIM_CMD | sed -e 's/\x1b\[[0-9;]*m//g'`

    # Remove prefix
    export SIM_CMD=${SIM_CMD#* }

    echo "#########################################################################"
    echo ""
    echo "Simulation command used to run simulation:"
    echo ""
    echo "$SIM_CMD"
    echo ""

    # Append "_debug_valgrind" to simulation log file name
    export SIM_CMD=`echo $SIM_CMD | sed -e 's/\.log/_valgrind_debug.log/g'`

    # Prepend valgrind options
    export VALGRIND_CMD="valgrind --leak-check=yes --trace-children=yes -s --log-file=${VALGRIND_LOG_DIR}/${TEST}_valgrind_debug.log $SIM_CMD"

    echo "Command to run the simulation with sanitizer:"
    echo ""
    echo "$VALGRIND_CMD"
    echo ""
    echo "#########################################################################"

    echo ""
    echo ""
    echo "Sanitizing '${TEST}' test:"
    echo ""
    echo ""
    $VALGRIND_CMD

done





echo "#########################################################################"
echo "#########################################################################"
echo "Sanitizing Release build of libVCS_VHPI_COSIM_LIB.so ..."
echo "#########################################################################"
echo "#########################################################################"

export LD_LIBRARY_PATH=`pwd`/test/main_tb/iso-16845-compliance-tests/build/Release/src/cosimulation

for TEST in "$@"
do
    echo "#########################################################################"
    echo "Sanitizing test: $TEST..."
    echo "#########################################################################"

    echo "#########################################################################"
    echo "Running reference simulation..."
    echo "#########################################################################"

    # Capture both stdout and stderr to "ts_hw_scripts.log"
    ts_sim_run.py --recompile --clear --seed 0 -vv $SANITIZE_SIM_TARGET $TEST 2>&1 | tee ts_hw_scripts.log

    # Parse out the exact simulation command used to simulate
    export SIM_CMD=`cat ts_hw_scripts.log | grep -E "\[INFO\] (.)+simv(.)+"`

    # Remove colors
    export SIM_CMD=`echo $SIM_CMD | sed -e 's/\x1b\[[0-9;]*m//g'`

    # Remove prefix
    export SIM_CMD=${SIM_CMD#* }

    echo "#########################################################################"
    echo ""
    echo "Simulation command used to run simulation:"
    echo ""
    echo "$SIM_CMD"
    echo ""

    # Append "_debug_valgrind" to simulation log file name
    export SIM_CMD=`echo $SIM_CMD | sed -e 's/\.log/_valgrind_release.log/g'`

    # Prepend valgrind options
    export VALGRIND_CMD="valgrind --leak-check=yes --trace-children=yes -s --log-file=${VALGRIND_LOG_DIR}/${TEST}_valgrind_release.log $SIM_CMD"

    echo "Command to run the simulation with sanitizer:"
    echo ""
    echo "$VALGRIND_CMD"
    echo ""
    echo "#########################################################################"

    echo ""
    echo ""
    echo "Sanitizing '${TEST}' test:"
    echo ""
    echo ""
    $VALGRIND_CMD

done