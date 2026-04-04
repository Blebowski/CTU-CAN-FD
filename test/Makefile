PYTHON := python3
XUNIT ?= 0
BUILD_DIR = build
TESTFW_DIR = testfw

TEST_FLAGS ?= -p14
TEST_OPTS_test_debug := --no-strict

all: test coverage functional_coverage

# Run test config
%:
	VUNIT_SIMULATOR=nvc $(PYTHON) run.py $@ $(TEST_OPTS_$@) $(TEST_FLAGS) $< --xunit-xml $@.xml


# Convert results to HTML, pack and publish textual summary
%_logs:

# Process reports
	junit2html --summary-matrix $*.xml > $*_summary

# Remove logs from reports to reduce the log size!
	awk ' /<system-out>/ { print; inside=1; next } /<\/system-out>/ { print ""; print; inside=0; next } !inside { print } ' $*.xml > tmp
	mv tmp $*.xml

# Sum simulation time from all tests
	cat vunit_out/test_output/*/output.txt | \
		grep "STOP called with status" | \
		grep -Eo "[0-9]+(us|ns|ps|fs)" | \
		sed 's/fs//g' | \
		sed 's/ps/000/g' | \
		sed 's/ns/000000/g' | \
		sed 's/us/000000000/g' | \
		paste -sd+ | bc > $*.sim_time
	echo "Total simulated time is `cat $*.sim_time` fs"

clean:
	rm -Rf vunit_out

FORCE:

.PHONY: all elaborate clean
