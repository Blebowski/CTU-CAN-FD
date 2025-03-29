PYTHON := python3
XUNIT ?= 0
BUILD_DIR = build
TESTFW_DIR = testfw

TEST_FLAGS = -p10
TEST_OPTS_test_debug := --no-strict

all: test coverage functional_coverage

# Run test config
%:
	VUNIT_SIMULATOR=nvc $(PYTHON) run.py $@ $(TEST_OPTS_$@) $(TEST_FLAGS) $< --xunit-xml $@.xml


# Convert results to HTML, pack and publish textual summary
%_logs:

# Process reports
#	junit2html --report-matrix $*.html $*.xml
#	tar -czvf $*.gz $*.html $*.xml.html
	junit2html --summary-matrix $*.xml > $*_summary

# Sum simulation time from all tests
	cat vunit_out/test_output/*/output.txt | \
		grep "simulation stopped" | \
		grep -Eo "[0-9]+(us|ns|ps)" | \
		sed 's/ps/000/g' | \
		sed 's/ns/000000/g' | \
		sed 's/us/000000000/g' | \
		paste -sd+ | bc > $*.sim_time
	echo "Total simulated time is `cat $*.sim_time` fs"

# Cleanup
#	rm -f $*.html
#	rm -f $*.xml.html
#	rm -f $*.xml


######################################################################################
# Coverage processing
######################################################################################
functional_coverage:
	mkdir -p vunit_out/functional_coverage/html
	$(PYTHON) $(TESTFW_DIR)/test_parse_psl_coverage.py

clean:
	rm -Rf vunit_out

FORCE:

.PHONY: all elaborate clean
