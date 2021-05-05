PYTHON := python3
XUNIT ?= 0
BUILD_DIR = build
TESTFW_DIR = testfw

TEST_FLAGS = -p`nproc`
TEST_OPTS_test_debug := --no-strict

all: test coverage functional_coverage

elaborate:
	$(PYTHON) run.py test tests_fast_asic.yml -- --elaborate $(TEST_FLAGS)
	$(PYTHON) run.py test tests_fast_fpga.yml -- --elaborate $(TEST_FLAGS)

test_%: tests_%.yml FORCE
	$(PYTHON) run.py test $(TEST_OPTS_$@) $< -- $(TEST_FLAGS)

# For debugging purposes of pipelines only
test_compliance_demo: tests_compliance.yml FORCE
	$(PYTHON) run.py test tests_compliance.yml -- ctu_can_fd_tb.can_compliance_tb.demo
test_nightly_demo: tests_nightly.yml FORCE
	$(PYTHON) run.py test tests_nightly.yml -- ctu_can_fd_tb.tb_feature.device_id

coverage:
	lcov --capture --gcov-tool=/opt/ghdl/bin/gcov --directory $(BUILD_DIR) --output-file code_coverage.info
	genhtml code_coverage.info --output-directory code_html

coverage_merged:
	lcov --capture --gcov-tool=/opt/ghdl/bin/gcov --directory $(BUILD_DIR)/code_coverage_merged --output-file code_coverage.info
	genhtml code_coverage.info --output-directory code_html

functional_coverage:
	$(PYTHON) $(TESTFW_DIR)/test_parse_psl_coverage.py

delete_vunit_out:
	rm -Rf build/vunit_out

clean:
	-rm -Rf build

FORCE:

.PHONY: all elaborate clean
