PYTHON := python3
XUNIT ?= 0

TEST_FLAGS = -p`nproc`
TEST_OPTS_test_debug := --no-strict

all: test coverage functional_coverage

elaborate:
	$(PYTHON) run.py test tests_fast.yml -- --elaborate $(TEST_FLAGS)

test_%: tests_%.yml FORCE
	$(PYTHON) run.py test $(TEST_OPTS_$@) $< -- $(TEST_FLAGS)

coverage:
	lcov --capture --directory build --output-file code_coverage.info
	genhtml code_coverage.info --output-directory code_html

functional_coverage:
	mkdir -p build/functional_coverage
	cp build/psl_coverage.json build/functional_coverage/psl_coverage.json
	$(PYTHON) testfw/test_parse_psl_coverage.py

clean:
	-rm -Rf build

FORCE:

.PHONY: all elaborate clean
