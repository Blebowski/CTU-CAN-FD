PYTHON := python3
XUNIT ?= 0

TEST_FLAGS = -p`nproc`

all: test coverage

elaborate:
	$(PYTHON) run.py test tests_fast.yml -- --elaborate $(TEST_FLAGS)

test_%: tests_%.yml FORCE
	$(PYTHON) run.py test $< -- $(TEST_FLAGS)

coverage:
	lcov --capture --directory build --output-file code_coverage.info
	genhtml code_coverage.info --output-directory code_html

clean:
	-rm -Rf build

FORCE:

.PHONY: all elaborate clean
