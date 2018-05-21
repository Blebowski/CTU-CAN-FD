PYTHON := python3
XUNIT ?= 0

TEST_FLAGS = -p`nproc`

ifeq ($(XUNIT),0)
	TEST_FLAGS +=
else
	TEST_FLAGS += --xunit-xml $@.xml
endif

all: test coverage

elaborate:
	$(PYTHON) run.py --elaborate

test:
	$(PYTHON) run.py $(TEST_FLAGS)
test_unit:
	$(PYTHON) run.py $(TEST_FLAGS) 'lib.tb_*_unit_test.*'
test_feature:
test_sanity:
	$(PYTHON) run.py $(TEST_FLAGS) 'lib.tb_sanity.*'

coverage:
	lcov --capture --directory . --output-file $@
	genhtml code_coverage.info --output-directory code_html

clean:
	-rm -Rf *.gcno *.gcda vunit_out

.PHONY: all elaborate test test_unit test_feature test_sanity clean
