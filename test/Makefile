PYTHON := python3
XUNIT ?= 0

TEST_FLAGS = -p`nproc`

ifeq ($(XUNIT),0)
	TEST_FLAGS +=
	POSTPROC := :
else
	TEST_FLAGS += --xunit-xml ../$@.xml1
	POSTPROC = ret=$$?; (echo '<?xml version="1.0" encoding="utf-8"?>'; echo '<?xml-stylesheet href="xunit.xsl" type="text/xsl"?>'; cat $@.xml1) >$@.xml; rm $@.xml1; exit $$ret
endif

all: test coverage

elaborate:
	$(PYTHON) run.py --elaborate

test:
	$(PYTHON) run.py $(TEST_FLAGS); $(POSTPROC)
test_unit:
	$(PYTHON) run.py $(TEST_FLAGS) 'lib.tb_*_unit_test.*'; $(POSTPROC)
test_feature:
test_sanity:
	$(PYTHON) run.py $(TEST_FLAGS) 'lib.tb_sanity.*'; $(POSTPROC)

coverage:
	lcov --capture --directory build --output-file code_coverage.info
	genhtml code_coverage.info --output-directory code_html

clean:
	-rm -Rf build

.PHONY: all elaborate test test_unit test_feature test_sanity clean
