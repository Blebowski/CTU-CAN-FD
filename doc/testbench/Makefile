all: ctu_can_fd_testbench

ctu_can_fd_testbench: version.tex
	lyx --export-to pdf2 $@.pdf $@.lyx

version.tex: FORCE
	desc=$$(git describe --always --tags); \
	date="$$(git show -s --format=%cd --date=format:%Y-%m-%d)"; \
	echo "$$desc, $$date" >$@

clean:
	-rm -f *.aux *.toc *.out *.log *.tex *.texe *.pdf

FORCE:

.PHONY: all clean
