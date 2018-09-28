all: Progdokum.pdf

Progdokum.pdf: version.tex
	lyx --export-to pdf Progdokum.pdf Progdokum.lyx

version.tex: FORCE
	desc=$$(git describe --always); \
	date="$$(git show -s --format=%cd --date=format:%Y-%m-%d)"; \
	echo "Datasheet $$desc, $$date" >$@

clean:
	-rm -f *.aux *.toc *.out *.log Progdokum.tex CANFrameFormat.tex registerMap.tex

FORCE:

.PHONY: all clean
