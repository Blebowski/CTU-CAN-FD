all: Progdokum ctu_can_fd_architecture tb_architecture

Progdokum: version.tex
	#lyx --export-to pdf Progdokum.pdf Progdokum.lyx
	lyx --export-to latex $@.tex $@.lyx
	sed -rie '/\{inputenc\}/d' $@.tex
	iconv -f latin2 -t utf8 <$@.tex >$@.tex-1 && mv $@.tex-1 $@.tex
	xelatex $@
	# run again to generate PDF outline and references
	xelatex $@
	xelatex $@

ctu_can_fd_architecture: version.tex
	lyx --export-to latex $@.tex $@.lyx
	sed -rie '/\{inputenc\}/d' $@.tex
	iconv -f latin2 -t utf8 <$@.tex >$@.tex-1 && mv $@.tex-1 $@.tex
	xelatex $@
	# run again to generate PDF outline and references
	xelatex $@
	xelatex $@

tb_architecture: version.tex
	lyx --export-to latex $@.tex $@.lyx
	sed -rie '/\{inputenc\}/d' $@.tex
	iconv -f latin2 -t utf8 <$@.tex >$@.tex-1 && mv $@.tex-1 $@.tex
	xelatex $@
	# run again to generate PDF outline and references
	xelatex $@
	xelatex $@

version.tex: FORCE
	desc=$$(git describe --always); \
	date="$$(git show -s --format=%cd --date=format:%Y-%m-%d)"; \
	echo "Datasheet $$desc, $$date" >$@

clean:
	-rm -f *.aux *.toc *.out *.log *.tex *.texe

FORCE:

.PHONY: all clean
