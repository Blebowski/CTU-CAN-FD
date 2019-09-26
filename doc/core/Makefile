all: Progdokum.pdf ctu_can_fd_architecture.pdf

Progdokum.pdf: version.tex
	#lyx --export-to pdf Progdokum.pdf Progdokum.lyx
	lyx --export-to latex Progdokum.tex Progdokum.lyx
	sed -rie '/\{inputenc\}/d' Progdokum.tex
	iconv -f latin2 -t utf8 <Progdokum.tex >Progdokum.tex-1 && mv Progdokum.tex-1 Progdokum.tex
	xelatex Progdokum
	# run again to generate PDF outline and references
	xelatex Progdokum
	xelatex Progdokum

ctu_can_fd_architecture.pdf: version.tex
	lyx --export-to latex ctu_can_fd_architecture.tex ctu_can_fd_architecture.lyx
	sed -rie '/\{inputenc\}/d' ctu_can_fd_architecture.tex
	iconv -f latin2 -t utf8 <ctu_can_fd_architecture.tex >ctu_can_fd_architecture.tex-1 && mv ctu_can_fd_architecture.tex-1 ctu_can_fd_architecture.tex
	xelatex ctu_can_fd_architecture
	# run again to generate PDF outline and references
	xelatex ctu_can_fd_architecture
	xelatex ctu_can_fd_architecture


version.tex: FORCE
	desc=$$(git describe --always); \
	date="$$(git show -s --format=%cd --date=format:%Y-%m-%d)"; \
	echo "Datasheet $$desc, $$date" >$@

clean:
	-rm -f *.aux *.toc *.out *.log Progdokum.tex CANFrameFormat.tex registerMap.tex

FORCE:

.PHONY: all clean
