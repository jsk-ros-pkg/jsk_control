TMPDIR := $(shell mktemp -d)
FIGDIR := ${TMPDIR}/fig
EUSFILES = instant-configuration-task trajectory-configuration-task sqp-optimization robot-object-environment instant-manipulation-configuration-task bspline-fundamental bspline-configuration-task extended-joint-link contact-kinematics robot-environment torque-gradient inverse-kinematics-wrapper sqp-msc-optimization bspline-dynamic-configuration-task discrete-kinematics-configuration-task
EUSFILESDIR = "../euslisp"

# IMAGE_FILES := $(shell find ${FIGDIR} -name "*.(jpg|png)" | sed s/jpg/bb/g)

# BB_FILES = $(patsubst %.jpg,%.bb,$(IMAGE_FILES))

# .SUFFIXES: .txt .tex .bbl .bb .jpg .JPG .png .PNG .bmp .BMP .pdf

# .jpg.bb:
# 	ebb -b $<

all: pdf
	rm -fr ${TMPDIR}

copy_eus_tex:
	exit
	-mkdir -p ${TMPDIR}/fig
	# cp ../eus/doc/jlatex/*.tex ${TMPDIR}
	cp jlatex/*.sty ${TMPDIR}
	# cp -r ../eus/doc/jlatex/fig/* ./fig/* ${TMPDIR}/fig/
	# for x in ${TMPDIR}/fig/*.jpg; do ebb -b $$x; extractbb $$x;done
	cp *.tex ${TMPDIR}
	platex --version | grep utf8 || nkf --in-place -e ${TMPDIR}/*.tex
	for x in ${EUSFILES}; do\
		sed "s@\\\\@\\\\\\\\@g" ${EUSFILESDIR}/$$x.l > ${TMPDIR}/$$x.l; \
		roseus ./documentation.l "(make-document \"${TMPDIR}/$$x.l\" \"${TMPDIR}/$$x.tex\")" "(exit)"; \
	done

toc: copy_eus_tex
	(cd ${TMPDIR}; makeindex manual.tex)

dvi: toc
	(cd ${TMPDIR}; platex manual.tex)
	(cd ${TMPDIR}; platex manual.tex)
	cp ${TMPDIR}/*.dvi .

pdf: dvi
	(cd ${TMPDIR}; dvipdfmx manual.dvi)
	cp ${TMPDIR}/*.pdf .


clean:
	rm -f *~ *.aux *.idx *.log *.toc *.out *.ind *.ilg

distclean: clean
	rm -f *.dvi *.pdf

open:
	gnome-open manual.pdf

watch:
	$(MUTE)CHANGE=true &&                            \
	while true ; do                                  \
          if $$CHANGE ; then                             \
	    for i in ${EUSFILES} ; do                    \
	      touch .$$i -r ${EUSFILESDIR}/$$i.l;        \
	    done;                                        \
	    CHANGE=false;                                \
	    make;                                        \
          fi;                                            \
          sleep 1;                                       \
          for i in ${EUSFILES} ; do                      \
	    if [ .$$i -ot ${EUSFILESDIR}/$$i.l ] ; then  \
	      CHANGE=true;                               \
	    fi                                           \
          done ;                                         \
        done
