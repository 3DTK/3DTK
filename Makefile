#MAKEFLAGS += -j

all: builddir .configured
	@mkdir -p .build
#	cd .build && cmake ..
ifneq ($(MAKE),)
	cd .build && $(MAKE) --no-print-directory
else
	cd .build && make --no-print-directory
endif

builddir:
	@mkdir -p .build

.configured:
	touch .configured
	cd .build && cmake ..

clean:
	-cd .build && make clean --no-print-directory
	rm -rf .build
	rm .configured

config: builddir
	touch .configured
	cd .build && ccmake ../


DOC     = doc/
docu: docu_html docu_latex docu_hl
	echo
	echo
	echo + Reference documentation generated: $(DOC)html/index.html
	echo + Reference documentation generated: $(DOC)refman.pdf
	echo + Highlevel documentation generated: $(DOC)documentation_HL.pdf
	echo

docu_html:
	doxygen doc/doxygen.cfg
	cd $(DOC) ; zip -q html.zip html/*
	echo
	echo

docu_latex:
	$(MAKE) -C $(DOC)latex
	cd $(DOC)latex ; dvips refman
	cd $(DOC)latex ; ps2pdf14 refman.ps refman.pdf
	cp $(DOC)latex/refman.pdf $(DOC)

docu_hl:	$(DOC)high_level_doc/documentation.tex
	cd $(DOC)high_level_doc ; latex documentation.tex
	cd $(DOC)high_level_doc ; bibtex documentation
	cd $(DOC)high_level_doc ; latex documentation.tex
	cd $(DOC)high_level_doc ; dvips documentation
	cd $(DOC)high_level_doc ; ps2pdf14 documentation.ps ../documentation_HL.pdf
