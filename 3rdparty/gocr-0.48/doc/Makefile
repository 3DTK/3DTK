#
# Makefile for ./doc path, used by configure
#

# these two lines are for cross-compiling, not tested
#srcdir = .
#VPATH = .

# changed to html, tex is not used anymore
#LATEX=@LATEX@
#DVIPS=@DVIPS@

OCRDOC=ocr
# add other source file to documentation here
# SRC=$(OCRDOC).tex

.PHONY : all clean proper install uninstall
default: all

all:	# do nothing!

#$(OCRDOC).ps: $(OCRDOC).dvi
#	$(DVIPS) -o $(OCRDOC).ps $?
#
#$(OCRDOC).dvi: $(SRC)
#	$(LATEX) $(OCRDOC).tex

install: all
	echo "Copy gocr.html to your document path."

uninstall:
	echo "Remove gocr.html from your document path."

clean:
	-rm -f *.bak *~ $(OCRDOC).{dvi,log,aux}

proper: clean
	-rm -f $(OCRDOC).ps 

