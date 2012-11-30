# makefile for laszippertest
#
#COPTS    = -g -Wall
COPTS     = -O3 -Wall
COMPILER  = g++
LINKER  = g++
#BITS     = -64

#LIBS     = -L/usr/lib32
#LIBS     = -L/usr/lib64
INCLUDE  = -I/usr/include

LASLIBS     = -L../lib
LASINCLUDE  = -I../inc

all: laszippertest

laszippertest: laszippertest.o
	${LINKER} ${BITS} ${COPTS} laszippertest.o -llas -o $@ ${LIBS} ${LASLIBS} $(INCLUDE) $(LASINCLUDE)

.cpp.o: 
	${COMPILER} ${BITS} -c ${COPTS} ${INCLUDE} $(LASINCLUDE) $(SRINCLUDE) $< -o $@

.c.o: 
	${COMPILER} ${BITS} -c ${COPTS} ${INCLUDE} $(LASINCLUDE) $(SRINCLUDE) $< -o $@

clean:
	rm -rf *.o
	rm -rf laszippertest

clobber:
	rm -rf *.o
	rm -rf laszippertest
	rm -rf *~
