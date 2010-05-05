include ../../../Makefile.options

BIN = ../../../bin/
OBJ = ../../../obj/

LDFLAGS = -pg 
CXXFLAGS = -Wall -Wextra 
CXXFLAGS += -O3
# OpenCV has "long long" error with pedantic @ 64bit Ubuntu
ifneq ($(ARCH), i686)
else
CXXFLAGS += -pedantic
endif

#CXXFLAGS += -g -ggdb  -pg


stat:
	g++ -fPIC -c pmdaccess.cc -o $(OBJ)pmdaccess.o
	ar rs $(OBJ)libpmdaccess2.a $(OBJ)pmdaccess.o

lib:
	g++ -fPIC -c pmdaccess.cc -o $(OBJ)pmdaccess.o
	#gcc -shared -Wl,-soname,libpmdaccess2.so pmdaccess.o
	ld -o $(BIN)libpmdaccess2.so -G $(OBJ)pmdaccess.o

clean:
	rm -f $(BIN)libpmdaccess2.so $(OBJ)pmdaccess.o $(OBJ)libpmdaccess2.a

