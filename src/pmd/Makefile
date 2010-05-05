include ../../Makefile.options

BIN = ../../bin/
OBJ = ../../obj/

CXXFLAGS = `pkg-config --cflags opencv` 
CXXFLAGS += -I./pmdaccess2  -I../

all: $(OBJ)pmdWrap.o
	make -C offline
	make -C calibrate
	make -C pose

$(OBJ)pmdWrap.o: $(OBJ)cvpmd.o

$(OBJ)cvpmd.o: $(OBJ)libpmdaccess2.a

$(OBJ)libpmdaccess2.a: ./pmdaccess2/pmdaccess.cc
	make -C pmdaccess2

clean:
	rm -f cvpmd.o pmdWrap.o
	make clean -C offline
	make clean -C calibrate
	make clean -C pose
	make clean -C pmdaccess2

