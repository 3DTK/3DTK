include ../../../Makefile.options

BIN = ../../../bin/
OBJ = ../../../obj/

CXXFLAGS = `pkg-config --cflags opencv` -I../pmdaccess2 -I../ -I../../ `pkg-config --cflags ftgl`
LDFLAGS = `pkg-config --libs libconfig` `pkg-config --libs opencv` -lglfw -lGLU -lftgl
LDFLAGS += -lxmlrpc_util -lglfw -lGLU -lftgl  -lxmlrpc_client -lxmlrpc -lxmlrpc++ -lxmlrpc_client++

ARCH = $(shell arch)
ifneq ($(ARCH), i686)
LDFLAGS += -lxmlrpc_xmltok -lxmlrpc_xmlparse
else
CXXFLAGS += -pedantic
endif
CXXFLAGS += -Wall -Wextra 
#CXXFLAGS += -g -ggdb  -pg
CXXFLAGS += -O3

all: grabVideoAnd3D convertToSLAM6D

$(BIN)convertToSLAM6D: $(OBJ)convertToSLAM6D.o $(OBJ)pmdWrap.o $(OBJ)cvpmd.o #($OBJ)icp6D.o ($OBJ)icp6D*.o ($OBJ)scanlib.a ($OBJ)libnewmat.a  ../pmdaccess2/libpmdaccess2.a

grabVideoAnd3D: grabVideoAnd3D.o $(OBJ)pmdWrap.o $(OBJ)cvpmd.o $(OBJ)libpmdaccess2.a ../../../obj/icp6D.o ../../../obj/icp6D.o ../../../obj/icp6D*.o ../../../obj/scanlib.a ../../../obj/libnewmat.a 

clean:
	rm -f grabVideoAnd3D convertToSLAM6D
	rm -f grabVideoAnd3D.o convertToSLAM6D.o
