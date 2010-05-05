CXXFLAGS = `pkg-config --cflags opencv` -I../pmdaccess2 -I../

LDFLAGS = `pkg-config --libs opencv` 
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


all: calibrate grabFramesCam grabFramesPMD extrinsic

calibrate: calibrate.o 

grabFramesPMD: grabFramesPMD.o ../cvpmd.o ../pmdaccess2/libpmdaccess2.a    

grabFramesCam: grabFramesCam.o 

extrinsic: extrinsic.o


clean:
	rm -f calibrate
	rm -f grabFramesPMD
	rm -f grabFramesCam
	rm -f extrinsic
	rm -f *.o
