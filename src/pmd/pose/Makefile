CXXFLAGS = `pkg-config --cflags opencv`  `pkg-config --cflags ftgl`
CXXFLAGS += -I../pmdaccess2 -I../ -isystem ../../trunk/src
LDFLAGS = `pkg-config --libs libconfig` `pkg-config --libs opencv` 
LDFLAGS += -lxmlrpc_util -lxmlrpc_client -lxmlrpc -lxmlrpc++ -lxmlrpc_client++ -lglfw -lGLU -lftgl 
ARCH = $(shell uname -m)
ifneq ($(ARCH), i686)
LDFLAGS += -lxmlrpc_xmltok -lxmlrpc_xmlparse
else
CXXFLAGS += -pedantic
endif
CXXFLAGS += -Wall -Wextra 
CXXFLAGS += -g -ggdb  -pg
CXXFLAGS += -O3

pose: pose.o history.o ../pmdWrap.o ../cvpmd.o ../../trunk/obj/icp6D.o ../../trunk/obj/icp6D*.o ../../trunk/obj/scanlib.a ../../trunk/src/newmat/libnewmat.a ../pmdaccess2/libpmdaccess2.a

.PHONY: profile clean leak

profile: 
	gprof `./pose --ui 0 --gl 0` | gprof2dot | dot -Tpng -o ./profile.png

clean:
	rm -f pose *.o

leak:
	valgrind --leak-check=yes `./pose --ui 0 --gl 0`
