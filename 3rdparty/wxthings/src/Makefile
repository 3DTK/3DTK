# Makefile for wxthings lib (UNIX).

# ----------------------------------------------------------------------------
# wxWidgets build settings

WXCONFIG := wx-config

WXDIR      = $(shell $(WXCONFIG) --prefix)
WXEXECDIR  = $(shell $(WXCONFIG) --exec-prefix)
WXLIBDIR   = $(WXEXECDIR)/lib

WXBASENAME = $(shell $(WXCONFIG) --basename)
WXVERSION  = $(shell $(WXCONFIG) --version)
WXRELEASE  = $(shell $(WXCONFIG) --release)

WXCXX      = $(shell $(WXCONFIG) --cxx)
WXCXXFLAGS = $(shell $(WXCONFIG) --cxxflags)
WXLIBS     = $(shell $(WXCONFIG) --libs)

WXSTC_LIB  = $(WXBASENAME)_stc-$(WXRELEASE)

# ----------------------------------------------------------------------------

# in filebrws.cpp we need to resize comboboxes in toolbar
ifndef GTK_CFLAGS
ifeq (gtk2, $(findstring gtk2, $(WXBASENAME)))
GTK_CFLAGS = $(shell pkg-config --cflags gtk+-2.0)
else
GTK_CFLAGS = $(shell gtk-config --cflags)
endif
endif

# ----------------------------------------------------------------------------

# the wxstedit component is used for the wxFileBrowser class
WXSTEDIT_LIB =  $(WXBASENAME)_stedit-$(WXRELEASE)
WXSTEDIT_DIR = ../../wxstedit

# ----------------------------------------------------------------------------

WXTHINGS_LIBNAME = $(WXBASENAME)_wxthings-$(WXRELEASE)
WXTHINGS_LIBDIR = $(WXLIBDIR)

LIBVERSION_CURRENT=1
LIBVERSION_REVISION=0
LIBVERSION_AGE=0

TARGET_LIBNAME = lib$(WXTHINGS_LIBNAME)

TARGET_LIB_STATIC = $(WXTHINGS_LIBDIR)/$(TARGET_LIBNAME).a
TARGET_LIB_SHARED = $(WXTHINGS_LIBDIR)/$(TARGET_LIBNAME).so.$(LIBVERSION_CURRENT).$(LIBVERSION_REVISION).$(LIBVERSION_AGE)
TARGET_LIB_LINK1  = $(TARGET_LIBNAME).so.$(LIBVERSION_CURRENT)
TARGET_LIB_LINK2  = $(TARGET_LIBNAME).so

# ----------------------------------------------------------------------------

CXXFLAGS = $(WXCXXFLAGS) $(GTK_CFLAGS) -MMD -g -Wall -Wunused -Wunused-parameter
LDLIBS   = $(WXLIBS)
CXX      = $(WXCXX)

#-fdump-class-hierarchy  this dumps out a file.cpp.class file
# -pg in libs and defs for profiling
PROFILE = #-pg
APPEXTRADEFS = -I$(WXDIR)/contrib/include -I$(WXSTEDIT_DIR)/include -I../include $(PROFILE)
APPEXTRALIBS = -l$(WXSTC_LIB) -l$(WXSTEDIT_LIB) $(PROFILE)

# ----------------------------------------------------------------------------

RM        = rm -f
AR        = ar
AROPTIONS = rcu
RANLIB    = ranlib
LN_S      = ln -s

# ----------------------------------------------------------------------------

OBJECTS = \
	block.o \
	bmpcombo.o \
	dropdown.o \
	filebrws.o \
	genergdi.o \
	geometry.o \
	menubtn.o \
	optvalue.o \
	range.o \
	spinctld.o \
	toggle.o

DEPFILES=$(OBJECTS:.o=.d)

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(APPEXTRADEFS) -o $@ $<

all: $(TARGET_LIB_STATIC) $(TARGET_LIB_SHARED)

$(TARGET_LIB_STATIC) : $(OBJECTS)
	@$(RM) $@
	$(AR) $(AROPTIONS) $@ $(OBJECTS)
	$(RANLIB) $@

$(TARGET_LIB_SHARED): $(OBJECTS)
	$(CXX) -shared -o $@ $(OBJECTS)
	cd $(WXTHINGS_LIBDIR) \
	&& $(RM) $(TARGET_LIB_LINK1) $(TARGET_LIB_LINK2) \
	&& $(LN_S) $(TARGET_LIB_SHARED) $(TARGET_LIB_LINK1) \
	&& $(LN_S) $(TARGET_LIB_SHARED) $(TARGET_LIB_LINK2)

clean:
	rm -f $(OBJECTS) $(DEPFILES) core \
		$(TARGET_LIB_STATIC) \
		$(TARGET_LIB_SHARED) \
		$(WXTHINGS_LIBDIR)/$(TARGET_LIB_LINK1) \
		$(WXTHINGS_LIBDIR)/$(TARGET_LIB_LINK2)

-include $(DEPFILES)
