#-----------------------------------------------------------------------------
# Makefile for ANN library
#----------------------------------------------------------------------
# Copyright (c) 1997-2005 University of Maryland and Sunil Arya and
# David Mount.  All Rights Reserved.
# 
# This software and related documentation is part of the Approximate
# Nearest Neighbor Library (ANN).  This software is provided under
# the provisions of the Lesser GNU Public License (LGPL).  See the
# file ../ReadMe.txt for further information.
# 
# The University of Maryland (U.M.) and the authors make no
# representations about the suitability or fitness of this software for
# any purpose.  It is provided "as is" without express or implied
# warranty.
#----------------------------------------------------------------------
# History:
#	Revision 0.1  03/04/98
#		Initial release
#	Revision 1.0  04/01/05
#		Renamed files from .cc to .cpp for Microsoft Visual C++
#		Added kd_dump.cpp
#	Revision 1.1  05/03/05
#		Added kd_fix_rad_search.cpp and bd_fix_rad_search.cpp
#----------------------------------------------------------------------

#-----------------------------------------------------------------------------
# Configuration definitions
#-----------------------------------------------------------------------------

include ../../../Makefile.options

#-----------------------------------------------------------------------------
# Some basic definitions:
#		BASEDIR		where include, src, lib, ... are
#		INCLIB		include directory
#		LIBLIB		library directory
#-----------------------------------------------------------------------------
ANNLIB    = ../../../obj/libANN.a
OBJDIR    = ../../../obj/
INCDIR	= ../include/

SOURCES = ANN.cpp brute.cpp kd_tree.cpp kd_util.cpp kd_split.cpp \
	kd_dump.cpp kd_search.cpp kd_pr_search.cpp kd_fix_rad_search.cpp \
	bd_tree.cpp bd_search.cpp bd_pr_search.cpp bd_fix_rad_search.cpp \
	perf.cpp

OBJECTS = $(OBJDIR)ANN.o $(OBJDIR)brute.o $(OBJDIR)kd_tree.o $(OBJDIR)kd_util.o $(OBJDIR)kd_split.o \
	$(OBJDIR)kd_dump.o $(OBJDIR)kd_search.o $(OBJDIR)kd_pr_search.o $(OBJDIR)kd_fix_rad_search.o \
	$(OBJDIR)bd_tree.o $(OBJDIR)bd_search.o $(OBJDIR)bd_pr_search.o $(OBJDIR)bd_fix_rad_search.o \
	$(OBJDIR)perf.o

#-----------------------------------------------------------------------------
# Make the library
#-----------------------------------------------------------------------------

default: $(LIBDIR)/$(ANNLIB)

$(LIBDIR)/$(ANNLIB): $(OBJECTS)
	@ar rs $(ANNLIB) $(OBJECTS)
	@ranlib $(ANNLIB)

#-----------------------------------------------------------------------------
# Make object files
#-----------------------------------------------------------------------------

$(OBJDIR)ANN.o: ANN.cpp
	$(GPP) -c -I$(INCDIR) $(CFLAGS) ANN.cpp -o $(OBJDIR)ANN.o

$(OBJDIR)brute.o: brute.cpp
	$(GPP) -c -I$(INCDIR) $(CFLAGS) brute.cpp -o $(OBJDIR)brute.o

$(OBJDIR)kd_tree.o: kd_tree.cpp
	$(GPP) -c -I$(INCDIR) $(CFLAGS) kd_tree.cpp -o $(OBJDIR)kd_tree.o

$(OBJDIR)kd_util.o: kd_util.cpp
	$(GPP) -c -I$(INCDIR) $(CFLAGS) kd_util.cpp -o $(OBJDIR)kd_util.o

$(OBJDIR)kd_split.o: kd_split.cpp
	$(GPP) -c -I$(INCDIR) $(CFLAGS) kd_split.cpp -o $(OBJDIR)kd_split.o

$(OBJDIR)kd_search.o: kd_search.cpp
	$(GPP) -c -I$(INCDIR) $(CFLAGS) kd_search.cpp -o $(OBJDIR)kd_search.o

$(OBJDIR)kd_pr_search.o: kd_pr_search.cpp
	$(GPP) -c -I$(INCDIR) $(CFLAGS) kd_pr_search.cpp -o $(OBJDIR)kd_pr_search.o

$(OBJDIR)kd_fix_rad_search.o: kd_fix_rad_search.cpp
	$(GPP) -c -I$(INCDIR) $(CFLAGS) kd_fix_rad_search.cpp -o $(OBJDIR)kd_fix_rad_search.o

$(OBJDIR)kd_dump.o: kd_dump.cpp
	$(GPP) -c -I$(INCDIR) $(CFLAGS) kd_dump.cpp -o $(OBJDIR)kd_dump.o

$(OBJDIR)bd_tree.o: bd_tree.cpp
	$(GPP) -c -I$(INCDIR) $(CFLAGS) bd_tree.cpp -o $(OBJDIR)bd_tree.o

$(OBJDIR)bd_search.o: bd_search.cpp
	$(GPP) -c -I$(INCDIR) $(CFLAGS) bd_search.cpp -o $(OBJDIR)bd_search.o

$(OBJDIR)bd_pr_search.o: bd_pr_search.cpp
	$(GPP) -c -I$(INCDIR) $(CFLAGS) bd_pr_search.cpp -o $(OBJDIR)bd_pr_search.o

$(OBJDIR)bd_fix_rad_search.o: bd_fix_rad_search.cpp
	$(GPP) -c -I$(INCDIR) $(CFLAGS) bd_fix_rad_search.cpp -o $(OBJDIR)bd_fix_rad_search.o

$(OBJDIR)perf.o: perf.cpp
	$(GPP) -c -I$(INCDIR) $(CFLAGS) perf.cpp -o $(OBJDIR)perf.o

#-----------------------------------------------------------------------------
# Cleaning
#-----------------------------------------------------------------------------

clean:
	-rm -f *.o core

realclean: clean

