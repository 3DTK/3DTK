# This Makefile is maintained manually

include Makefile.master

BIN     = bin/
OBJ     = obj/
SRC     = src/
SHOWSRC = src/show/
GRIDSRC = src/grid/
DOC     = doc/

all: $(BIN)slam6D $(BIN)scan_io_uos.so $(BIN)scan_io_uos_map.so $(BIN)scan_io_old.so $(BIN)scan_io_rts.so $(BIN)scan_io_iais.so $(BIN)scan_io_rts_map.so $(BIN)scan_io_front.so $(BIN)scan_io_riegl.so $(BIN)scan_io_zuf.so $(BIN)scan_io_xyz.so $(BIN)scan_io_ifp.so $(BIN)scan_io_ply.so $(BIN)scan_io_wrl.so $(BIN)scan_io_zahn.so $(BIN)show $(BIN)2DGridder #$(BIN)convergence $(BIN)frame_to_graph $(BIN)graph_balancer

it:
	@echo
	@echo "Tag 'it:' shouldn't be needed if the Makefile were correct... :-)"
	@echo
	make clean && make

docu: docu_html docu_latex docu_hl
	echo
	echo
	echo + Reference documentation generated: $(DOC)html/index.html
	echo + Reference documentation generated: $(DOC)refman.pdf
	echo + Highlevel documentation generated: $(DOC)documentation_HL.pdf
	echo

$(BIN)frame_to_graph: $(SRC)frame_to_graph.cc $(SRC)globals.icc
	echo Compiling and linking Frame_to_graph ...
	$(GPP) $(CFLAGS) -o $(BIN)frame_to_graph $(SRC)frame_to_graph.cc 
	echo DONE
	echo

$(BIN)convergence: $(SRC)convergence.cc $(SRC)convergence.h $(SRC)globals.icc
	echo Compiling and linking Convergence ...
	$(GPP) $(CFLAGS) -o $(BIN)convergence $(SRC)convergence.cc $(SRC)convergence.h
	echo DONE
	echo

$(BIN)graph_balancer: $(OBJ)elch6D.o $(SRC)graph_balancer.cc 
	echo Compiling and linking Graph Balancer ...
	$(GPP) $(CFLAGS) -lboost_graph-mt -o $(BIN)graph_balancer $(SRC)graph_balancer.cc $(OBJ)elch6D.o 
	echo DONE
	echo

$(BIN)show: $(SHOWSRC)glui/libglui.a $(SHOWSRC)show.cc $(SHOWSRC)show.h $(SHOWSRC)show.icc $(SHOWSRC)show1.icc $(SHOWSRC)show_menu.cc $(SHOWSRC)show_gl.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc $(OBJ)scan.o $(OBJ)vertexarray.o $(OBJ)camera.o $(OBJ)PathGraph.o $(OBJ)NurbsPath.o $(OBJ)scanlib.a
	echo Compiling and Linking Show ...
	$(GPP) $(CFLAGS) -o $(BIN)show -I$(SRC) $(SHOWSRC)show.cc $(OBJ)scanlib.a $(OBJ)vertexarray.o $(OBJ)camera.o $(OBJ)PathGraph.o $(OBJ)NurbsPath.o $(SHOWSRC)glui/libglui.a $(LIBRARIES)
	echo DONE
	echo

$(OBJ)vertexarray.o: $(SHOWSRC)vertexarray.h $(SHOWSRC)vertexarray.cc
	echo Compiling VertexArray ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)vertexarray.o $(SHOWSRC)vertexarray.cc

$(OBJ)camera.o: $(SHOWSRC)camera.h $(SHOWSRC)camera.cc $(SRC)globals.icc
	echo Compiling Camera ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)camera.o $(SHOWSRC)camera.cc

$(OBJ)NurbsPath.o: $(SHOWSRC)NurbsPath.h $(SHOWSRC)NurbsPath.cc $(SRC)globals.icc $(OBJ)PathGraph.o
	echo Compiling NurbsPath ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)NurbsPath.o $(SHOWSRC)NurbsPath.cc

$(OBJ)PathGraph.o: $(SHOWSRC)PathGraph.h $(SHOWSRC)PathGraph.cc $(SRC)globals.icc
	echo Compiling PathGraph ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)PathGraph.o $(SHOWSRC)PathGraph.cc


$(SHOWSRC)glui/libglui.a: $(SHOWSRC)glui/*.c $(SHOWSRC)glui/*.cpp $(SHOWSRC)glui/*.h
	echo Compiling Glui ...
	cd $(SHOWSRC)glui ; make

$(SRC)newmat/libnewmat.a: $(SRC)newmat/*.cpp $(SRC)newmat/*.h
	echo Compiling Newmat ...
	cd $(SRC)newmat ; make

$(BIN)slam6D: $(OBJ)scanlib.a $(OBJ)icp6D.o $(OBJ)graphSlam6D.o $(OBJ)icp6Dapx.o $(OBJ)icp6Dsvd.o $(OBJ)icp6Dortho.o $(OBJ)icp6Dquat.o $(OBJ)icp6Dhelix.o $(OBJ)gapx6D.o $(OBJ)ghelix6DQ2.o $(OBJ)lum6Deuler.o $(OBJ)lum6Dquat.o $(OBJ)graph.o $(SRC)slam6D.cc $(SRC)globals.icc $(OBJ)csparse.o $(SRC)newmat/libnewmat.a $(OBJ)elch6D.o $(OBJ)elch6Dquat.o $(OBJ)elch6DunitQuat.o $(OBJ)elch6Dslerp.o $(OBJ)elch6Deuler.o
	echo Compiling and Linking SLAM 6D ...
	$(GPP) $(CFLAGS) -o $(BIN)slam6D $(SRC)slam6D.cc $(OBJ)scanlib.a $(OBJ)icp6D.o $(OBJ)gapx6D.o $(OBJ)ghelix6DQ2.o $(OBJ)lum6Deuler.o $(OBJ)lum6Dquat.o $(OBJ)graphSlam6D.o $(OBJ)graph.o $(OBJ)icp6Dapx.o $(OBJ)icp6Dsvd.o $(OBJ)icp6Dortho.o $(OBJ)icp6Dquat.o $(OBJ)icp6Dhelix.o $(OBJ)csparse.o $(SRC)newmat/libnewmat.a $(OBJ)elch6D.o $(OBJ)elch6Dquat.o $(OBJ)elch6DunitQuat.o $(OBJ)elch6Dslerp.o $(OBJ)elch6Deuler.o -ldl 
	echo DONE
	echo

$(OBJ)kdc.o: $(SRC)kdcache.h $(SRC)searchCache.h $(SRC)searchTree.h $(SRC)kdc.h $(SRC)kdc.cc $(SRC)globals.icc
	echo Compiling KD tree with cache...
	$(GPP) $(CFLAGS) -c -o $(OBJ)kdc.o $(SRC)kdc.cc 

$(OBJ)kd.o: $(SRC)searchTree.h $(SRC)kd.h $(SRC)kd.cc $(SRC)globals.icc
	echo Compiling KD tree ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)kd.o $(SRC)kd.cc 

$(OBJ)octtree.o: $(SRC)octtree.h $(SRC)octtree.cc $(SRC)globals.icc
	echo Compiling Octree ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)octtree.o $(SRC)octtree.cc 

$(OBJ)d2tree.o: $(SRC)d2tree.h $(SRC)d2tree.cc $(SRC)searchTree.h $(OBJ)octtree.o $(SRC)globals.icc
	echo Compiling D2tree ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)d2tree.o $(SRC)d2tree.cc 

$(OBJ)scan.o: $(OBJ)octtree.o $(OBJ)kd.o $(OBJ)kdc.o $(SRC)scan.h $(SRC)scan_io.h $(SRC)scan.cc $(SRC)scan.icc $(SRC)globals.icc $(SRC)point.h $(SRC)ptpair.h $(SRC)point.icc $(OBJ)d2tree.o
	echo Compiling Scan ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)scan.o $(SRC)scan.cc 

$(OBJ)scanlib.a: $(OBJ)octtree.o $(OBJ)kd.o $(OBJ)kdc.o $(OBJ)scan.o $(OBJ)d2tree.o $(OBJ)octtree.o
	echo Linking Scanlib ...
	$(AR) -cr $(OBJ)scanlib.a $(OBJ)scan.o $(OBJ)octtree.o $(OBJ)kd.o $(OBJ)kdc.o $(OBJ)d2tree.o
	ranlib $(OBJ)scanlib.a

$(OBJ)icp6D.o: $(OBJ)kd.o $(OBJ)kdc.o $(OBJ)scan.o $(SRC)icp6D.h $(SRC)icp6D.cc $(SRC)ptpair.h $(SRC)globals.icc $(SRC)icp6Dminimizer.h $(SRC)newmat/libnewmat.a
	echo Compiling ICP 6D ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)icp6D.o $(SRC)icp6D.cc 

$(OBJ)graphSlam6D.o: $(OBJ)icp6D.o $(OBJ)graph.o $(SRC)globals.icc $(SRC)graphSlam6D.h $(SRC)graphSlam6D.cc $(SRC)newmat/libnewmat.a $(OBJ)csparse.o
	echo Compiling GraphSlam6D ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)graphSlam6D.o $(SRC)graphSlam6D.cc

$(OBJ)gapx6D.o: $(OBJ)icp6D.o $(OBJ)graph.o $(SRC)globals.icc $(SRC)gapx6D.cc $(SRC)gapx6D.h $(OBJ)graphSlam6D.o $(SRC)newmat/libnewmat.a $(OBJ)csparse.o
	echo Compiling Global APX 6D ...
	$(GPP) $(CFLAGS) -DUSE_C_SPARSE -c -o $(OBJ)gapx6D.o $(SRC)gapx6D.cc 

$(OBJ)ghelix6DQ2.o: $(OBJ)icp6D.o $(OBJ)graph.o $(SRC)globals.icc $(SRC)ghelix6DQ2.cc $(SRC)ghelix6DQ2.h $(OBJ)graphSlam6D.o $(SRC)newmat/libnewmat.a $(OBJ)csparse.o
	echo Compiling global HELIX 6D Q2 ...
	$(GPP) $(CFLAGS) -DUSE_C_SPARSE -c -o $(OBJ)ghelix6DQ2.o $(SRC)ghelix6DQ2.cc 

$(OBJ)lum6Deuler.o: $(OBJ)icp6D.o $(OBJ)graph.o $(SRC)globals.icc $(SRC)lum6Deuler.h $(SRC)lum6Deuler.cc $(OBJ)graphSlam6D.o $(SRC)newmat/libnewmat.a $(OBJ)csparse.o
	echo Compiling LUM 6D Euler  ...
	$(GPP) $(CFLAGS) -DUSE_C_SPARSE -c -o $(OBJ)lum6Deuler.o $(SRC)lum6Deuler.cc 

$(OBJ)lum6Dquat.o: $(OBJ)icp6D.o $(OBJ)graph.o $(SRC)globals.icc $(SRC)lum6Dquat.h $(SRC)lum6Dquat.cc $(OBJ)graphSlam6D.o $(SRC)newmat/libnewmat.a $(OBJ)csparse.o
	echo Compiling LUM 6D Quaternion ...
	$(GPP) $(CFLAGS) -DUSE_C_SPARSE -c -o $(OBJ)lum6Dquat.o $(SRC)lum6Dquat.cc 

$(OBJ)elch6D.o: $(SRC)icp6Dminimizer.h $(SRC)elch6D.h $(SRC)elch6D.cc $(SRC)loopSlam6D.h
	echo Compiling ELCH 6D ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)elch6D.o $(SRC)elch6D.cc 

$(OBJ)elch6Deuler.o: $(SRC)icp6D.h $(SRC)icp6Dminimizer.h $(SRC)graph.h $(SRC)elch6Deuler.h $(SRC)elch6Deuler.cc $(SRC)elch6D.h
	echo Compiling ELCH 6D Euler ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)elch6Deuler.o $(SRC)elch6Deuler.cc 

$(OBJ)elch6Dquat.o: $(SRC)icp6D.h $(SRC)icp6Dminimizer.h $(SRC)graph.h $(SRC)elch6Dquat.h $(SRC)elch6Dquat.cc $(SRC)elch6D.h
	echo Compiling ELCH 6D Quaternion ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)elch6Dquat.o $(SRC)elch6Dquat.cc 

$(OBJ)elch6DunitQuat.o: $(SRC)icp6D.h $(SRC)icp6Dminimizer.h $(SRC)graph.h $(SRC)elch6DunitQuat.h $(SRC)elch6DunitQuat.cc $(SRC)elch6D.h
	echo Compiling ELCH 6D Unit Quaternion ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)elch6DunitQuat.o $(SRC)elch6DunitQuat.cc 

$(OBJ)elch6Dslerp.o: $(SRC)icp6D.h $(SRC)icp6Dminimizer.h $(SRC)graph.h $(SRC)elch6Dslerp.h $(SRC)elch6Dslerp.cc $(SRC)elch6D.h
	echo Compiling ELCH 6D SLERP Quaternion ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)elch6Dslerp.o $(SRC)elch6Dslerp.cc 

$(OBJ)icp6Dapx.o: $(SRC)icp6Dapx.h $(SRC)icp6Dapx.cc $(SRC)ptpair.h $(SRC)icp6Dminimizer.h
	echo Compiling ICP 6D with Approximation ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)icp6Dapx.o $(SRC)icp6Dapx.cc 

$(OBJ)icp6Dsvd.o: $(SRC)icp6Dsvd.h $(SRC)icp6Dsvd.cc $(SRC)ptpair.h $(SRC)icp6Dminimizer.h 
	echo Compiling ICP 6D with SVD ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)icp6Dsvd.o $(SRC)icp6Dsvd.cc 

$(OBJ)icp6Dortho.o: $(SRC)icp6Dortho.h $(SRC)icp6Dortho.cc $(SRC)ptpair.h $(SRC)icp6Dminimizer.h 
	echo Compiling ICP 6D with Orthonormal Matrices ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)icp6Dortho.o $(SRC)icp6Dortho.cc 

$(OBJ)icp6Dquat.o: $(SRC)icp6Dquat.h $(SRC)icp6Dquat.cc $(SRC)ptpair.h $(SRC)icp6Dminimizer.h
	echo Compiling ICP 6D with Quaternion ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)icp6Dquat.o $(SRC)icp6Dquat.cc 

$(OBJ)icp6Dhelix.o: $(SRC)icp6Dhelix.h $(SRC)icp6Dhelix.cc $(SRC)ptpair.h $(SRC)icp6Dminimizer.h
	echo Compiling ICP 6D with Helix ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)icp6Dhelix.o $(SRC)icp6Dhelix.cc

$(OBJ)graph.o: $(SRC)graph.h $(SRC)graph.cc $(SRC)globals.icc $(OBJ)scan.o
	echo Compiling Graph ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)graph.o $(SRC)graph.cc

$(OBJ)csparse.o: $(SRC)sparse/csparse.h $(SRC)sparse/csparse.cc
	echo Compiling Csparse ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)csparse.o $(SRC)sparse/csparse.cc

docu_html:
	doxygen doc/doxygen.cfg
	cd $(DOC) ; zip -q html.zip html/*
	echo
	echo

docu_latex:
	cd $(DOC)latex ; make
	cd $(DOC)latex ; dvips refman
	cd $(DOC)latex ; ps2pdf14 refman.ps refman.pdf
	cp $(DOC)latex/refman.pdf $(DOC)

docu_hl:	$(DOC)high_level_doc/documentation.tex
	cd $(DOC)high_level_doc ; latex documentation.tex
	cd $(DOC)high_level_doc ; bibtex documentation
	cd $(DOC)high_level_doc ; latex documentation.tex
	cd $(DOC)high_level_doc ; dvips documentation
	cd $(DOC)high_level_doc ; ps2pdf14 documentation.ps ../documentation_HL.pdf


##################################################################################

$(BIN)scan_io_uos.so: $(SRC)scan_io.h $(SRC)scan_io_uos.h $(SRC)scan_io_uos.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading UOS scans ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_uos.so $(SRC)scan_io_uos.cc 

$(BIN)scan_io_uos_map.so: $(SRC)scan_io.h $(SRC)scan_io_uos_map.h $(SRC)scan_io_uos_map.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading UOS scans with given map ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_uos_map.so $(SRC)scan_io_uos_map.cc 

$(BIN)scan_io_old.so: $(SRC)scan_io.h $(SRC)scan_io_old.h $(SRC)scan_io_old.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading old UOS scans ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_old.so $(SRC)scan_io_old.cc 

$(BIN)scan_io_rts.so: $(SRC)scan_io.h $(SRC)scan_io_rts.h $(SRC)scan_io_rts.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading RTS scans ...
	$(GPP) $(CFLAGS)  $(SHAREDFLAGS) -o $(BIN)scan_io_rts.so $(SRC)scan_io_rts.cc 

$(BIN)scan_io_iais.so: $(SRC)scan_io.h $(SRC)scan_io_iais.h $(SRC)scan_io_iais.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading IAIS scans ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_iais.so $(SRC)scan_io_iais.cc 

$(BIN)scan_io_rts_map.so: $(SRC)scan_io.h $(SRC)scan_io_rts_map.h $(SRC)scan_io_rts_map.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading RTS scans with given map ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_rts_map.so $(SRC)scan_io_rts_map.cc 

$(BIN)scan_io_riegl.so: $(SRC)scan_io.h $(SRC)scan_io_riegl.h $(SRC)scan_io_riegl.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading Riegl scans ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_riegl.so $(SRC)scan_io_riegl.cc 

$(BIN)scan_io_zuf.so: $(SRC)scan_io.h $(SRC)scan_io_zuf.h $(SRC)scan_io_zuf.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading Z+F scans ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_zuf.so $(SRC)scan_io_zuf.cc 

$(BIN)scan_io_ifp.so: $(SRC)scan_io.h $(SRC)scan_io_ifp.h $(SRC)scan_io_ifp.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading IFP scans ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_ifp.so $(SRC)scan_io_ifp.cc 

$(BIN)scan_io_ply.so: $(SRC)scan_io.h $(SRC)scan_io_ply.h $(SRC)scan_io_ply.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading PLY scans ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_ply.so $(SRC)scan_io_ply.cc 

$(BIN)scan_io_xyz.so: $(SRC)scan_io.h $(SRC)scan_io_xyz.h $(SRC)scan_io_xyz.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading XYZ scans ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_xyz.so $(SRC)scan_io_xyz.cc 

$(BIN)scan_io_wrl.so: $(SRC)scan_io.h $(SRC)scan_io_wrl.h $(SRC)scan_io_wrl.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading VRML v1.0 scans ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_wrl.so $(SRC)scan_io_wrl.cc 

$(BIN)scan_io_zahn.so: $(SRC)scan_io.h $(SRC)scan_io_zahn.h $(SRC)scan_io_zahn.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading Zaehne scans ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_zahn.so $(SRC)scan_io_zahn.cc 
	echo DONE
	echo

$(BIN)scan_io_front.so: $(SRC)scan_io.h $(SRC)scan_io_front.h $(SRC)scan_io_front.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading 2D Front scans ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_front.so $(SRC)scan_io_front.cc 
	echo DONE
	echo


######## GRID

$(OBJ)gridder.o: $(GRIDSRC)2DGridder.cc $(OBJ)scanmanager.o $(OBJ)scanGrid.o $(OBJ)scanToGrid.o $(OBJ)parcelmanager.o $(OBJ)gridlines.o
	echo Compiling 2Dgridder ...
	$(GPP) $(CFLAGS) -c $(GRIDSRC)2DGridder.cc -o $(OBJ)gridder.o

$(OBJ)grid.o: $(GRIDSRC)grid.cc $(GRIDSRC)grid.h $(OBJ)gridPoint.o
	echo Compiling Grid ...
	$(GPP) $(CFLAGS) -c $(GRIDSRC)grid.cc -o $(OBJ)grid.o

$(OBJ)scanGrid.o: $(GRIDSRC)scanGrid.cc $(GRIDSRC)scanGrid.h $(OBJ)grid.o
	echo Compiling ScanGrid ...
	$(GPP) $(CFLAGS) -c $(GRIDSRC)scanGrid.cc -o $(OBJ)scanGrid.o

$(OBJ)scanToGrid.o: $(GRIDSRC)scanToGrid.cc $(GRIDSRC)scanToGrid.h
	echo Compiling ScanToGrid ...
	$(GPP) $(CFLAGS) -c $(GRIDSRC)scanToGrid.cc -o $(OBJ)scanToGrid.o

$(OBJ)gridPoint.o: $(GRIDSRC)gridPoint.cc $(GRIDSRC)gridPoint.h
	echo Compiling GridPoint ...
	$(GPP) $(CFLAGS) -c $(GRIDSRC)gridPoint.cc -o $(OBJ)gridPoint.o

$(OBJ)scanmanager.o: $(GRIDSRC)scanmanager.cc $(GRIDSRC)scanmanager.h
	echo Compiling Scanmanager ...
	$(GPP) $(CFLAGS) -c $(GRIDSRC)scanmanager.cc -o $(OBJ)scanmanager.o

$(OBJ)parcel.o: $(GRIDSRC)parcel.cc $(GRIDSRC)parcel.h $(OBJ)grid.o
	echo Compiling Parcel ...
	$(GPP) $(CFLAGS) -c $(GRIDSRC)parcel.cc -o $(OBJ)parcel.o

$(OBJ)parcelmanager.o: $(GRIDSRC)parcelmanager.cc $(GRIDSRC)parcelmanager.h
	echo Compiling Parcelmanager ...
	$(GPP) $(CFLAGS) -c $(GRIDSRC)parcelmanager.cc -o $(OBJ)parcelmanager.o

$(OBJ)parcelinfo.o: $(GRIDSRC)parcelinfo.cc $(GRIDSRC)parcelinfo.h
	echo Compiling Parcelinfo ...
	$(GPP) $(CFLAGS) -c $(GRIDSRC)parcelinfo.cc -o $(OBJ)parcelinfo.o

$(OBJ)gridWriter.o: $(GRIDSRC)gridWriter.h $(GRIDSRC)gridWriter.cc
	echo Compiling GridWriter ...
	$(GPP) $(CFLAGS) -c $(GRIDSRC)gridWriter.cc -o $(OBJ)gridWriter.o

$(OBJ)viewpointinfo.o: $(GRIDSRC)viewpointinfo.h $(GRIDSRC)viewpointinfo.cc $(OBJ)scanGrid.o
	echo Compiling Viewpointinfo ...
	$(GPP) $(CFLAGS) -c $(GRIDSRC)viewpointinfo.cc -o $(OBJ)viewpointinfo.o

$(OBJ)line.o: $(GRIDSRC)line.cc $(GRIDSRC)line.h $(OBJ)gridPoint.o
	echo Compiling Line ...
	$(GPP) $(CFLAGS) -c $(GRIDSRC)line.cc -o $(OBJ)line.o

$(OBJ)gridlines.o: $(GRIDSRC)gridlines.cc $(GRIDSRC)gridlines.h $(OBJ)hough.o $(OBJ)line.o $(OBJ)grid.o
	echo Compiling Gridlines ...
	$(GPP) $(CFLAGS) -c $(GRIDSRC)gridlines.cc -o $(OBJ)gridlines.o

$(OBJ)hough.o: $(GRIDSRC)hough.cc $(GRIDSRC)hough.h
	echo Compiling Hough ...
	$(GPP) $(CFLAGS) -c $(GRIDSRC)hough.cc -o $(OBJ)hough.o

$(BIN)2DGridder: $(OBJ)gridder.o $(OBJ)line.o $(OBJ)gridlines.o $(OBJ)hough.o $(OBJ)viewpointinfo.o $(OBJ)gridWriter.o $(OBJ)parcelmanager.o $(OBJ)parcel.o $(OBJ)parcelinfo.o $(OBJ)scanGrid.o $(OBJ)grid.o $(OBJ)scanToGrid.o $(OBJ)gridPoint.o $(OBJ)scan.o $(OBJ)scanmanager.o $(OBJ)kd.o $(OBJ)kdc.o
	echo Compiling and Linking Grid ...
	$(GPP) $(CFLAGS) -o $(BIN)2DGridder $(OBJ)viewpointinfo.o $(OBJ)line.o $(OBJ)gridlines.o $(OBJ)hough.o $(OBJ)gridder.o $(OBJ)gridWriter.o $(OBJ)parcelmanager.o $(OBJ)parcelinfo.o $(OBJ)scanmanager.o $(OBJ)grid.o $(OBJ)scanGrid.o $(OBJ)parcel.o $(OBJ)gridPoint.o $(OBJ)scanToGrid.o $(OBJ)scan.o $(OBJ)octtree.o $(OBJ)kd.o $(OBJ)kdc.o -ldl  -lstdc++
	echo DONE
	echo

##################################################################################

svn_clean: # "are you sure?"-version
	@find . -name '.svn'         | xargs zip .svn.zip -r -q -m

svn_clean_del:
	@find . -name '.svn'         | xargs rm -r -f

clean:	
	/bin/rm -f $(OBJ)*
	/bin/rm -f $(BIN)*.so
	cd $(SRC)newmat ; make clean
	rm -f $(SRC)newmat/libnewmat.a
	cd $(SHOWSRC)glui ; make clean
	rm -f $(SHOWSRC)glui/libglui.a
	cd $(DOC)high_level_doc ; make clean
#	cd $(DOC)latex ; make clean
