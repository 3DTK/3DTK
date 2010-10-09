# This Makefile is maintained manually

include Makefile.options

BIN     = bin/
OBJ     = obj/
SRC     = src/
SHOWSRC = src/show/
GRIDSRC = src/grid/
PMDSRC  = src/pmd/
APSSRC  = src/sift/autopano-sift-c/
SIFTSRC = src/sift/
DOC     = doc/

TARGETS = $(BIN)slam6D $(BIN)scan_io_uos.so $(BIN)scan_io_rxp.so $(BIN)scan_io_uos_map.so $(BIN)scan_io_uos_frames.so $(BIN)scan_io_uos_map_frames.so $(BIN)scan_io_old.so $(BIN)scan_io_x3d.so $(BIN)scan_io_asc.so $(BIN)scan_io_rts.so $(BIN)scan_io_iais.so $(BIN)scan_io_rts_map.so $(BIN)scan_io_front.so $(BIN)scan_io_riegl_txt.so $(BIN)scan_io_riegl_bin.so $(BIN)scan_io_zuf.so $(BIN)scan_io_xyz.so $(BIN)scan_io_ifp.so $(BIN)scan_io_ply.so $(BIN)scan_io_wrl.so $(BIN)scan_io_zahn.so 

ifdef WITH_SCANRED
TARGETS += $(BIN)scan_red
endif

ifdef WITH_SCANDIFF
TARGETS += $(BIN)scan_diff
endif

ifdef WITH_SHOW
TARGETS += $(BIN)show
endif

ifdef WITH_GRIDDER
TARGETS += $(BIN)2DGridder
endif

ifdef WITH_TOOLS
TARGETS += $(BIN)convergence $(BIN)frame_to_graph $(BIN)graph_balancer $(BIN)exportPoints
endif

ifdef WITH_PMD
TARGETS += $(BIN)grabVideoAnd3D $(BIN)grabFramesCam $(BIN)grabFramesPMD $(BIN)convertToSLAM6D $(BIN)calibrate $(BIN)extrinsic $(BIN)pose
endif

ifdef WITH_SIFT
TARGETS += $(OBJ)libANN.a $(BIN)autopano $(BIN)autopano-sift-c $(BIN)generatesiftfeatures $(BIN)mergehistograms $(BIN)panoramacreator $(BIN)visualizemap $(BIN)visualizescan $(BIN)reduceppc $(BIN)matchsiftfeatures $(BIN)registerscans $(BIN)readscan $(BIN)visualizematches $(BIN)visualizeregistrations
endif

ifdef WITH_TORO
TARGETS += $(BIN)toro3d
endif

ifdef WITH_HOGMAN
TARGETS += $(BIN)hogman3d
endif

all: $(OBJ) $(TARGETS)

it:
	@echo
	@echo "Tag 'it:' shouldn't be needed if the Makefile were correct... :-)"
	@echo
	$(MAKE) clean && $(MAKE)

docu: docu_html docu_latex docu_hl
	echo
	echo
	echo + Reference documentation generated: $(DOC)html/index.html
	echo + Reference documentation generated: $(DOC)refman.pdf
	echo + Highlevel documentation generated: $(DOC)documentation_HL.pdf
	echo

$(OBJ):
	@mkdir $@

############# SLAM6D ##############

ifdef WITH_CUDA
$(BIN)slam6D: $(OBJ)libglui.a $(OBJ)scanlib.a $(OBJ)icp6D.o $(OBJ)graphSlam6D.o $(OBJ)icp6Dapx.o $(OBJ)icp6Dsvd.o $(OBJ)icp6Dortho.o $(OBJ)icp6Dquat.o $(OBJ)icp6Dhelix.o $(OBJ)icp6Ddual.o $(OBJ)icp6Dlumeuler.o $(OBJ)icp6Dlumquat.o $(OBJ)gapx6D.o $(OBJ)ghelix6DQ2.o $(OBJ)lum6Deuler.o $(OBJ)lum6Dquat.o $(OBJ)graph.o $(SRC)slam6D.cc $(SRC)globals.icc $(OBJ)csparse.o $(OBJ)libnewmat.a $(OBJ)elch6D.o $(OBJ)elch6Dquat.o $(OBJ)elch6DunitQuat.o $(OBJ)elch6Dslerp.o $(OBJ)elch6Deuler.o $(OBJ)loopToro.o $(OBJ)loopHOG-Man.o $(OBJ)graphToro.o $(OBJ)graphHOG-Man.o $(OBJ)CIcpGpuCuda.o $(OBJ)icp6Dcuda.o $(OBJ)libANN.a
	echo Compiling and Linking SLAM 6D with CUDA ...
	$(GPP) $(CFLAGS) -DWITH_CUDA  -I$(SRC)ann_1.1.1_modified/include/ -I$(SRC)ann_1.1.1_modified/src/ -I$(SRC) -o $(BIN)slam6D $(SRC)slam6D.cc $(OBJ)scanlib.a $(OBJ)icp6D.o $(OBJ)gapx6D.o $(OBJ)ghelix6DQ2.o $(OBJ)lum6Deuler.o $(OBJ)lum6Dquat.o $(OBJ)graphSlam6D.o $(OBJ)graph.o $(OBJ)icp6Dapx.o $(OBJ)icp6Dsvd.o $(OBJ)icp6Dortho.o $(OBJ)icp6Dquat.o $(OBJ)icp6Dhelix.o $(OBJ)icp6Ddual.o $(OBJ)icp6Dlumeuler.o $(OBJ)icp6Dlumquat.o $(OBJ)csparse.o $(OBJ)libnewmat.a $(OBJ)elch6D.o $(OBJ)elch6Dquat.o $(OBJ)elch6DunitQuat.o $(OBJ)elch6Dslerp.o $(OBJ)elch6Deuler.o -ldl $(OBJ)libglui.a $(OBJ)loopToro.o $(OBJ)loopHOG-Man.o $(OBJ)graphToro.o $(OBJ)graphHOG-Man.o $(OBJ)CIcpGpuCuda.o $(OBJ)icp6Dcuda.o $(OBJ)libANN.a $(LIBRARIES) $(CUDALIBDIRS) $(CUDALIBS)
else
$(BIN)slam6D: $(OBJ)libglui.a $(OBJ)scanlib.a $(OBJ)icp6D.o $(OBJ)graphSlam6D.o $(OBJ)icp6Dapx.o $(OBJ)icp6Dsvd.o $(OBJ)icp6Dortho.o $(OBJ)icp6Dquat.o $(OBJ)icp6Dhelix.o $(OBJ)icp6Ddual.o $(OBJ)icp6Dlumeuler.o $(OBJ)icp6Dlumquat.o $(OBJ)gapx6D.o $(OBJ)ghelix6DQ2.o $(OBJ)lum6Deuler.o $(OBJ)lum6Dquat.o $(OBJ)graph.o $(SRC)slam6D.cc $(SRC)globals.icc $(OBJ)csparse.o $(OBJ)libnewmat.a $(OBJ)elch6D.o $(OBJ)elch6Dquat.o $(OBJ)elch6DunitQuat.o $(OBJ)elch6Dslerp.o $(OBJ)elch6Deuler.o $(OBJ)loopToro.o $(OBJ)loopHOG-Man.o $(OBJ)graphToro.o $(OBJ)graphHOG-Man.o $(OBJ)libANN.a
	echo Compiling and Linking SLAM 6D ...
	$(GPP) $(CFLAGS) -I$(SRC)ann_1.1.1_modified/include/ -o $(BIN)slam6D $(SRC)slam6D.cc $(OBJ)scanlib.a $(OBJ)icp6D.o $(OBJ)gapx6D.o $(OBJ)ghelix6DQ2.o $(OBJ)lum6Deuler.o $(OBJ)lum6Dquat.o $(OBJ)graphSlam6D.o $(OBJ)graph.o $(OBJ)icp6Dapx.o $(OBJ)icp6Dsvd.o $(OBJ)icp6Dortho.o $(OBJ)icp6Dquat.o $(OBJ)icp6Dhelix.o $(OBJ)icp6Ddual.o $(OBJ)icp6Dlumeuler.o $(OBJ)icp6Dlumquat.o $(OBJ)csparse.o $(OBJ)libnewmat.a $(OBJ)elch6D.o $(OBJ)elch6Dquat.o $(OBJ)elch6DunitQuat.o $(OBJ)elch6Dslerp.o $(OBJ)elch6Deuler.o -ldl $(OBJ)libglui.a $(OBJ)loopToro.o $(OBJ)loopHOG-Man.o $(OBJ)graphToro.o $(OBJ)graphHOG-Man.o $(OBJ)libANN.a $(LIBRARIES)
endif
	echo DONE
	echo


$(OBJ)kdc.o: $(SRC)kdcache.h $(SRC)searchCache.h $(SRC)searchTree.h $(SRC)kdc.h $(SRC)kdc.cc $(SRC)globals.icc
	echo Compiling KD tree with cache...
	$(GPP) $(CFLAGS) -c -o $(OBJ)kdc.o $(SRC)kdc.cc 

$(OBJ)kd.o: $(SRC)searchTree.h $(SRC)kd.h $(SRC)kd.cc $(SRC)globals.icc
	echo Compiling KD tree ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)kd.o $(SRC)kd.cc 


$(OBJ)d2tree.o: $(SRC)d2tree.h $(SRC)d2tree.cc $(SRC)searchTree.h $(SRC)globals.icc
	echo Compiling D2tree ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)d2tree.o $(SRC)d2tree.cc 

$(OBJ)scan.o: $(SRC)Boctree.h $(SRC)kd.h $(SRC)kdc.h $(SRC)kdcache.h $(SRC)scan.h $(SRC)scan_io.h $(SRC)scan.cc $(SRC)scan.icc $(SRC)globals.icc $(SRC)point.h $(SRC)ptpair.h $(SRC)point.icc $(SRC)d2tree.h
	echo Compiling Scan ...
ifdef WITH_CUDA
	$(GPP) -DWITH_CUDA $(CFLAGS) -I$(SRC)ann_1.1.1_modified/include/ -c -o $(OBJ)scan.o $(SRC)scan.cc 
else
	$(GPP) $(CFLAGS) -I$(SRC)ann_1.1.1_modified/include/ -c -o $(OBJ)scan.o $(SRC)scan.cc
endif

$(OBJ)scanlib.a: $(OBJ)kd.o $(OBJ)kdc.o $(OBJ)scan.o $(OBJ)d2tree.o
	echo Linking Scanlib ...
	$(AR) -cr $(OBJ)scanlib.a $(OBJ)scan.o $(OBJ)kd.o $(OBJ)kdc.o $(OBJ)d2tree.o
	ranlib $(OBJ)scanlib.a

$(OBJ)icp6D.o: $(SRC)kd.h $(SRC)kdc.h $(SRC)scan.h $(SRC)icp6D.h $(SRC)icp6D.cc $(SRC)ptpair.h $(SRC)globals.icc $(SRC)icp6Dminimizer.h $(SRC)newmat/newmat.h
	echo Compiling ICP 6D ...
	$(GPP) $(CFLAGS) -I$(SRC)ann_1.1.1_modified/include/ -c -o $(OBJ)icp6D.o $(SRC)icp6D.cc 

$(OBJ)graphSlam6D.o: $(SRC)icp6D.h $(SRC)graph.h $(SRC)globals.icc $(SRC)graphSlam6D.h $(SRC)graphSlam6D.cc $(SRC)newmat/newmat.h $(SRC)sparse/csparse.h
	echo Compiling GraphSlam6D ...
	$(GPP) $(CFLAGS) -I$(SRC)ann_1.1.1_modified/include/ -c -o $(OBJ)graphSlam6D.o $(SRC)graphSlam6D.cc

$(OBJ)gapx6D.o: $(SRC)icp6D.h $(SRC)graph.h $(SRC)globals.icc $(SRC)gapx6D.cc $(SRC)gapx6D.h $(SRC)graphSlam6D.h $(SRC)newmat/newmat.h $(SRC)sparse/csparse.h
	echo Compiling Global APX 6D ...
	$(GPP) $(CFLAGS)  -I$(SRC)ann_1.1.1_modified/include/ -DUSE_C_SPARSE -c -o $(OBJ)gapx6D.o $(SRC)gapx6D.cc 

$(OBJ)ghelix6DQ2.o: $(SRC)icp6D.h $(SRC)graph.h $(SRC)globals.icc $(SRC)ghelix6DQ2.cc $(SRC)ghelix6DQ2.h $(SRC)graphSlam6D.h $(SRC)newmat/newmat.h $(SRC)sparse/csparse.h
	echo Compiling global HELIX 6D Q2 ...
	$(GPP) $(CFLAGS)  -I$(SRC)ann_1.1.1_modified/include/ -DUSE_C_SPARSE -c -o $(OBJ)ghelix6DQ2.o $(SRC)ghelix6DQ2.cc 

$(OBJ)lum6Deuler.o: $(SRC)icp6D.h $(SRC)graph.h $(SRC)globals.icc $(SRC)lum6Deuler.h $(SRC)lum6Deuler.cc $(SRC)graphSlam6D.h $(SRC)newmat/newmat.h $(SRC)sparse/csparse.h
	echo Compiling LUM 6D Euler  ...
	$(GPP) $(CFLAGS)  -I$(SRC)ann_1.1.1_modified/include/ -DUSE_C_SPARSE -c -o $(OBJ)lum6Deuler.o $(SRC)lum6Deuler.cc 

$(OBJ)lum6Dquat.o: $(SRC)icp6D.h $(SRC)graph.h $(SRC)globals.icc $(SRC)lum6Dquat.h $(SRC)lum6Dquat.cc $(SRC)graphSlam6D.h $(SRC)newmat/newmat.h $(SRC)sparse/csparse.h
	echo Compiling LUM 6D Quaternion ...
	$(GPP) $(CFLAGS)  -I$(SRC)ann_1.1.1_modified/include/ -I$(SRC)ann_1.1.1_modified/include/ -DUSE_C_SPARSE -c -o $(OBJ)lum6Dquat.o $(SRC)lum6Dquat.cc 

$(OBJ)graphToro.o: $(SRC)graphToro.cc $(SRC)graphToro.h $(SRC)graphSlam6D.h $(SRC)icp6D.h $(SRC)graph.h $(SRC)globals.icc $(SRC)lum6Deuler.h
	echo Compiling Graph TORO  ...
	$(GPP) $(CFLAGS) -I$(SRC)ann_1.1.1_modified/include/ -c -o $(OBJ)graphToro.o $(SRC)graphToro.cc

$(OBJ)graphHOG-Man.o: $(SRC)graphHOG-Man.cc $(SRC)graphHOG-Man.h $(SRC)graphSlam6D.h $(SRC)icp6D.h $(SRC)graph.h $(SRC)globals.icc $(SRC)lum6Deuler.h
	echo Compiling Graph HOG-Man  ...
	$(GPP) $(CFLAGS)  -I$(SRC)ann_1.1.1_modified/include/ -c -o $(OBJ)graphHOG-Man.o $(SRC)graphHOG-Man.cc

$(OBJ)elch6D.o: $(SRC)elch6D.cc $(SRC)elch6D.h $(SRC)loopSlam6D.h $(SRC)icp6D.h $(SRC)icp6Dminimizer.h $(SRC)scan.h $(SRC)graph.h $(SRC)globals.icc
	echo Compiling ELCH 6D ...
	$(GPP) $(CFLAGS) -I$(SRC)ann_1.1.1_modified/include/ -c -o $(OBJ)elch6D.o $(SRC)elch6D.cc 

$(OBJ)elch6Deuler.o: $(SRC)elch6Deuler.cc $(SRC)elch6Deuler.h $(SRC)elch6D.h $(SRC)loopSlam6D.h $(SRC)icp6D.h $(SRC)icp6Dminimizer.h $(SRC)scan.h $(SRC)graph.h $(SRC)lum6Deuler.h
	echo Compiling ELCH 6D Euler ...
	$(GPP) $(CFLAGS) -I$(SRC)ann_1.1.1_modified/include/ -c -o $(OBJ)elch6Deuler.o $(SRC)elch6Deuler.cc 

$(OBJ)elch6Dquat.o: $(SRC)elch6Dquat.cc $(SRC)elch6Dquat.h $(SRC)elch6D.h $(SRC)loopSlam6D.h $(SRC)icp6D.h $(SRC)icp6Dminimizer.h $(SRC)scan.h $(SRC)graph.h $(SRC)lum6Dquat.h $(SRC)globals.icc
	echo Compiling ELCH 6D Quaternion ...
	$(GPP) $(CFLAGS) -I$(SRC)ann_1.1.1_modified/include/ -c -o $(OBJ)elch6Dquat.o $(SRC)elch6Dquat.cc 

$(OBJ)elch6DunitQuat.o: $(SRC)elch6DunitQuat.cc $(SRC)elch6DunitQuat.h $(SRC)elch6D.h $(SRC)loopSlam6D.h $(SRC)icp6D.h $(SRC)icp6Dminimizer.h $(SRC)scan.h $(SRC)graph.h $(SRC)lum6Dquat.h $(SRC)globals.icc
	echo Compiling ELCH 6D Unit Quaternion ...
	$(GPP) $(CFLAGS) -I$(SRC)ann_1.1.1_modified/include/ -c -o $(OBJ)elch6DunitQuat.o $(SRC)elch6DunitQuat.cc 

$(OBJ)elch6Dslerp.o: $(SRC)elch6Dslerp.cc $(SRC)elch6Dslerp.h $(SRC)elch6D.h $(SRC)loopSlam6D.h $(SRC)icp6D.h $(SRC)icp6Dminimizer.h $(SRC)scan.h $(SRC)graph.h $(SRC)lum6Dquat.h $(SRC)globals.icc
	echo Compiling ELCH 6D SLERP Quaternion ...
	$(GPP) $(CFLAGS) -I$(SRC)ann_1.1.1_modified/include/ -c -o $(OBJ)elch6Dslerp.o $(SRC)elch6Dslerp.cc 

$(OBJ)loopToro.o: $(SRC)loopToro.cc $(SRC)loopToro.h $(SRC)loopSlam6D.h $(SRC)icp6D.h $(SRC)icp6Dminimizer.h $(SRC)scan.h $(SRC)graph.h $(SRC)lum6Dquat.h $(SRC)globals.icc
	echo Compiling Loop TORO ...
	$(GPP) $(CFLAGS)  -I$(SRC)ann_1.1.1_modified/include/ -c -o $(OBJ)loopToro.o $(SRC)loopToro.cc

$(OBJ)loopHOG-Man.o: $(SRC)loopHOG-Man.cc $(SRC)loopHOG-Man.h $(SRC)loopSlam6D.h $(SRC)icp6D.h $(SRC)icp6Dminimizer.h $(SRC)scan.h $(SRC)graph.h $(SRC)lum6Dquat.h $(SRC)globals.icc
	echo Compiling Loop HOG-Man ...
	$(GPP) $(CFLAGS)  -I$(SRC)ann_1.1.1_modified/include/ -c -o $(OBJ)loopHOG-Man.o $(SRC)loopHOG-Man.cc

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
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -c -o $(OBJ)icp6Dquat.o $(SRC)icp6Dquat.cc 

$(OBJ)icp6Dhelix.o: $(SRC)icp6Dhelix.h $(SRC)icp6Dhelix.cc $(SRC)ptpair.h $(SRC)icp6Dminimizer.h
	echo Compiling ICP 6D with Helix ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)icp6Dhelix.o $(SRC)icp6Dhelix.cc

$(OBJ)icp6Ddual.o: $(SRC)icp6Ddual.h $(SRC)icp6Ddual.cc $(SRC)ptpair.h $(SRC)icp6Dminimizer.h
	echo Compiling ICP 6D with Dual Quaternions ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)icp6Ddual.o $(SRC)icp6Ddual.cc

$(OBJ)icp6Dlumeuler.o: $(SRC)icp6Dlumeuler.h $(SRC)icp6Dlumeuler.cc $(SRC)ptpair.h $(SRC)icp6Dminimizer.h
	echo Compiling ICP 6D with uncertainty-based using Euler angles ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)icp6Dlumeuler.o $(SRC)icp6Dlumeuler.cc

$(OBJ)icp6Dlumquat.o: $(SRC)icp6Dlumquat.h $(SRC)icp6Dlumquat.cc $(SRC)ptpair.h $(SRC)icp6Dminimizer.h
	echo Compiling ICP 6D with uncertainty-based using quaternions ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)icp6Dlumquat.o $(SRC)icp6Dlumquat.cc

$(OBJ)graph.o: $(SRC)graph.h $(SRC)graph.cc $(SRC)globals.icc $(SRC)scan.h
	echo Compiling Graph ...
	$(GPP) $(CFLAGS)  -I$(SRC)ann_1.1.1_modified/include/ -c -o $(OBJ)graph.o $(SRC)graph.cc

$(OBJ)csparse.o: $(SRC)sparse/csparse.h $(SRC)sparse/csparse.cc
	echo Compiling Csparse ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)csparse.o $(SRC)sparse/csparse.cc

$(OBJ)CIcpGpuCuda.o: $(SRC)cuda/CIcpGpuCuda.cu $(SRC)cuda/CIcpGpuCuda.cuh $(SRC)cuda/CIcpGpuCuda_kernel.cuh 
	echo Compiling CUDA ICP ... 
	$(CUDA) -c $(SRC)cuda/CIcpGpuCuda.cu -o $(OBJ)CIcpGpuCuda.o -I$(SRC) -I$(SRC)ann_1.1.1_modified/include/ -I$(SRC)ann_1.1.1_modified/src/ $(CUDAINCDIRS)

$(OBJ)icp6Dcuda.o: $(SRC)cuda/icp6Dcuda.cc $(SRC)cuda/icp6Dcuda.h $(SRC)scan.h
	echo Compiling CUDA wrapper ... 
	$(GPP) ${CFLAGSCUDA} ${LIBDIRS} $(CFLAGS) -c $(SRC)cuda/icp6Dcuda.cc -I$(SRC) -I$(SRC)ann_1.1.1_modified/include/ -I$(SRC)ann_1.1.1_modified/src/ $(CUDAINCDIRS) -o $(OBJ)icp6Dcuda.o 


docu_html:
	doxygen doc/doxygen.cfg
	cd $(DOC) ; zip -q html.zip html/*
	echo
	echo

docu_latex:
	$(MAKE) -C $(DOC)latex
	cd $(DOC)latex ; dvips refman
	cd $(DOC)latex ; ps2pdf14 refman.ps refman.pdf
	cp $(DOC)latex/refman.pdf $(DOC)

docu_hl:	$(DOC)high_level_doc/documentation.tex
	cd $(DOC)high_level_doc ; latex documentation.tex
	cd $(DOC)high_level_doc ; bibtex documentation
	cd $(DOC)high_level_doc ; latex documentation.tex
	cd $(DOC)high_level_doc ; dvips documentation
	cd $(DOC)high_level_doc ; ps2pdf14 documentation.ps ../documentation_HL.pdf


############# SLAM6D LIBS ##############

$(OBJ)libANN.a: $(SRC)ann_1.1.1_modified/src/*.cpp $(SRC)ann_1.1.1_modified/src/*.h
	echo Making modified ANN lib ...
	$(MAKE) -C $(SRC)ann_1.1.1_modified/src

$(OBJ)libnewmat.a: $(SRC)newmat/*.cpp $(SRC)newmat/*.h
	echo Compiling Newmat ...
	$(MAKE) -C $(SRC)newmat

$(BIN)scan_io_uos.so: $(SRC)scan_io.h $(SRC)scan_io_uos.h $(SRC)scan_io_uos.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading UOS scans ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_uos.so $(SRC)scan_io_uos.cc

ifdef WITH_RIVLIB
$(BIN)scan_io_rxp.so: $(SRC)scan_io.h $(SRC)scan_io_rxp.h $(SRC)scan_io_rxp.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading RIEGL rxp scans ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_rxp.so -I$(SRC) $(SRC)scan_io_rxp.cc $(SRC)riegl/libscanlib-mt-s.a $(SRC)riegl/libctrllib-mt-s.a $(SRC)riegl/libboost_system-mt-s-1_35-vns.a -lpthread
else
$(BIN)scan_io_rxp.so: 
endif

$(BIN)scan_io_uos_map.so: $(SRC)scan_io.h $(SRC)scan_io_uos_map.h $(SRC)scan_io_uos_map.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading UOS scans with given map ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_uos_map.so $(SRC)scan_io_uos_map.cc 

$(BIN)scan_io_uos_frames.so: $(SRC)scan_io.h $(SRC)scan_io_uos_frames.h $(SRC)scan_io_uos_frames.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading UOS scans with frames as poses...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_uos_frames.so $(SRC)scan_io_uos_frames.cc

$(BIN)scan_io_uos_map_frames.so: $(SRC)scan_io.h $(SRC)scan_io_uos_map_frames.h $(SRC)scan_io_uos_map_frames.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading UOS scans with given map and frames as poses...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_uos_map_frames.so $(SRC)scan_io_uos_map_frames.cc

$(BIN)scan_io_old.so: $(SRC)scan_io.h $(SRC)scan_io_old.h $(SRC)scan_io_old.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading old UOS scans ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_old.so $(SRC)scan_io_old.cc 

$(BIN)scan_io_x3d.so: $(SRC)scan_io.h $(SRC)scan_io_x3d.h $(SRC)scan_io_x3d.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading x3d scans ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_x3d.so $(SRC)scan_io_x3d.cc 

$(BIN)scan_io_rts.so: $(SRC)scan_io.h $(SRC)scan_io_rts.h $(SRC)scan_io_rts.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading RTS scans ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_rts.so $(SRC)scan_io_rts.cc 

$(BIN)scan_io_iais.so: $(SRC)scan_io.h $(SRC)scan_io_iais.h $(SRC)scan_io_iais.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading IAIS scans ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_iais.so $(SRC)scan_io_iais.cc 

$(BIN)scan_io_rts_map.so: $(SRC)scan_io.h $(SRC)scan_io_rts_map.h $(SRC)scan_io_rts_map.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading RTS scans with given map ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_rts_map.so $(SRC)scan_io_rts_map.cc 

$(BIN)scan_io_riegl_bin.so: $(SRC)scan_io.h $(SRC)scan_io_riegl_bin.h $(SRC)scan_io_riegl_bin.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading Riegl scans in binary mode ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_riegl_bin.so $(SRC)scan_io_riegl_bin.cc 

$(BIN)scan_io_riegl_txt.so: $(SRC)scan_io.h $(SRC)scan_io_riegl_txt.h $(SRC)scan_io_riegl_txt.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading Riegl scans in text mode ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_riegl_txt.so $(SRC)scan_io_riegl_txt.cc 

$(BIN)scan_io_zuf.so: $(SRC)scan_io.h $(SRC)scan_io_zuf.h $(SRC)scan_io_zuf.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading Z+F scans ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_zuf.so $(SRC)scan_io_zuf.cc 

$(BIN)scan_io_asc.so: $(SRC)scan_io.h $(SRC)scan_io_asc.h $(SRC)scan_io_asc.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading ASC scans ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_asc.so $(SRC)scan_io_asc.cc 

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

$(BIN)scan_io_front.so: $(SRC)scan_io.h $(SRC)scan_io_front.h $(SRC)scan_io_front.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading 2D Front scans ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_front.so $(SRC)scan_io_front.cc 

$(BIN)scan_io_zahn.so: $(SRC)scan_io.h $(SRC)scan_io_zahn.h $(SRC)scan_io_zahn.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
	echo Compiling shared library for reading Zaehne scans ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_zahn.so $(SRC)scan_io_zahn.cc 
	echo DONE
	echo


############# SCAN REDUCTION ##############

$(BIN)scan_red: $(OBJ)scanlib.a $(SRC)globals.icc $(SRC)scan_red.cc $(OBJ)libANN.a
	echo Compiling and Linking Scan Reduction ...
	$(GPP) $(CFLAGS)  -I$(SRC)ann_1.1.1_modified/include/ -o $(BIN)scan_red $(SRC)scan_red.cc $(OBJ)scanlib.a $(OBJ)libANN.a -ldl $(LIBRARIES) 
	echo DONE
	echo

############# SCAN DIFFERENCE ##############

$(BIN)scan_diff: $(OBJ)scanlib.a $(SRC)globals.icc $(SRC)scan_diff.cc $(OBJ)libANN.a
	echo Compiling and Linking Scan Difference ...
	$(GPP) $(CFLAGS) -I$(SRC)ann_1.1.1_modified/include/ -o $(BIN)scan_diff $(SRC)scan_diff.cc $(OBJ)scanlib.a $(OBJ)libANN.a -ldl $(LIBRARIES) 
	echo DONE
	echo

############# SHOW ##############

$(BIN)show: $(OBJ)libglui.a $(SHOWSRC)show.cc $(SHOWSRC)show.h $(SHOWSRC)show.icc $(SHOWSRC)show1.icc $(SHOWSRC)show_menu.cc $(SHOWSRC)show_gl.cc $(SHOWSRC)show_animate.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc $(OBJ)scan.o $(OBJ)vertexarray.o $(OBJ)PathGraph.o $(OBJ)NurbsPath.o $(OBJ)viewcull.o $(OBJ)scanlib.a $(OBJ)colormanager.o  $(OBJ)libANN.a
	echo Compiling and Linking Show ...
	$(GPP) $(CFLAGS)  -I$(SRC)ann_1.1.1_modified/include/ -o $(BIN)show -I$(SRC) $(SHOWSRC)show.cc $(OBJ)scanlib.a $(OBJ)vertexarray.o $(OBJ)PathGraph.o $(OBJ)NurbsPath.o $(OBJ)viewcull.o $(OBJ)colormanager.o $(OBJ)libglui.a $(OBJ)libANN.a $(LIBRARIES)
	echo DONE
	echo

$(OBJ)colormanager.o: $(SHOWSRC)colormanager.h $(SHOWSRC)colormanager.cc
	echo Compiling ColorManager for Show ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)colormanager.o -I$(SRC) $(SHOWSRC)colormanager.cc

$(OBJ)viewcull.o: $(SHOWSRC)viewcull.h $(SHOWSRC)viewcull.cc
	echo Compiling Software View Culling ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)viewcull.o $(SHOWSRC)viewcull.cc 

$(OBJ)vertexarray.o: $(SHOWSRC)vertexarray.h $(SHOWSRC)vertexarray.cc
	echo Compiling VertexArray ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)vertexarray.o $(SHOWSRC)vertexarray.cc

$(OBJ)NurbsPath.o: $(SHOWSRC)NurbsPath.h $(SHOWSRC)NurbsPath.cc $(SRC)globals.icc $(SHOWSRC)PathGraph.h
	echo Compiling NurbsPath ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)NurbsPath.o $(SHOWSRC)NurbsPath.cc

$(OBJ)PathGraph.o: $(SHOWSRC)PathGraph.h $(SHOWSRC)PathGraph.cc $(SRC)globals.icc
	echo Compiling PathGraph ...
	$(GPP) $(CFLAGS) -c -o $(OBJ)PathGraph.o $(SHOWSRC)PathGraph.cc

$(OBJ)libglui.a: $(SHOWSRC)glui/*.c $(SHOWSRC)glui/*.cpp $(SHOWSRC)glui/*.h
	echo Compiling Glui ...
	$(MAKE) -C $(SHOWSRC)glui


############# GRIDDER ##############

$(OBJ)gridder.o: $(GRIDSRC)2DGridder.cc $(GRIDSRC)scanmanager.h $(GRIDSRC)scanGrid.h $(GRIDSRC)scanToGrid.h $(GRIDSRC)parcelmanager.h $(GRIDSRC)gridlines.h
	echo Compiling 2Dgridder ...
	$(GPP) $(CFLAGS) -I$(SRC)ann_1.1.1_modified/include/ -c $(GRIDSRC)2DGridder.cc -o $(OBJ)gridder.o

$(OBJ)grid.o: $(GRIDSRC)grid.cc $(GRIDSRC)grid.h $(GRIDSRC)gridPoint.h
	echo Compiling Grid ...
	$(GPP) $(CFLAGS) -c $(GRIDSRC)grid.cc -o $(OBJ)grid.o

$(OBJ)scanGrid.o: $(GRIDSRC)scanGrid.cc $(GRIDSRC)scanGrid.h $(GRIDSRC)grid.h
	echo Compiling ScanGrid ...
	$(GPP) $(CFLAGS) -c $(GRIDSRC)scanGrid.cc -o $(OBJ)scanGrid.o

$(OBJ)scanToGrid.o: $(GRIDSRC)scanToGrid.cc $(GRIDSRC)scanToGrid.h
	echo Compiling ScanToGrid ...
	$(GPP) -I$(SRC)ann_1.1.1_modified/include/ $(CFLAGS) -c $(GRIDSRC)scanToGrid.cc -o $(OBJ)scanToGrid.o

$(OBJ)gridPoint.o: $(GRIDSRC)gridPoint.cc $(GRIDSRC)gridPoint.h
	echo Compiling GridPoint ...
	$(GPP) $(CFLAGS) -c $(GRIDSRC)gridPoint.cc -o $(OBJ)gridPoint.o

$(OBJ)scanmanager.o: $(GRIDSRC)scanmanager.cc $(GRIDSRC)scanmanager.h
	echo Compiling Scanmanager ...
	$(GPP) $(CFLAGS) -I$(SRC)ann_1.1.1_modified/include/ -c $(GRIDSRC)scanmanager.cc -o $(OBJ)scanmanager.o

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

$(OBJ)viewpointinfo.o: $(GRIDSRC)viewpointinfo.h $(GRIDSRC)viewpointinfo.cc $(GRIDSRC)scanGrid.h
	echo Compiling Viewpointinfo ...
	$(GPP) $(CFLAGS) -c $(GRIDSRC)viewpointinfo.cc -o $(OBJ)viewpointinfo.o

$(OBJ)line.o: $(GRIDSRC)line.cc $(GRIDSRC)line.h $(GRIDSRC)gridPoint.h
	echo Compiling Line ...
	$(GPP) $(CFLAGS) -c $(GRIDSRC)line.cc -o $(OBJ)line.o

$(OBJ)gridlines.o: $(GRIDSRC)gridlines.cc $(GRIDSRC)gridlines.h $(GRIDSRC)hough.h $(GRIDSRC)line.h $(GRIDSRC)grid.h
	echo Compiling Gridlines ...
	$(GPP) $(CFLAGS) -c $(GRIDSRC)gridlines.cc -o $(OBJ)gridlines.o

$(OBJ)hough.o: $(GRIDSRC)hough.cc $(GRIDSRC)hough.h
	echo Compiling Hough ...
	$(GPP) $(CFLAGS) -c $(GRIDSRC)hough.cc -o $(OBJ)hough.o

$(BIN)2DGridder: $(OBJ)gridder.o $(OBJ)line.o $(OBJ)gridlines.o $(OBJ)hough.o $(OBJ)viewpointinfo.o $(OBJ)gridWriter.o $(OBJ)parcelmanager.o $(OBJ)parcel.o $(OBJ)parcelinfo.o $(OBJ)scanGrid.o $(OBJ)grid.o $(OBJ)scanToGrid.o $(OBJ)gridPoint.o $(OBJ)scanlib.a $(OBJ)scanmanager.o $(OBJ)libANN.a
	echo Compiling and Linking Grid ...
	$(GPP) $(CFLAGS) -I$(SRC)ann_1.1.1_modified/include/ -o $(BIN)2DGridder $(OBJ)viewpointinfo.o $(OBJ)line.o $(OBJ)gridlines.o $(OBJ)hough.o $(OBJ)gridder.o $(OBJ)gridWriter.o $(OBJ)parcelmanager.o $(OBJ)parcelinfo.o $(OBJ)scanmanager.o $(OBJ)grid.o $(OBJ)scanGrid.o $(OBJ)parcel.o $(OBJ)gridPoint.o $(OBJ)scanToGrid.o $(OBJ)scanlib.a $(OBJ)libANN.a -ldl  -lstdc++ $(LIBRARIES)
	echo DONE
	echo


############# TOOLS ##############

$(BIN)frame_to_graph: $(SRC)frame_to_graph.cc $(SRC)globals.icc
	echo Compiling and linking Frame_to_graph ...
	$(GPP) $(CFLAGS) -o $(BIN)frame_to_graph $(SRC)frame_to_graph.cc 
	echo DONE
	echo

$(BIN)convergence: $(SRC)convergence.cc $(SRC)convergence.h $(SRC)globals.icc
	echo Compiling and linking Convergence ...
	$(GPP) $(CFLAGS) -I$(SRC)ann_1.1.1_modified/include/ -o $(BIN)convergence $(SRC)convergence.cc
	echo DONE
	echo

$(BIN)graph_balancer: $(OBJ)elch6D.o $(SRC)graph_balancer.cc $(SRC)graph.h
	echo Compiling and linking Graph Balancer ...
	$(GPP) $(CFLAGS) -I$(SRC)ann_1.1.1_modified/include/ -lboost_graph-mt -o $(BIN)graph_balancer $(SRC)graph_balancer.cc $(OBJ)elch6D.o 
	echo DONE
	echo

$(BIN)exportPoints: $(SRC)exportPoints.cc $(OBJ)scanlib.a $(SRC)globals.icc $(OBJ)libANN.a

	echo Compiling and linking exportPoints ...
	$(GPP) $(CFLAGS) -I$(SRC)ann_1.1.1_modified/include/ -o $(BIN)exportPoints $(SRC)exportPoints.cc $(OBJ)scanlib.a $(OBJ)libANN.a -ldl 
	echo DONE
	echo


############# PMD camera ##############

$(OBJ)libpmdaccess2.a: $(PMDSRC)/pmdaccess2/pmdaccess.cc
	echo Compiling libpmdaccess ...
	$(GPP) -c $(PMDSRC)/pmdaccess2/pmdaccess.cc -o $(OBJ)pmdaccess.o                                   
	ar rs $(OBJ)libpmdaccess2.a $(OBJ)pmdaccess.o  

$(OBJ)cvpmd.o: $(PMDSRC)cvpmd.cc
	echo Compiling OpenCV PMD ...
	$(GPP) $(CFLAGS) $(PMDPKG) -I$(PMDSRC)pmdaccess2 -I$(SRC) -c -o $(OBJ)cvpmd.o $(PMDSRC)cvpmd.cc

$(OBJ)pmdWrap.o: $(PMDSRC)pmdWrap.cc
	echo Compiling PMD wrapper ...
	$(GPP) $(CFLAGS) $(PMDPKG) -I$(PMDSRC)pmdaccess2 -I$(SRC) -c -o $(OBJ)pmdWrap.o $(PMDSRC)pmdWrap.cc

$(BIN)grabVideoAnd3D: $(OBJ)pmdWrap.o $(OBJ)cvpmd.o $(OBJ)icp6D.o $(OBJ)icp6Dapx.o $(OBJ)icp6Dhelix.o $(OBJ)icp6Dortho.o $(OBJ)icp6Dquat.o $(OBJ)icp6Dsvd.o $(OBJ)scanlib.a $(OBJ)libnewmat.a $(OBJ)libpmdaccess2.a $(PMDSRC)offline/grabVideoAnd3D.cc $(OBJ)libANN.a
	echo Compiling and Linking video and pmd grabber ...
	$(GPP) $(CFLAGS) $(PMDPKG) -I$(PMDSRC)  -I$(SRC)ann_1.1.1_modified/include/ -I$(PMDSRC)pmdaccess2 -I$(SRC) $(PMDLIBS) $(OBJ)pmdWrap.o $(OBJ)cvpmd.o $(OBJ)icp6D.o $(OBJ)icp6Dapx.o $(OBJ)icp6Dhelix.o $(OBJ)icp6Dortho.o $(OBJ)icp6Dquat.o $(OBJ)icp6Dsvd.o $(OBJ)scanlib.a $(OBJ)libnewmat.a $(OBJ)libpmdaccess2.a $(OBJ)libANN.a -o $(BIN)grabVideoAnd3D $(PMDSRC)offline/grabVideoAnd3D.cc

$(BIN)convertToSLAM6D: $(OBJ)pmdWrap.o $(OBJ)cvpmd.o $(OBJ)icp6D.o $(OBJ)icp6Dapx.o $(OBJ)icp6Dhelix.o $(OBJ)icp6Dortho.o $(OBJ)icp6Dquat.o $(OBJ)icp6Dsvd.o $(OBJ)scanlib.a $(OBJ)libnewmat.a $(OBJ)libpmdaccess2.a $(OBJ)libANN.a $(PMDSRC)offline/convertToSLAM6D.cc
	echo Compiling and Linking converting tool to slam6D ...
	echo Linking converting tool to slam6D ...
	$(GPP) $(CFLAGS) $(PMDPKG) -I$(PMDSRC) -I$(SRC)ann_1.1.1_modified/include/ -I$(PMDSRC)pmdaccess2 -I$(SRC) $(PMDLIBS) $(OBJ)pmdWrap.o $(OBJ)cvpmd.o $(OBJ)icp6D.o $(OBJ)icp6Dapx.o $(OBJ)icp6Dhelix.o $(OBJ)icp6Dortho.o $(OBJ)icp6Dquat.o $(OBJ)icp6Dsvd.o $(OBJ)scanlib.a $(OBJ)libnewmat.a $(OBJ)libpmdaccess2.a $(OBJ)libANN.a -o $(BIN)convertToSLAM6D $(PMDSRC)offline/convertToSLAM6D.cc

$(BIN)calibrate: $(PMDSRC)/calibrate/calibrate.cc
	echo Compiling and Linking calibrate ...
	$(GPP) $(CFLAGS) $(PMDPKG) -I$(PMDSRC) -I$(PMDSRC)pmdaccess2 -I$(SRC) $(PMDLIBS) -o $(BIN)calibrate $(PMDSRC)/calibrate/calibrate.cc

$(BIN)grabFramesCam: $(PMDSRC)calibrate/grabFramesCam.cc
	echo Compiling and Linking grab frames camera ...
	$(GPP) $(CFLAGS) $(PMDPKG) -I$(PMDSRC) -I$(PMDSRC)pmdaccess2 -I$(SRC) $(PMDLIBS) -o $(BIN)grabFramesCam $(PMDSRC)calibrate/grabFramesCam.cc

$(BIN)grabFramesPMD: $(PMDSRC)calibrate/grabFramesPMD.cc $(OBJ)libpmdaccess2.a $(OBJ)scanlib.a $(OBJ)libANN.a
	echo Compiling and Linking grab frames PMD ...
	$(GPP) $(CFLAGS) $(PMDPKG) -I$(PMDSRC)  -I$(SRC)ann_1.1.1_modified/include/ -I$(PMDSRC)pmdaccess2 -I$(SRC) $(OBJ)cvpmd.o $(OBJ)pmdWrap.o $(OBJ)libpmdaccess2.a $(OBJ)icp6D.o $(OBJ)icp6Dapx.o $(OBJ)icp6Dhelix.o $(OBJ)icp6Dortho.o $(OBJ)icp6Dquat.o $(OBJ)icp6Dsvd.o $(OBJ)scanlib.a $(PMDLIBS) $(OBJ)libnewmat.a $(OBJ)libANN.a -o $(BIN)grabFramesPMD $(PMDSRC)calibrate/grabFramesPMD.cc

$(BIN)extrinsic: $(PMDSRC)calibrate/extrinsic.cc
	echo Compiling and Linking extrinsic camera calibration ...
	$(GPP) $(CFLAGS) $(PMDPKG) -I$(PMDSRC) -I$(PMDSRC)pmdaccess2 -I$(SRC) $(PMDLIBS) -o $(BIN)extrinsic $(PMDSRC)calibrate/extrinsic.cc

$(BIN)pose: $(PMDSRC)pose/pose.cc $(PMDSRC)pose/history.cc $(OBJ)libpmdaccess2.a $(OBJ)scanlib.a $(OBJ)libnewmat.a $(OBJ)libANN.a
	echo Compiling and Linking PMD pose ...
	$(GPP) $(CFLAGS) $(PMDPKG) -I$(PMDSRC)  -I$(SRC)ann_1.1.1_modified/include/ -I$(PMDSRC)pmdaccess2 -I$(SRC) $(OBJ)cvpmd.o $(OBJ)pmdWrap.o $(OBJ)libpmdaccess2.a $(OBJ)icp6D.o $(OBJ)icp6Dapx.o $(OBJ)icp6Dhelix.o $(OBJ)icp6Dortho.o $(OBJ)icp6Dquat.o $(OBJ)icp6Dsvd.o $(OBJ)scanlib.a $(PMDLIBS) $(OBJ)libnewmat.a $(OBJ)libANN.a -o $(BIN)pose $(PMDSRC)pose/pose.cc $(PMDSRC)pose/history.cc
	echo DONE
	echo

############# SIFT based registration ##############

$(OBJ)LoweDetector.o: $(APSSRC)LoweDetector.*
	echo Compiling LoweDetector ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -I$(APSSRC) -o $(OBJ)LoweDetector.o -c $(APSSRC)LoweDetector.c

$(OBJ)RANSAC.o: $(APSSRC)RANSAC.*
	echo Compiling RANSAC ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -I$(APSSRC) -o $(OBJ)RANSAC.o -c $(APSSRC)RANSAC.c

$(OBJ)GaussianConvolution.o: $(APSSRC)GaussianConvolution.*
	echo Compiling GaussianConvolution ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -I$(APSSRC) -o $(OBJ)GaussianConvolution.o -c $(APSSRC)GaussianConvolution.c

$(OBJ)ScaleSpace.o: $(APSSRC)ScaleSpace.*
	echo Compiling ScaleSpace ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -I$(APSSRC) -o $(OBJ)ScaleSpace.o -c $(APSSRC)ScaleSpace.c

$(OBJ)KeypointXML.o: $(APSSRC)KeypointXML.*
	echo Compiling KeypointXML ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -I$(APSSRC) -o $(OBJ)KeypointXML.o -c $(APSSRC)KeypointXML.c `pkg-config --cflags libxml-2.0`

$(OBJ)MatchKeys.o: $(APSSRC)MatchKeys.*
	echo Compiling MatchKeys ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -I$(APSSRC) -o $(OBJ)MatchKeys.o -c $(APSSRC)MatchKeys.c

$(OBJ)KDTree.o: $(APSSRC)KDTree.*
	echo Compiling KDTree for SIFT ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -I$(APSSRC) -o $(OBJ)KDTree.o -c $(APSSRC)KDTree.c

$(OBJ)BondBall.o: $(APSSRC)BondBall.*
	echo Compiling BondBall ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -I$(APSSRC) -o $(OBJ)BondBall.o -c $(APSSRC)BondBall.c

$(OBJ)AreaFilter.o: $(APSSRC)AreaFilter.*
	echo Compiling AreaFilter ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -I$(APSSRC) -o $(OBJ)AreaFilter.o -c $(APSSRC)AreaFilter.c

$(OBJ)ImageMatchModel.o: $(APSSRC)ImageMatchModel.*
	echo Compiling ImageMatchModel ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -I$(APSSRC) -o $(OBJ)ImageMatchModel.o -c $(APSSRC)ImageMatchModel.c

$(OBJ)Transform.o: $(APSSRC)Transform.*
	echo Compiling Transform ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -I$(APSSRC) -o $(OBJ)Transform.o -c $(APSSRC)Transform.c

$(OBJ)DisplayImage.o: $(APSSRC)DisplayImage.*
	echo Compiling DisplayImager ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -I$(APSSRC) -o $(OBJ)DisplayImage.o -c $(APSSRC)DisplayImage.c

$(OBJ)ImageMap.o: $(APSSRC)ImageMap.*
	echo Compiling ImageMap ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -I$(APSSRC) -o $(OBJ)ImageMap.o -c $(APSSRC)ImageMap.c

$(OBJ)HashTable.o: $(APSSRC)HashTable.*
	echo Compiling HashTable ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -I$(APSSRC) -o $(OBJ)HashTable.o -c $(APSSRC)HashTable.c

$(OBJ)ArrayList.o: $(APSSRC)ArrayList.*
	echo Compiling ArrayList ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -I$(APSSRC) -o $(OBJ)ArrayList.o -c $(APSSRC)ArrayList.c

$(OBJ)SAreaFilter.o: $(APSSRC)SAreaFilter.*
	echo Compiling HashTable ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -I$(APSSRC) -o $(OBJ)SAreaFilter.o -c $(APSSRC)ASAreaFilter.c

$(OBJ)Random.o: $(APSSRC)Random.*
	echo Compiling Random ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -I$(APSSRC) -o $(OBJ)Random.o -c $(APSSRC)Random.c

$(OBJ)SimpleMatrix.o: $(APSSRC)SimpleMatrix.*
	echo Compiling SimpleMatrix ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -I$(APSSRC) -o $(OBJ)SimpleMatrix.o -c $(APSSRC)SimpleMatrix.c

$(OBJ)Utils.o: $(APSSRC)Utils.*
	echo Compiling Utils ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -I$(APSSRC) -o $(OBJ)Utils.o -c $(APSSRC)Utils.c

$(OBJ)liblibsift.a: $(OBJ)LoweDetector.o $(OBJ)RANSAC.o $(OBJ)GaussianConvolution.o $(OBJ)ScaleSpace.o $(OBJ)KeypointXML.o $(OBJ)MatchKeys.o $(OBJ)KDTree.o $(OBJ)BondBall.o $(OBJ)AreaFilter.o $(OBJ)ImageMatchModel.o $(OBJ)Transform.o $(OBJ)DisplayImage.o $(OBJ)ImageMap.o $(OBJ)HashTable.o $(OBJ)ArrayList.o $(OBJ)Random.o $(OBJ)SimpleMatrix.o $(OBJ)Utils.o
	echo Linking LibLibSift ...
	ar cr $(OBJ)liblibsift.a $(OBJ)LoweDetector.o $(OBJ)RANSAC.o $(OBJ)GaussianConvolution.o $(OBJ)ScaleSpace.o $(OBJ)KeypointXML.o $(OBJ)MatchKeys.o $(OBJ)KDTree.o $(OBJ)BondBall.o $(OBJ)AreaFilter.o $(OBJ)ImageMatchModel.o $(OBJ)Transform.o $(OBJ)DisplayImage.o $(OBJ)ImageMap.o $(OBJ)HashTable.o $(OBJ)ArrayList.o $(OBJ)Random.o $(OBJ)SimpleMatrix.o $(OBJ)Utils.o
	ranlib $(OBJ)liblibsift.a

$(OBJ)AutoPano.o: $(APSSRC)AutoPano.c
	echo Compiling AutoPano ...
	$(GCC) $(CFLAGS) $(SIFTFLAGS) -O2 -I$(APSSRC) -o $(OBJ)AutoPano.o -c $(APSSRC)AutoPano.c

$(BIN)autopano: $(OBJ)AutoPano.o $(OBJ)liblibsift.a
	echo Linking AutoPano ...
	$(GCC) $(CFLAGS) -o $(BIN)autopano $(OBJ)AutoPano.o -rdynamic $(OBJ)liblibsift.a -ljpeg -ltiff -lpng -lz -lpano13 -lxml2

$(BIN)generatekeys: $(OBJ)GenerateKeys.o $(OBJ)liblibsift.a
	echo Linking GenerateKeys ...
	$(GCC) $(CFLAGS) -o $(BIN)generatekeys $(OBJ)GenerateKeys.o -rdynamic $(OBJ)liblibsift.a -ljpeg -ltiff -lpng -lz -lpano13 -lxml2

$(OBJ)ANNkd_wrap.o: $(APSSRC)APSCpp/ANNkd_wrap.cpp
	echo Compiling ANN kd wrap ...
	$(GPP) $(CFLAGS) $(SIFTFLAGS) -o $(OBJ)ANNkd_wrap.o -c -I$(SRC)ann_1.1.1_modified/include/ $(APSSRC)APSCpp/ANNkd_wrap.cpp

$(OBJ)APSCpp_main.o: $(APSSRC)APSCpp/APSCpp_main.c
	echo Compiling APSCpp_main ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -o $(OBJ)APSCpp_main.o -c -I$(APSSRC) $(APSSRC)APSCpp/APSCpp_main.c

$(OBJ)APSCpp.o: $(APSSRC)APSCpp/APSCpp.c
	echo Compiling APSCpp ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -o $(OBJ)APSCpp.o -c -I$(APSSRC) $(APSSRC)APSCpp/APSCpp.c

$(OBJ)CamLens.o: $(APSSRC)APSCpp/CamLens.c 
	echo Compiling CamLens ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -o $(OBJ)CamLens.o -c -I$(APSSRC) $(APSSRC)APSCpp/CamLens.c

$(OBJ)HermiteSpline.o: $(APSSRC)APSCpp/HermiteSpline.c 
	echo Compiling HermiteSpline ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -o $(OBJ)HermiteSpline.o -c -I$(APSSRC) $(APSSRC)APSCpp/HermiteSpline.c

$(OBJ)saInterp.o: $(APSSRC)APSCpp/saInterp.c 
	echo Compiling saInterp ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -o $(OBJ)saInterp.o -c -I$(APSSRC) $(APSSRC)APSCpp/saInterp.c

$(OBJ)saRemap.o: $(APSSRC)APSCpp/saRemap.c
	echo Compiling saRemap ...
	$(GCC) $(FLAGS) $(SIFTFLAGS) -o $(OBJ)saRemap.o -c -I$(APSSRC) $(APSSRC)APSCpp/saRemap.c

$(BIN)autopano-sift-c: $(OBJ)ANNkd_wrap.o $(OBJ)APSCpp_main.o $(OBJ)APSCpp.o $(OBJ)CamLens.o $(OBJ)HermiteSpline.o $(OBJ)saInterp.o $(OBJ)saRemap.o $(OBJ)libANN.a $(OBJ)liblibsift.a
	echo Linking autopano-sift-c
	$(GPP) $(CFLAGS) $(SIFTFLAGS) $(OBJ)ANNkd_wrap.o $(OBJ)APSCpp_main.o $(OBJ)APSCpp.o $(OBJ)CamLens.o $(OBJ)HermiteSpline.o $(OBJ)saInterp.o $(OBJ)saRemap.o -o $(BIN)autopano-sift-c -rdynamic $(OBJ)libANN.a $(OBJ)liblibsift.a -ljpeg -ltiff -lpng -lz -lz -lpano13 -lxml2 -lstdc++ 
	echo DONE
	echo 

$(OBJ)Coord.o: $(SIFTSRC)library/Coord.*
	echo Compiling Coord ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dterscanreg_EXPORTS -o $(OBJ)Coord.o -c $(SIFTSRC)library/Coord.cpp

$(OBJ)PanoramaMap.o: $(SIFTSRC)library/PanoramaMap.*
	echo Compiling PanoramaMap ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dterscanreg_EXPORTS -o $(OBJ)PanoramaMap.o -c $(SIFTSRC)library/PanoramaMap.cpp

$(OBJ)PointC.o: $(SIFTSRC)library/PointC.*
	echo Compiling PointC ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dterscanreg_EXPORTS -o $(OBJ)PointC.o -c $(SIFTSRC)library/PointC.cpp

$(OBJ)PointCloud.o: $(SIFTSRC)library/PointCloud.*
	echo Compiling PointCloud ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dterscanreg_EXPORTS -o $(OBJ)PointCloud.o -c $(SIFTSRC)library/PointCloud.cpp

$(OBJ)PolarPoint.o: $(SIFTSRC)library/PolarPoint.*
	echo Compiling PolarPoint ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dterscanreg_EXPORTS -o $(OBJ)PolarPoint.o -c $(SIFTSRC)library/PolarPoint.cpp

$(OBJ)PolarPointCloud.o: $(SIFTSRC)library/PolarPointCloud.*
	echo Compiling PolarPointCloud ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dterscanreg_EXPORTS -o $(OBJ)PolarPointCloud.o -c $(SIFTSRC)library/PolarPointCloud.cpp

$(OBJ)Reader.o: $(SIFTSRC)library/Reader.cpp
	echo Compiling Reader.cpp ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dterscanreg_EXPORTS -o $(OBJ)Reader.o -c $(SIFTSRC)library/Reader.cpp

$(OBJ)Reader_RIEGL.o: $(SIFTSRC)library/Reader_RIEGL.*
	echo Compiling Reader_RIEGL ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dterscanreg_EXPORTS -o $(OBJ)Reader_RIEGL.o -c $(SIFTSRC)library/Reader_RIEGL.cpp

$(OBJ)SuperPixel.o: $(SIFTSRC)library/SuperPixel.*
	echo Compiling SuperPixel ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dterscanreg_EXPORTS -o $(OBJ)SuperPixel.o -c $(SIFTSRC)library/SuperPixel.cpp

$(OBJ)Feature.o: $(SIFTSRC)library/Feature.*
	echo Compiling Feature ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dterscanreg_EXPORTS -o $(OBJ)Feature.o -c $(SIFTSRC)library/Feature.cpp

$(OBJ)FeatureBase.o: $(SIFTSRC)library/FeatureBase.*
	echo Compiling FeatureBase ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dterscanreg_EXPORTS -o $(OBJ)FeatureBase.o -c $(SIFTSRC)library/FeatureBase.cpp

$(OBJ)FeatureSet.o: $(SIFTSRC)library/FeatureSet.*
	echo Compiling FeatureSet ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dterscanreg_EXPORTS -o $(OBJ)FeatureSet.o -c $(SIFTSRC)library/FeatureSet.cpp

$(OBJ)FeatureMatch.o: $(SIFTSRC)library/FeatureMatch.*
	echo Compiling FeatureMatch ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dterscanreg_EXPORTS -o $(OBJ)FeatureMatch.o -c $(SIFTSRC)library/FeatureMatch.cpp

$(OBJ)FeatureMatchSet.o: $(SIFTSRC)library/FeatureMatchSet.*
	echo Compiling FeatureMatchSet ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dterscanreg_EXPORTS -o $(OBJ)FeatureMatchSet.o -c $(SIFTSRC)library/FeatureMatchSet.cpp

$(OBJ)FeatureMatchSetGroup.o: $(SIFTSRC)library/FeatureMatchSetGroup.*
	echo Compiling FeatureMatchSetGroup.cpp ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dterscanreg_EXPORTS -o $(OBJ)FeatureMatchSetGroup.o -c $(SIFTSRC)library/FeatureMatchSetGroup.cpp

$(OBJ)Register.o: $(SIFTSRC)library/Register.*
	echo Compiling Register ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -I/usr/include/FTGL -I/usr/include/freetype2 -Dterscanreg_EXPORTS -I$(SRC) -I$(SIFTSRC) -I$(SIFTSRC)opengl_framework/ -I$(SIFTSRC)library/ -o $(OBJ)Register.o -c $(SIFTSRC)library/Register.cpp

$(OBJ)ScanTransform.o: $(SIFTSRC)library/ScanTransform.*
	echo Compiling ScanTransform ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dterscanreg_EXPORTS -o $(OBJ)ScanTransform.o -I$(SIFTSRC)library/ -c $(SIFTSRC)library/ScanTransform.cpp

$(OBJ)PanoramaMap_gl.o: $(SIFTSRC)library/opengl_objects/PanoramaMap_gl.*
	echo Compiling PanoramaMap_gl ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dterscanreg_EXPORTS -o $(OBJ)PanoramaMap_gl.o -I$(SIFTSRC)library/ -I$(SIFTSRC) -c $(SIFTSRC)library/opengl_objects/PanoramaMap_gl.cpp

$(OBJ)FeatureMatchSet_gl.o: $(SIFTSRC)library/opengl_objects/FeatureMatchSet_gl.*
	echo Compiling FeatureMatchSet_gl ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dterscanreg_EXPORTS -o $(OBJ)FeatureMatchSet_gl.o -I$(SIFTSRC)library/ -I$(SIFTSRC) -c $(SIFTSRC)library/opengl_objects/FeatureMatchSet_gl.cpp

$(OBJ)PointCloud_gl.o: $(SIFTSRC)library/opengl_objects/PointCloud_gl.*
	echo Compiling PointCloud_gl ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dterscanreg_EXPORTS -o $(OBJ)PointCloud_gl.o -I$(SIFTSRC)library/ -I$(SIFTSRC) -c $(SIFTSRC)library/opengl_objects/PointCloud_gl.cpp

$(OBJ)PolarPointCloud_gl.o: $(SIFTSRC)library/opengl_objects/PolarPointCloud_gl.*
	echo Compiling PolarPointCloud_gl ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dterscanreg_EXPORTS -o $(OBJ)PolarPointCloud_gl.o -I$(SIFTSRC)library/ -I$(SIFTSRC) -c $(SIFTSRC)library/opengl_objects/PolarPointCloud_gl.cpp

$(BIN)libterscanreg.so: $(OBJ)Coord.o $(OBJ)PanoramaMap.o $(OBJ)PointC.o $(OBJ)PointCloud.o $(OBJ)PolarPoint.o $(OBJ)PolarPointCloud.o $(OBJ)Reader.o $(OBJ)Reader_RIEGL.o $(OBJ)SuperPixel.o $(OBJ)Feature.o $(OBJ)FeatureBase.o $(OBJ)FeatureSet.o $(OBJ)FeatureMatch.o $(OBJ)FeatureMatchSet.o $(OBJ)FeatureMatchSetGroup.o $(OBJ)Register.o $(OBJ)ScanTransform.o $(OBJ)PanoramaMap_gl.o $(OBJ)FeatureMatchSet_gl.o $(OBJ)PointCloud_gl.o $(OBJ)PolarPointCloud_gl.o $(OBJ)icp6Dquat.o $(BIN)libopengl-framework.so
	echo Linking libterscanreg shared lib ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)libterscanreg.so $(OBJ)Coord.o $(OBJ)PanoramaMap.o $(OBJ)PointC.o $(OBJ)PointCloud.o $(OBJ)PolarPoint.o $(OBJ)PolarPointCloud.o $(OBJ)Reader.o $(OBJ)Reader_RIEGL.o $(OBJ)SuperPixel.o $(OBJ)Feature.o $(OBJ)FeatureBase.o $(OBJ)FeatureSet.o $(OBJ)FeatureMatch.o $(OBJ)FeatureMatchSet.o $(OBJ)FeatureMatchSetGroup.o $(OBJ)Register.o $(OBJ)ScanTransform.o $(OBJ)PanoramaMap_gl.o $(OBJ)FeatureMatchSet_gl.o $(OBJ)PointCloud_gl.o $(OBJ)PolarPointCloud_gl.o $(OBJ)icp6Dquat.o $(BIN)libopengl-framework.so -lfreetype -lftgl -lglut

$(OBJ)CoordGL.o: $(SIFTSRC)opengl_framework/CoordGL.*
	echo Compiling CoordGL ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dopengl_framework_EXPORTS -o $(OBJ)CoordGL.o -c $(SIFTSRC)opengl_framework/CoordGL.cpp

$(OBJ)GL.o: $(SIFTSRC)opengl_framework/GL.*
	echo Compiling GL ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dopengl_framework_EXPORTS  -I/usr/include/FTGL -I/usr/include/freetype2 -o $(OBJ)GL.o -c $(SIFTSRC)opengl_framework/GL.cpp

$(OBJ)Object_gl.o: $(SIFTSRC)opengl_framework/Object_gl.*
	echo Compiling CoordGL ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dopengl_framework_EXPORTS -o $(OBJ)Object_gl.o -c $(SIFTSRC)opengl_framework/Object_gl.cpp

$(OBJ)X.o: $(SIFTSRC)opengl_framework/X.*
	echo Compiling X ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dopengl_framework_EXPORTS -o $(OBJ)X.o -c $(SIFTSRC)opengl_framework/X.cpp

$(BIN)libopengl-framework.so: $(OBJ)CoordGL.o $(OBJ)GL.o $(OBJ)Object_gl.o $(OBJ)X.o 
	echo Linking libopengl-framework shared lib ...
	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -Dopengl_framework_EXPORTS -o $(BIN)libopengl-framework.so $(OBJ)CoordGL.o $(OBJ)GL.o $(OBJ)Object_gl.o $(OBJ)X.o -lfreetype -lftgl -lglut


$(BIN)generatesiftfeatures: $(SIFTSRC)programs/generatesiftfeatures.cpp $(BIN)libterscanreg.so $(BIN)libopengl-framework.so
	echo Compiling and Linking Generatesiftfeatures ...
	$(GPP) $(CFLAGS) -o $(BIN)generatesiftfeatures -I$(SIFTSRC)opengl_framework/ -I$(SIFTSRC)library/ -DHAS_PANO13 -I$(APSSRC) $(SIFTSRC)programs/generatesiftfeatures.cpp $(OBJ)liblibsift.a $(BIN)libterscanreg.so $(BIN)libopengl-framework.so -lvigraimpex -lfreetype -lftgl -lglut -lxml2

$(BIN)mergehistograms: $(SIFTSRC)programs/mergehistograms.cpp $(BIN)libterscanreg.so $(BIN)libopengl-framework.so
	echo Compiling and Linking Mergehistograms ...
	$(GPP) $(CFLAGS) -o $(BIN)mergehistograms -I$(SIFTSRC)opengl_framework/ -I$(SIFTSRC)library/ -DHAS_PANO13 -I$(APSSRC) $(SIFTSRC)programs/mergehistograms.cpp $(OBJ)liblibsift.a $(BIN)libterscanreg.so $(BIN)libopengl-framework.so -lvigraimpex -lfreetype -lftgl -lglut -lxml2

$(BIN)panoramacreator: $(SIFTSRC)programs/panoramacreator.cpp $(BIN)libterscanreg.so $(BIN)libopengl-framework.so
	echo Compiling and Linking Panoramacreator ...
	$(GPP) $(CFLAGS) -o $(BIN)panoramacreator -I$(SIFTSRC)opengl_framework/ -I$(SIFTSRC)library/ -DHAS_PANO13 -I$(APSSRC) $(SIFTSRC)programs/panoramacreator.cpp $(OBJ)liblibsift.a $(BIN)libterscanreg.so $(BIN)libopengl-framework.so -lvigraimpex -lfreetype -lftgl -lglut -lxml2

$(BIN)visualizemap: $(SIFTSRC)programs/visualizemap.cpp $(BIN)libterscanreg.so $(BIN)libopengl-framework.so
	echo Compiling and Linking Visualizemap ...
	$(GPP) $(CFLAGS) -o $(BIN)visualizemap -I$(SIFTSRC)opengl_framework/ -I$(SIFTSRC)library/ -I/usr/include/FTGL -I/usr/include/freetype2 -DHAS_PANO13 -I$(APSSRC) -I$(SIFTSRC) $(SIFTSRC)programs/visualizemap.cpp $(OBJ)liblibsift.a $(BIN)libterscanreg.so $(BIN)libopengl-framework.so -lvigraimpex -lfreetype -lftgl -lglut -lxml2

$(BIN)visualizescan: $(SIFTSRC)programs/visualizescan.cpp $(BIN)libterscanreg.so $(BIN)libopengl-framework.so
	echo Compiling and Linking Visualizescan ...
	$(GPP) $(CFLAGS) -o $(BIN)visualizescan -I$(SIFTSRC)opengl_framework/ -I$(SIFTSRC)library/ -I/usr/include/FTGL -I/usr/include/freetype2 -DHAS_PANO13 -I$(APSSRC) -I$(SIFTSRC) $(SIFTSRC)programs/visualizescan.cpp $(OBJ)liblibsift.a $(BIN)libterscanreg.so $(BIN)libopengl-framework.so -lvigraimpex -lfreetype -lftgl -lglut -lxml2

$(BIN)visualizematches: $(SIFTSRC)programs/visualizematches.cpp $(BIN)libterscanreg.so $(BIN)libopengl-framework.so
	echo Compiling and Linking Visualizematches ...
	$(GPP) $(CFLAGS) -o $(BIN)visualizematches -I$(SIFTSRC)opengl_framework/ -I$(SIFTSRC)library/ -I/usr/include/FTGL -I/usr/include/freetype2 -DHAS_PANO13 -I$(APSSRC) -I$(SIFTSRC) $(SIFTSRC)programs/visualizematches.cpp $(OBJ)liblibsift.a $(BIN)libterscanreg.so $(BIN)libopengl-framework.so -lvigraimpex -lfreetype -lftgl -lglut -lxml2

$(BIN)visualizeregistrations: $(SIFTSRC)programs/visualizeregistrations.cpp $(BIN)libterscanreg.so $(BIN)libopengl-framework.so
	echo Compiling and Linking Visualizeregistrations ...
	$(GPP) $(CFLAGS) -o $(BIN)visualizeregistrations -I$(SIFTSRC)opengl_framework/ -I$(SIFTSRC)library/ -I/usr/include/FTGL -I/usr/include/freetype2 -DHAS_PANO13 -I$(APSSRC) -I$(SIFTSRC) -I$(SIFTSRC) $(SIFTSRC)programs/visualizeregistrations.cpp $(OBJ)liblibsift.a $(BIN)libterscanreg.so $(BIN)libopengl-framework.so -lvigraimpex -lfreetype -lftgl -lglut -lxml2

$(BIN)reduceppc: $(SIFTSRC)programs/reduceppc.cpp $(BIN)libterscanreg.so $(BIN)libopengl-framework.so
	echo Compiling and Linking Reduceppc ...
	$(GPP) $(CFLAGS) -o $(BIN)reduceppc -I$(SIFTSRC)opengl_framework/ -I$(SIFTSRC)library/ -DHAS_PANO13 -I$(APSSRC) $(SIFTSRC)programs/reduceppc.cpp $(OBJ)liblibsift.a $(BIN)libterscanreg.so $(BIN)libopengl-framework.so -lvigraimpex -lfreetype -lxml2

$(BIN)matchsiftfeatures: $(SIFTSRC)programs/matchsiftfeatures.cpp $(BIN)libterscanreg.so $(BIN)libopengl-framework.so
	echo Compiling and Linking Matchsiftfeatures ...
	$(GPP) $(CFLAGS) -o $(BIN)matchsiftfeatures -I$(SIFTSRC)opengl_framework/ -I$(SIFTSRC)library/ -DHAS_PANO13 -I$(APSSRC) $(SIFTSRC)programs/matchsiftfeatures.cpp $(OBJ)liblibsift.a $(BIN)libterscanreg.so $(BIN)libopengl-framework.so -lvigraimpex -lglut -lxml2

$(BIN)registerscans: $(SIFTSRC)programs/registerscans.cpp $(BIN)libterscanreg.so $(BIN)libopengl-framework.so
	echo Compiling and Linking Registerscans ...
	$(GPP) $(CFLAGS) -o $(BIN)registerscans -I$(SIFTSRC)opengl_framework/ -I$(SIFTSRC)library/ -I/usr/include/FTGL -I/usr/include/freetype2 -DHAS_PANO13 -I$(APSSRC) -I$(SIFTSRC) $(SIFTSRC)programs/registerscans.cpp $(OBJ)liblibsift.a $(BIN)libterscanreg.so $(BIN)libopengl-framework.so -lvigraimpex -lglut -lxml2

$(BIN)readscan: $(SIFTSRC)programs/readscan.cpp $(BIN)libterscanreg.so $(BIN)libopengl-framework.so
	echo Compiling and Linking readscan ...
	$(GPP) $(CFLAGS) -o $(BIN)readscan -I$(SIFTSRC)opengl_framework/ -I$(SIFTSRC)library/ -DHAS_PANO13 -I$(APSSRC) $(SIFTSRC)programs/readscan.cpp $(OBJ)liblibsift.a $(BIN)libterscanreg.so $(BIN)libopengl-framework.so -lvigraimpex -lglut -lxml2

############# TORO ##############
$(BIN)toro3d: $(SRC)toro/*
	$(MAKE) -C $(SRC)toro
	cp $(SRC)toro/toro3d $(BIN)

############# HOG-Man ##############
$(BIN)hogman3d: $(SRC)hogman/aislib/graph_optimizer_hogman/*.cpp
	cd src/hogman && ./configure
	LD_LIBRARY_PATH=`pwd`/src/hogman/lib $(MAKE) -C src/hogman
	cp src/hogman/bin/hogman3d $(BIN)
	cp src/hogman/lib/*.so $(BIN)

##################################################################################

svn_clean: # "are you sure?"-version
	@find . -name '.svn'         | xargs zip .svn.zip -r -q -m

svn_clean_del:
	@find . -name '.svn'         | xargs rm -r -f

clean:	
	/bin/rm -f $(OBJ)*
	/bin/rm -f $(BIN)*.so
	$(MAKE) -C $(SRC)newmat clean
	$(MAKE) -C $(SHOWSRC)glui clean
	$(MAKE) -C $(DOC)high_level_doc clean
#	$(MAKE) -C $(DOC)latex clean
