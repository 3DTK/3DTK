if (WIN32 OR CYGWIN)
# Compile ANN library
SET(ANN_DIR ${PROJECT_SOURCE_DIR}/3rdparty/ann_1.1.1_modified/src/)
SET(ANN_SRCS 
  ${ANN_DIR}ANN.cpp
  ${ANN_DIR}brute.cpp 
  ${ANN_DIR}kd_tree.cpp 
  ${ANN_DIR}kd_util.cpp 
  ${ANN_DIR}kd_split.cpp  
  ${ANN_DIR}kd_dump.cpp
  ${ANN_DIR}kd_search.cpp 
  ${ANN_DIR}kd_pr_search.cpp 
  ${ANN_DIR}kd_fix_rad_search.cpp 
  ${ANN_DIR}bd_tree.cpp 
  ${ANN_DIR}bd_search.cpp
  ${ANN_DIR}bd_pr_search.cpp 
  ${ANN_DIR}bd_fix_rad_search.cpp 
  ${ANN_DIR}perf.cpp)
add_library(ann STATIC ${ANN_SRCS})
IF(EXPORT_SHARED_LIBS)
add_library(ann_s SHARED ${ANN_SRCS})
ENDIF(EXPORT_SHARED_LIBS)

# on windows systems the DLL_EXPORTS is required
if (WIN32)
  SET_TARGET_PROPERTIES(ann PROPERTIES COMPILE_FLAGS -DDLL_EXPORTS)
endif(WIN32)
endif()

# Compile glui library
SET(GLUI_DIR ${PROJECT_SOURCE_DIR}/3rdparty/glui-2.36/)
SET(GLUI_SRCS
 ${GLUI_DIR}algebra3.cpp
 ${GLUI_DIR}arcball.cpp
 ${GLUI_DIR}glui_add_controls.cpp
 ${GLUI_DIR}glui_bitmap_img_data.cpp
 ${GLUI_DIR}glui_bitmaps.cpp
 ${GLUI_DIR}glui_button.cpp
 ${GLUI_DIR}glui_checkbox.cpp
 ${GLUI_DIR}glui_column.cpp
 ${GLUI_DIR}glui_commandline.cpp
 ${GLUI_DIR}glui_control.cpp
 ${GLUI_DIR}glui.cpp
 ${GLUI_DIR}glui_edittext.cpp
 ${GLUI_DIR}glui_filebrowser.cpp
 ${GLUI_DIR}glui_listbox.cpp
 ${GLUI_DIR}glui_list.cpp
 ${GLUI_DIR}glui_mouse_iaction.cpp
 ${GLUI_DIR}glui_node.cpp
 ${GLUI_DIR}glui_panel.cpp
 ${GLUI_DIR}glui_radio.cpp
 ${GLUI_DIR}glui_rollout.cpp
 ${GLUI_DIR}glui_rotation.cpp
 ${GLUI_DIR}glui_scrollbar.cpp
 ${GLUI_DIR}glui_separator.cpp
 ${GLUI_DIR}glui_spinner.cpp
 ${GLUI_DIR}glui_statictext.cpp
 ${GLUI_DIR}glui_string.cpp
 ${GLUI_DIR}glui_textbox.cpp
 ${GLUI_DIR}glui_translation.cpp
 ${GLUI_DIR}glui_tree.cpp
 ${GLUI_DIR}glui_treepanel.cpp
 ${GLUI_DIR}glui_window.cpp
 ${GLUI_DIR}quaternion.cpp
 ${GLUI_DIR}viewmodel.cpp
)
add_library(glui STATIC ${GLUI_SRCS})
if (WIN32)
	target_link_libraries(glui freeglut)
endif(WIN32)
IF(EXPORT_SHARED_LIBS)
	add_library(glui_s SHARED ${GLUI_SRCS})
	target_link_libraries(glui_s ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})
ENDIF(EXPORT_SHARED_LIBS)

IF(WITH_WXSHOW)

# Compile wxthings library
SET(WXTHINGS_DIR ${PROJECT_SOURCE_DIR}/3rdparty/wxthings/src/)
add_library(wxthings STATIC 
  ${WXTHINGS_DIR}block.cpp
  ${WXTHINGS_DIR}bmpcombo.cpp
  ${WXTHINGS_DIR}dropdown.cpp
  # ${WXTHINGS_DIR}filebrws.cpp  # these two are not needed and would create unnecessary dependencies
  #${WXTHINGS_DIR}genergdi.cpp
  ${WXTHINGS_DIR}geometry.cpp
  ${WXTHINGS_DIR}menubtn.cpp
  ${WXTHINGS_DIR}optvalue.cpp
  ${WXTHINGS_DIR}range.cpp
  ${WXTHINGS_DIR}spinctld.cpp
  ${WXTHINGS_DIR}toggle.cpp
)
ENDIF(WITH_WXSHOW)

if (WIN32)
# Compile sparse library 
SET(SPARSE_DIR ${PROJECT_SOURCE_DIR}/3rdparty/cxsparse-4.4.4/suitesparse/)
SET(SPARSE_SOURCES
	${SPARSE_DIR}cs_add.c
	${SPARSE_DIR}cs_amd.c
	${SPARSE_DIR}cs_chol.c
	${SPARSE_DIR}cs_cholsol.c
	${SPARSE_DIR}cs_compress.c
	${SPARSE_DIR}cs_convert.c
	${SPARSE_DIR}cs_counts.c
	${SPARSE_DIR}cs_cumsum.c
	${SPARSE_DIR}cs_dfs.c
	${SPARSE_DIR}cs_dmperm.c
	${SPARSE_DIR}cs_droptol.c
	${SPARSE_DIR}cs_dropzeros.c
	${SPARSE_DIR}cs_dupl.c
	${SPARSE_DIR}cs_entry.c
	${SPARSE_DIR}cs_ereach.c
	${SPARSE_DIR}cs_etree.c
	${SPARSE_DIR}cs_fkeep.c
	${SPARSE_DIR}cs_gaxpy.c
	${SPARSE_DIR}cs_happly.c
	${SPARSE_DIR}cs_house.c
	${SPARSE_DIR}cs_ipvec.c
	${SPARSE_DIR}cs_leaf.c
	${SPARSE_DIR}cs_load.c
	${SPARSE_DIR}cs_lsolve.c
	${SPARSE_DIR}cs_ltsolve.c
	${SPARSE_DIR}cs_lu.c
	${SPARSE_DIR}cs_lusol.c
	${SPARSE_DIR}cs_malloc.c
	${SPARSE_DIR}cs_maxtrans.c
	${SPARSE_DIR}cs_multiply.c
	${SPARSE_DIR}cs_norm.c
	${SPARSE_DIR}cs_permute.c
	${SPARSE_DIR}cs_pinv.c
	${SPARSE_DIR}cs_post.c
	${SPARSE_DIR}cs_print.c
	${SPARSE_DIR}cs_pvec.c
	${SPARSE_DIR}cs_qr.c
	${SPARSE_DIR}cs_qrsol.c
	${SPARSE_DIR}cs_randperm.c
	${SPARSE_DIR}cs_reach.c
	${SPARSE_DIR}cs_scatter.c
	${SPARSE_DIR}cs_scc.c
	${SPARSE_DIR}cs_schol.c
	${SPARSE_DIR}cs_spsolve.c
	${SPARSE_DIR}cs_sqr.c
	${SPARSE_DIR}cs_symperm.c
	${SPARSE_DIR}cs_tdfs.c
	${SPARSE_DIR}cs_transpose.c
	${SPARSE_DIR}cs_updown.c
	${SPARSE_DIR}cs_usolve.c
	${SPARSE_DIR}cs_util.c
	${SPARSE_DIR}cs_utsolve.c
	)
# force suitesparse to be compiled in CXX mode. This is necessary because MSVC
# will compile them in C mode due to their .c file extension but that in turn
# will fail because of the line "#define cs_complex_t double _Complex" in cs.h
# does not work in C mode in MSVC
# this is because MSVC does not support C99...
SET_SOURCE_FILES_PROPERTIES( ${SPARSE_SOURCES} PROPERTIES LANGUAGE CXX )
add_library(cxsparse STATIC ${SPARSE_SOURCES})
IF(EXPORT_SHARED_LIBS)
	add_library(cxsparse_s SHARED ${SPARSE_SOURCES})
ENDIF(EXPORT_SHARED_LIBS)

endif(WIN32)

IF(WITH_B3D)
  SET(B3DDIR ${PROJECT_SOURCE_DIR}/3rdparty/b3dfile/)
  add_library(b3dfile STATIC ${B3DDIR}b3dpsreader.cpp)
  set_target_properties(b3dfile PROPERTIES POSITION_INDEPENDENT_CODE ON)
  IF(EXPORT_SHARED_LIBS)
    add_library(b3dfile_s SHARED ${B3DDIR}b3dpsreader.cpp)
  ENDIF(EXPORT_SHARED_LIBS)
ENDIF(WITH_B3D)

if(NOT ${NEWMAT_FOUND})
# Compile newmat library
SET(NEWMAT_DIR ${PROJECT_SOURCE_DIR}/3rdparty/newmat-10/newmat/)
SET(NEWMAT_SOURCES ${NEWMAT_DIR}newmat1.cpp ${NEWMAT_DIR}newmat2.cpp ${NEWMAT_DIR}newmat3.cpp ${NEWMAT_DIR}newmat4.cpp ${NEWMAT_DIR}newmat5.cpp ${NEWMAT_DIR}newmat6.cpp ${NEWMAT_DIR}newmat7.cpp ${NEWMAT_DIR}newmat8.cpp ${NEWMAT_DIR}newmatex.cpp ${NEWMAT_DIR}bandmat.cpp ${NEWMAT_DIR}submat.cpp ${NEWMAT_DIR}myexcept.cpp ${NEWMAT_DIR}cholesky.cpp ${NEWMAT_DIR}evalue.cpp ${NEWMAT_DIR}fft.cpp ${NEWMAT_DIR}hholder.cpp ${NEWMAT_DIR}jacobi.cpp ${NEWMAT_DIR}newfft.cpp ${NEWMAT_DIR}sort.cpp ${NEWMAT_DIR}svd.cpp ${NEWMAT_DIR}newmatrm.cpp ${NEWMAT_DIR}newmat9.cpp)
add_library(newmat STATIC ${NEWMAT_SOURCES})
IF(EXPORT_SHARED_LIBS)
add_library(newmat_s SHARED ${NEWMAT_SOURCES})
ENDIF(EXPORT_SHARED_LIBS)
endif()

if (WITH_LASLIB)
# Compile las library
set(CMAKE_CXX_FLAGS "-DNDEBUG -DUNORDERED -std=c++0x -fPIC" )
SET(LASLIB_DIR ${PROJECT_SOURCE_DIR}/3rdparty/lastools/laslib/src/)
FILE(GLOB LASLIB_SRCS "${LASLIB_DIR}/*.cpp" "${LASLIB_DIR}/*.hpp")

add_library(LAZ STATIC ${LASLIB_SRCS})
IF(EXPORT_SHARED_LIBS)
add_library(LAZ_s SHARED ${LASLIB_SRCS})
ENDIF(EXPORT_SHARED_LIBS)

include_directories(${PROJECT_SOURCE_DIR}/3rdparty/lastools/laslib/src)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/lastools/laslib/inc)
endif(WITH_LASLIB)

if (WIN32 AND WITH_GLEE)
# Compile GLEE library 
IF (APPLE)
  FIND_LIBRARY(COREFOUNDATION_LIBRARY CoreFoundation )
ENDIF (APPLE)

SET(GLEE_DIR ${PROJECT_SOURCE_DIR}/3rdparty/glee/)
add_library(glee STATIC ${GLEE_DIR}GLee.c)
IF(APPLE)
  target_link_libraries(glee ${COREFOUNDATION_LIBRARY} ${OPENGL_LIBRARIES})
ENDIF(APPLE)

IF(EXPORT_SHARED_LIBS)
add_library(glee_s SHARED ${GLEE_DIR}GLee.c)
IF(APPLE)
  target_link_libraries(glee_s ${COREFOUNDATION_LIBRARY} ${OPENGL_LIBRARIES})
ENDIF(APPLE)
ENDIF(EXPORT_SHARED_LIBS)
endif(WIN32 AND WITH_GLEE)

IF(WIN32)
  add_subdirectory(libzip-1.0.1)
ENDIF()

# Compile AprilTags library if required
add_subdirectory(apriltag-2016-12-01)

add_subdirectory(rply-1.1.4)
