# Compile ANN library
SET(ANN_DIR ${CMAKE_SOURCE_DIR}/3rdparty/ann_1.1.1_modified/src/)
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
add_library(ANN STATIC ${ANN_SRCS})
IF(EXPORT_SHARED_LIBS)
add_library(ANN_s SHARED ${ANN_SRCS})
ENDIF(EXPORT_SHARED_LIBS)

# on windows systems the DLL_EXPORTS is required
if (WIN32)
  SET_TARGET_PROPERTIES(ANN PROPERTIES COMPILE_FLAGS -DDLL_EXPORTS)
endif(WIN32)

# Compile glui library
SET(GLUI_DIR ${CMAKE_SOURCE_DIR}/3rdparty/glui/)
add_library(glui STATIC 
 ${GLUI_DIR}glui_add_controls.cpp 
 ${GLUI_DIR}glui.cpp 
 ${GLUI_DIR}glui_bitmap_img_data.cpp 
 ${GLUI_DIR}glui_bitmaps.cpp 
 ${GLUI_DIR}glui_button.cpp
 ${GLUI_DIR}glui_edittext.cpp
 ${GLUI_DIR}glui_checkbox.cpp 
 ${GLUI_DIR}glui_node.cpp 
 ${GLUI_DIR}glui_radio.cpp 
 ${GLUI_DIR}glui_statictext.cpp 
 ${GLUI_DIR}glui_panel.cpp 
 ${GLUI_DIR}glui_separator.cpp 
 ${GLUI_DIR}glui_spinner.cpp 
 ${GLUI_DIR}glui_control.cpp 
 ${GLUI_DIR}glui_column.cpp 
 ${GLUI_DIR}glui_translation.cpp 
 ${GLUI_DIR}glui_rotation.cpp 
 ${GLUI_DIR}glui_mouse_iaction.cpp 
 ${GLUI_DIR}glui_listbox.cpp 
 ${GLUI_DIR}glui_rollout.cpp 
 ${GLUI_DIR}arcball.cpp 
 ${GLUI_DIR}algebra3.cpp
 ${GLUI_DIR}quaternion.cpp )
 


IF(WITH_WXSHOW)

# Compile wxthings library
SET(WXTHINGS_DIR ${CMAKE_SOURCE_DIR}/3rdparty/wxthings/src/)
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

# Compile sparse library 
SET(SPARSE_DIR ${CMAKE_SOURCE_DIR}/3rdparty/sparse/)
add_library(sparse STATIC ${SPARSE_DIR}csparse.cc)
IF(EXPORT_SHARED_LIBS)
add_library(sparse_s SHARED ${SPARSE_DIR}csparse.cc)
ENDIF(EXPORT_SHARED_LIBS)


# Compile newmat library
SET(NEWMAT_DIR ${CMAKE_SOURCE_DIR}/3rdparty/newmat/)
SET(NEWMAT_SOURCES ${NEWMAT_DIR}newmat1.cpp ${NEWMAT_DIR}newmat2.cpp ${NEWMAT_DIR}newmat3.cpp ${NEWMAT_DIR}newmat4.cpp ${NEWMAT_DIR}newmat5.cpp ${NEWMAT_DIR}newmat6.cpp ${NEWMAT_DIR}newmat7.cpp ${NEWMAT_DIR}newmat8.cpp ${NEWMAT_DIR}newmatex.cpp ${NEWMAT_DIR}bandmat.cpp ${NEWMAT_DIR}submat.cpp ${NEWMAT_DIR}myexcept.cpp ${NEWMAT_DIR}cholesky.cpp ${NEWMAT_DIR}evalue.cpp ${NEWMAT_DIR}fft.cpp ${NEWMAT_DIR}hholder.cpp ${NEWMAT_DIR}jacobi.cpp ${NEWMAT_DIR}newfft.cpp ${NEWMAT_DIR}sort.cpp ${NEWMAT_DIR}svd.cpp ${NEWMAT_DIR}newmatrm.cpp ${NEWMAT_DIR}newmat9.cpp)
add_library(newmat STATIC ${NEWMAT_SOURCES})
IF(EXPORT_SHARED_LIBS)
add_library(newmat_s SHARED ${NEWMAT_SOURCES})
ENDIF(EXPORT_SHARED_LIBS)

# Compile las library
set(CMAKE_CXX_FLAGS "-DNDEBUG -DUNORDERED -std=c++0x -fPIC" )
SET(LASLIB_DIR ${CMAKE_SOURCE_DIR}/3rdparty/lastools/laslib/src/)
FILE(GLOB LASLIB_SRCS "${LASLIB_DIR}/*.cpp" "${LASLIB_DIR}/*.hpp")

add_library(LAZ STATIC ${LASLIB_SRCS})
IF(EXPORT_SHARED_LIBS)
add_library(LAZ_s SHARED ${LASLIB_SRCS})
ENDIF(EXPORT_SHARED_LIBS)

# Compile GLEE library 
IF (APPLE)
  FIND_LIBRARY(COREFOUNDATION_LIBRARY CoreFoundation )
ENDIF (APPLE)

SET(GLEE_DIR ${CMAKE_SOURCE_DIR}/3rdparty/glee/)
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
