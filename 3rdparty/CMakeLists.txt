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
add_library(ANN_s SHARED ${ANN_SRCS})
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
 

# Compile sparse library 
SET(SPARSE_DIR ${CMAKE_SOURCE_DIR}/3rdparty/sparse/)
add_library(sparse STATIC ${SPARSE_DIR}csparse.cc)
add_library(sparse_s SHARED ${SPARSE_DIR}csparse.cc)


# Compile newmat library
SET(NEWMAT_DIR ${CMAKE_SOURCE_DIR}/3rdparty/newmat/)
SET(NEWMAT_SOURCES ${NEWMAT_DIR}newmat1.cpp ${NEWMAT_DIR}newmat2.cpp ${NEWMAT_DIR}newmat3.cpp ${NEWMAT_DIR}newmat4.cpp ${NEWMAT_DIR}newmat5.cpp ${NEWMAT_DIR}newmat6.cpp ${NEWMAT_DIR}newmat7.cpp ${NEWMAT_DIR}newmat8.cpp ${NEWMAT_DIR}newmatex.cpp ${NEWMAT_DIR}bandmat.cpp ${NEWMAT_DIR}submat.cpp ${NEWMAT_DIR}myexcept.cpp ${NEWMAT_DIR}cholesky.cpp ${NEWMAT_DIR}evalue.cpp ${NEWMAT_DIR}fft.cpp ${NEWMAT_DIR}hholder.cpp ${NEWMAT_DIR}jacobi.cpp ${NEWMAT_DIR}newfft.cpp ${NEWMAT_DIR}sort.cpp ${NEWMAT_DIR}svd.cpp ${NEWMAT_DIR}newmatrm.cpp ${NEWMAT_DIR}newmat9.cpp)
add_library(newmat STATIC ${NEWMAT_SOURCES})
add_library(newmat_s SHARED ${NEWMAT_SOURCES})
#add_library(newmat STATIC ${NEWMAT_DIR}newmat1.cpp ${NEWMAT_DIR}newmat2.cpp ${NEWMAT_DIR}newmat3.cpp ${NEWMAT_DIR}newmat4.cpp ${NEWMAT_DIR}newmat5.cpp ${NEWMAT_DIR}newmat6.cpp ${NEWMAT_DIR}newmat7.cpp ${NEWMAT_DIR}newmat8.cpp ${NEWMAT_DIR}newmatex.cpp ${NEWMAT_DIR}bandmat.cpp ${NEWMAT_DIR}submat.cpp ${NEWMAT_DIR}myexcept.cpp ${NEWMAT_DIR}cholesky.cpp ${NEWMAT_DIR}evalue.cpp ${NEWMAT_DIR}fft.cpp ${NEWMAT_DIR}hholder.cpp ${NEWMAT_DIR}jacobi.cpp ${NEWMAT_DIR}newfft.cpp ${NEWMAT_DIR}sort.cpp ${NEWMAT_DIR}svd.cpp ${NEWMAT_DIR}newmatrm.cpp ${NEWMAT_DIR}newmat9.cpp)

