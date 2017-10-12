include_directories("${PROJECT_SOURCE_DIR}/3rdparty/ann/ann_1.1.1_modified/include")

SET(ANN_DIR ${PROJECT_SOURCE_DIR}/3rdparty/ann/ann_1.1.1_modified/src/)
add_library(ann_objects OBJECT
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

# compile with -fPIC so that the result can be made into a shared library
set_property(TARGET ann_objects PROPERTY POSITION_INDEPENDENT_CODE 1)

add_library(ann_static STATIC $<TARGET_OBJECTS:ann_objects>)
add_library(ann_shared SHARED $<TARGET_OBJECTS:ann_objects>)

# on windows systems the DLL_EXPORTS is required
if (WIN32)
  SET_TARGET_PROPERTIES(ann PROPERTIES COMPILE_FLAGS -DDLL_EXPORTS)
endif(WIN32)

set_target_properties(ann_static PROPERTIES OUTPUT_NAME ann CLEAN_DIRECT_OUTPUT 1)
set_target_properties(ann_shared PROPERTIES OUTPUT_NAME ann CLEAN_DIRECT_OUTPUT 1)
