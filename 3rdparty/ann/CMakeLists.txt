include_directories("${PROJECT_SOURCE_DIR}/3rdparty/ann/ann_1.1.1_modified/include")

SET(ANN_DIR ${PROJECT_SOURCE_DIR}/3rdparty/ann/ann_1.1.1_modified/src/)
add_library(ann
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

# on windows systems the DLL_EXPORTS is required
if (WIN32)
  target_compile_definitions(ann PUBLIC "-DDLL_EXPORTS")
endif(WIN32)
