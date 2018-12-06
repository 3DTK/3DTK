add_library(spherical_quadtree spherical_quadtree.cc)
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set_property(TARGET spherical_quadtree PROPERTY POSITION_INDEPENDENT_CODE 1)
endif()
