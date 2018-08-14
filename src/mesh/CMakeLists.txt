## OpenGL
if (WITH_OPENGL)
  FIND_PACKAGE(OpenGL REQUIRED)
  FIND_PACKAGE(GLUT REQUIRED)
  INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR})
  INCLUDE_DIRECTORIES(${GLUT_INCLUDE_DIR})
  add_definitions(-DWITH_OPENGL)
endif()

## Eigen
if (WITH_EIGEN3)
  find_package(Eigen3 REQUIRED)
  include_directories(${EIGEN3_INCLUDE_DIR})
endif()

## recon program
string(REPLACE "-O3" "-O0" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(MESH_ADDITIONAL_CFLAGS "-fpermissive -g" CACHE STRING "Additional flags of MESH module given to the compiler" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CFLAGS} ${MESH_ADDITIONAL_CFLAGS}")

include_directories(poisson vcglib)
add_executable(recon recon.cc poisson.cc auto_scan_red.cc parse_options.cc calc_normals.cc recon_commons.cc poisson/PlyFile.cpp)
TARGET_LINK_LIBRARIES(recon scan_shared normals_panorama_shared dl ${NORMALS_PANORAMA_LIBRARIES} ${Boost_LIBS} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
