# ICE on MSVC:
# c:\projects\3dtk\src\mesh\poisson\marchingcubes.h(532): fatal error C1001: An internal error has occurred in the compiler.
if(NOT MSVC)
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

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.9)
	message(WARNING "Not compiling mesh component in gcc versions less than 4.9 because of ICE")
else()
## recon program
string(REPLACE "-O3" "-O0" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(MESH_ADDITIONAL_CFLAGS "-fpermissive -g" CACHE STRING "Additional flags of MESH module given to the compiler" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CFLAGS} ${MESH_ADDITIONAL_CFLAGS}")

include_directories(poisson vcglib)

# needs opencv because of normals_panorama
if (WITH_OPENCV)
add_executable(recon recon.cc poisson.cc auto_scan_red.cc parse_options.cc calc_normals.cc recon_commons.cc poisson/PlyFile.cpp)
TARGET_LINK_LIBRARIES(recon scan normals_panorama ${NORMALS_PANORAMA_LIBRARIES} ${Boost_LIBS} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
if (UNIX)
  target_link_libraries(recon dl)
endif()
endif()
endif()
endif()
