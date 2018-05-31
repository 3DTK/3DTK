## OpenGL
FIND_PACKAGE(OpenGL REQUIRED)
INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR})
 
## OpenGL
FIND_PACKAGE(GLUT REQUIRED)
INCLUDE_DIRECTORIES(${GLUT_INCLUDE_DIR})

## testmesh - simple mesh I/O
add_executable(testmesh testmesh.cc mesh.cc)
TARGET_LINK_LIBRARIES(testmesh ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
SET_TARGET_PROPERTIES(testmesh PROPERTIES OUTPUT_NAME "testmesh")

## PoissonRecon - KB's implementation
# include_directories(./PoissonRecon/Src)
# add_executable(PoissonRecon ./PoissonRecon/Src/CmdLineParser.cpp ./PoissonRecon/Src/Factor.cpp ./PoissonRecon/Src/Geometry.cpp ./PoissonRecon/Src/MarchingCubes.cpp ./PoissonRecon/Src/PlyFile.cpp ./PoissonRecon/Src/PoissonRecon.cpp)

## Poisson surface reconstruction
## based on KB's implementation, with scanio and normal calculation
set(MESH_CFLAGS "-fpermissive -std=c++0x -msse3 -Wall -finline-functions -Wno-unused-but-set-variable -Wno-write-strings -Wno-char-subscripts -Wno-unused-result -g" CACHE STRING "Additional flags given to the compiler (-O3 -Wall -finline-functions -Wno-write-strings)" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MESH_CFLAGS}")
include_directories(poisson vcglib)
add_executable(poisson poisson/Factor.cpp poisson/Geometry.cpp poisson/MarchingCubes.cpp poisson/MultiGridOctest.cpp poisson.cc)
TARGET_LINK_LIBRARIES(poisson scan_shared ${NORMALS_PANORAMA_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
