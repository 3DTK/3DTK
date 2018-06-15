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
string(REPLACE "-O3" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(MESH_ADDITIONAL_CFLAGS "-fpermissive -g" CACHE STRING "Additional flags of MESH module given to the compiler" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CFLAGS} ${MESH_ADDITIONAL_CFLAGS}")
include_directories(poisson vcglib)
add_executable(poisson poisson/Factor.cpp poisson/Geometry.cpp poisson/MarchingCubes.cpp poisson/MultiGridOctest.cpp poisson.cc vcglib/wrap/ply/plylib.cpp)
TARGET_LINK_LIBRARIES(poisson scan_shared ${NORMALS_PANORAMA_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
