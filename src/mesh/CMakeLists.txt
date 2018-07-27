## OpenGL
FIND_PACKAGE(OpenGL REQUIRED)
INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR})
 
## OpenGL
FIND_PACKAGE(GLUT REQUIRED)
INCLUDE_DIRECTORIES(${GLUT_INCLUDE_DIR})

## CGAL (CGAL has to be put forehead)
# find_package(CGAL QUIET COMPONENTS Core)
# include(${CGAL_USE_FILE})

## Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

## testmesh - simple mesh I/O
# add_executable(testmesh testmesh.cc mesh.cc)
# TARGET_LINK_LIBRARIES(testmesh ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
# SET_TARGET_PROPERTIES(testmesh PROPERTIES OUTPUT_NAME "testmesh")

## PoissonRecon - KB's implementation
# include_directories(./PoissonRecon/Src)
# add_executable(PoissonRecon ./PoissonRecon/Src/CmdLineParser.cpp ./PoissonRecon/Src/Factor.cpp ./PoissonRecon/Src/Geometry.cpp ./PoissonRecon/Src/MarchingCubes.cpp ./PoissonRecon/Src/PlyFile.cpp ./PoissonRecon/Src/PoissonRecon.cpp)

## Poisson surface reconstruction
## based on KB's implementation, with scanio and normal calculation
string(REPLACE "-O3" "-O0" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(MESH_ADDITIONAL_CFLAGS "-fpermissive -g" CACHE STRING "Additional flags of MESH module given to the compiler" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CFLAGS} ${MESH_ADDITIONAL_CFLAGS}")
include_directories(poisson vcglib)
add_executable(recon recon.cc poisson.cc poisson/PlyFile.cpp)
TARGET_LINK_LIBRARIES(recon scan_shared normals_panorama_shared dl ${NORMALS_PANORAMA_LIBRARIES} ${Boost_LIBS} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
