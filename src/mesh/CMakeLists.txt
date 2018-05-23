## OpenGL
FIND_PACKAGE(OpenGL REQUIRED)
INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR})
 
## OpenGL
FIND_PACKAGE(GLUT REQUIRED)
INCLUDE_DIRECTORIES(${GLUT_INCLUDE_DIR})

add_executable(testmesh testmesh.cc mesh.cc)
TARGET_LINK_LIBRARIES(testmesh ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
SET_TARGET_PROPERTIES(testmesh PROPERTIES OUTPUT_NAME "testmesh")

## PoissonRecon
include_directories(./PoissonRecon/Src)
add_executable(poisson ./PoissonRecon/Src/CmdLineParser.cpp ./PoissonRecon/Src/Factor.cpp ./PoissonRecon/Src/Geometry.cpp ./PoissonRecon/Src/MarchingCubes.cpp ./PoissonRecon/Src/PlyFile.cpp ./PoissonRecon/Src/PoissonRecon.cpp)
# add_executable(poisson poisson.cc)