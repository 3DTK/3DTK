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

## poisson - 3DTK poisson based on KB's implementation
# include_directories(${CXSPARSE_INCLUDE_DIRS})
# include_directories(${NEWMAT_INCLUDE_DIRS})
# include_directories(${ANN_INCLUDE_DIRS})

# set(SCANIO_LIBNAMES
#   faro_xyz_rgbr ks ks_rgb leica_xyzr ply pts ptsr pts_rgb pts_rgbr pts_rrgb riegl_rgb riegl_txt rts uos uosr uos_rgb uos_rgbr uos_rrgb uos_rrgbt velodyne xyz xyzr xyz_rgb xyz_rgba xyz_rgbr xyz_rrgb
# )

# if(WITH_B3D)
#   set(SCANIO_LIBNAMES ${SCANIO_LIBNAMES} b3d)
# endif()

# if(WITH_RIVLIB)
#   set(SCANIO_LIBNAMES ${SCANIO_LIBNAMES} rxp)
# endif()

# if(WITH_LASLIB)
#   set(SCANIO_LIBNAMES ${SCANIO_LIBNAMES} laz)
#   include_directories(${PROJECT_SOURCE_DIR}/3rdparty/lastools/)
# endif()

add_executable(poisson poisson.cc)
TARGET_LINK_LIBRARIES(poisson scan_shared ${NORMALS_PANORAMA_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
