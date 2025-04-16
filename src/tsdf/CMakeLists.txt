find_package(OpenVDB REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenEXR REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OPENVDB_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

# tsdf library
set(TSDF_SOURCES
    TsdSpaceVDB.cc
    TsdVoxelVDB.cc
    Sensor.cc
    SensorPolar3D.cc
)

add_library(tsdf ${TSDF_SOURCES})
target_link_libraries(tsdf
    ${OpenCV_LIBRARIES}
    ${OPENVDB_LIBRARIES}
    ${OPENEXR_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${TBB_LIBRARIES}
)

# scan2vdb
add_executable(scan2vdb scan2vdb.cc)
target_link_libraries(scan2vdb tsdf scan scanio ${TBB_LIBRARIES} ${LIBZIP_LIBRARY} ${NEWMAT_LIBRARIES_SHARED} ${CXSPARSE_LIBRARIES_SHARED} ${ANN_LIBRARIES_SHARED} dl ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${OPENVDB_LIBRARIES} ${OPENEXR_LIBRARIES})

# vdb2mesh
add_executable(vdb2mesh vdb2mesh.cc)
target_link_libraries(vdb2mesh tsdf ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${OPENVDB_LIBRARIES} ${TBB_LIBRARIES} ${OPENEXR_LIBRARIES})

# scan2tsdf
add_executable(scan2tsdf scan2tsdf.cc)
target_link_libraries(scan2tsdf tsdf scan scanio ${LIBZIP_LIBRARY} ${NEWMAT_LIBRARIES_SHARED} ${CXSPARSE_LIBRARIES_SHARED} ${ANN_LIBRARIES_SHARED} dl ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${OPENVDB_LIBRARIES} ${TBB_LIBRARIES} ${OPENEXR_LIBRARIES})
