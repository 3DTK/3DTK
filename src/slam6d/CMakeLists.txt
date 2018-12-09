## e57 to scan tool
if(WITH_E57)
  message(STATUS "With e57 Support.")
  add_definitions(-DWITH_E57)
  include_directories(${LIBE57_INCLUDE_DIRS})
  add_executable(e572scan e572scan.cpp)
  target_link_libraries(e572scan E57RefImpl ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
else()
  message(STATUS "Whitout e57 Support.")
endif()

if(WITH_METRICS)
  message(STATUS "With metrics in slam6d.")
  add_definitions(-DWITH_METRICS)
else()
  message(STATUS "Without metrics in slam6d.")
endif()

include_directories(${CXSPARSE_INCLUDE_DIRS})
include_directories(${NEWMAT_INCLUDE_DIRS})
include_directories(${ANN_INCLUDE_DIRS})

### TOOLS

### SCAN_RED

if (NOT MSVC AND WITH_OPENCV)
  add_executable(scan_red scan_red.cc ../scanio/writer.cc fbr/fbr_global.cc fbr/panorama.cc fbr/scan_cv.cc)

  if(MSVC)
    target_link_libraries(scan_red scan_shared show_shared ${ANN_LIBRARIES_STATIC} fbr_cv_io_shared fbr_panorama_shared ${OpenCV_LIBS} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${SHOW_LIBS_SHARED})
  else()
    target_link_libraries(scan_red scan_shared show_shared ${ANN_LIBRARIES_SHARED} fbr_cv_io_shared fbr_panorama_shared ${OpenCV_LIBS} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${SHOW_LIBS_SHARED})
  endif()
endif()

### SCAN_DIFF
add_executable(scan_diff scan_diff.cc)
if (WIN32)
  target_link_libraries(scan_diff XGetopt scan_static ${ANN_LIBRARIES_STATIC} ${NEWMAT_LIBRARIES_SHARED} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
else()
  target_link_libraries(scan_diff scan_shared ${ANN_LIBRARIES_SHARED} ${NEWMAT_LIBRARIES_SHARED} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
endif()

add_executable(frame_to_graph frame_to_graph.cc)
add_executable(convergence convergence.cc)
add_executable(graph_balancer graph_balancer.cc)
add_executable(frames2riegl frames2riegl.cc)
add_executable(frames2pose frames2pose.cc)
add_executable(kitti2scan kitti2scan.cc)
add_executable(kitti2pose kitti2pose.cc)
add_executable(frames2kitti frames2kitti.cc)
add_executable(framesdiff2frames framesdiff2frames.cc)
add_executable(pose2frames pose2frames.cc)
add_executable(riegl2frames riegl2frames.cc)
add_executable(transformFrames transformFrames.cc)
add_executable(multFrames multFrames.cc)

#  add_executable(vigo23dtk vigo23dtk.cc)
#  add_executable(g23dtk g23dtk.cc)
add_executable(toGlobal toGlobal.cc)
add_executable(average6DoFposes average6DoFposes.cc)
add_executable(align sICP.cc)

if (WITH_OPENCV)
  add_executable(exportPoints exportPoints.cc ../scanio/writer.cc)
  if (WIN32)
    target_link_libraries(exportPoints scan_static ${ANN_LIBRARIES_STATIC} ${NEWMAT_LIBRARIES_SHARED} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${OpenCV_LIBS})
  else()
    target_link_libraries(exportPoints scan_shared ${ANN_LIBRARIES_SHARED} ${NEWMAT_LIBRARIES_SHARED} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${OpenCV_LIBS})
  endif()
endif()

if(UNIX)
  target_link_libraries(graph_balancer scan_shared ${Boost_GRAPH_LIBRARY} ${Boost_SERIALIZATION_LIBRARY} ${Boost_REGEX_LIBRARY} ${Boost_SYSTEM_LIBRARY})
  target_link_libraries(align scan_shared ${ANN_LIBRARIES_SHARED} ${NEWMAT_LIBRARIES_SHARED} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
  target_link_libraries(transformFrames scan_shared ${ANN_LIBRARIES} newmat ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
  target_link_libraries(multFrames scan_shared ${ANN_LIBRARIES} newmat ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
  target_link_libraries(toGlobal scan_shared)
  target_link_libraries(convergence ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
  target_link_libraries(pose2frames ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
  target_link_libraries(frames2pose ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
  target_link_libraries(frames2riegl ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
  target_link_libraries(riegl2frames ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
  target_link_libraries(exportPoints ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
endif()

if(APPLE)
  target_link_libraries(graph_balancer scan_shared ${Boost_GRAPH_LIBRARY} ${Boost_SERIALIZATION_LIBRARY} ${Boost_REGEX_LIBRARY} ${Boost_SYSTEM_LIBRARY})
  target_link_libraries(align scan_shared dl ${ANN_LIBRARIES} newmat ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
  target_link_libraries(transformFrames scan_shared dl ${ANN_LIBRARIES} newmat ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
  target_link_libraries(multFrames scan_shared dl ${ANN_LIBRARIES} newmat ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
  target_link_libraries(toGlobal scan)
  target_link_libraries(convergence ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
  target_link_libraries(pose2frames ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
  target_link_libraries(frames2pose ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
  target_link_libraries(frames2riegl ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
  target_link_libraries(riegl2frames ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
endif()

if (WIN32)
  target_link_libraries(frame_to_graph XGetopt ${Boost_LIBRARIES})
  target_link_libraries(convergence XGetopt ${Boost_LIBRARIES})
  target_link_libraries(graph_balancer scan_static XGetopt ${Boost_GRAPH_LIBRARY} ${Boost_SERIALIZATION_LIBRARY} ${Boost_REGEX_LIBRARY} ${Boost_LIBRARIES})
  target_link_libraries(align scan_static ${ANN_LIBRARIES_STATIC} ${Boost_LIBRARIES} ${NEWMAT_LIBRARIES_SHARED})
  target_link_libraries(transformFrames scan_static ${ANN_LIBRARIES_STATIC} ${Boost_LIBRARIES} ${NEWMAT_LIBRARIES_SHARED})
  target_link_libraries(multFrames scan_static ${ANN_LIBRARIES_STATIC} ${Boost_LIBRARIES} ${NEWMAT_LIBRARIES_SHARED})
  target_link_libraries(frames2pose ${Boost_LIBRARIES})
  target_link_libraries(kitti2scan XGetopt ${Boost_LIBRARIES})
  target_link_libraries(kitti2pose XGetopt ${Boost_LIBRARIES})
  target_link_libraries(frames2kitti XGetopt ${Boost_LIBRARIES})
  target_link_libraries(framesdiff2frames XGetopt ${Boost_LIBRARIES})
  target_link_libraries(pose2frames ${Boost_LIBRARIES})
  target_link_libraries(frames2riegl  ${Boost_LIBRARIES})
  target_link_libraries(riegl2frames ${Boost_LIBRARIES})
  target_link_libraries(toGlobal scan_static XGetopt ${Boost_LIBRARIES})
endif()

target_link_libraries(average6DoFposes ${NEWMAT_LIBRARIES_SHARED})

### SCANLIB

add_library(scanlib_objects OBJECT
        kd.cc             kdManaged.cc      kdMeta.cc         graphSlam6D.cc
        graph.cc          icp6D.cc          icp6Dapx.cc       icp6Dsvd.cc
        icp6Dortho.cc     icp6Dquat.cc      icp6Dhelix.cc     icp6Dlumeuler.cc
        icp6Dlumquat.cc   icp6Ddual.cc      lum6Deuler.cc     lum6Dquat.cc
        ghelix6DQ2.cc     gapx6D.cc         ann_kd.cc         elch6D.cc
        elch6Dquat.cc     elch6DunitQuat.cc elch6Dslerp.cc    elch6Deuler.cc
        point_type.cc     icp6Dquatscale.cc searchTree.cc     Boctree.cc
        scan.cc           basicScan.cc      managedScan.cc    metaScan.cc
        io_types.cc       io_utils.cc       pointfilter.cc    allocator.cc
        icp6Dnapx.cc      normals.cc        kdIndexed.cc
        )
# compile with -fPIC so that the result can be made into a shared library
set_property(TARGET scanlib_objects PROPERTY POSITION_INDEPENDENT_CODE 1)

add_library(scan_static STATIC $<TARGET_OBJECTS:scanlib_objects>)
add_library(scan_shared SHARED $<TARGET_OBJECTS:scanlib_objects>)

set_target_properties(scan_static PROPERTIES OUTPUT_NAME scan CLEAN_DIRECT_OUTPUT 1)
set_target_properties(scan_shared PROPERTIES OUTPUT_NAME scan CLEAN_DIRECT_OUTPUT 1)

if(MSVC)
  target_link_libraries(scan_static scanclient_static scanio_static ${ANN_LIBRARIES_STATIC} ${NEWMAT_LIBRARIES_STATIC} ${CXSPARSE_LIBRARIES})
else()
  target_link_libraries(scan_static scanclient_static scanio_static ${ANN_LIBRARIES} ${NEWMAT_LIBRARIES_STATIC} ${CXSPARSE_LIBRARIES})
endif()
target_link_libraries(scan_shared scanclient_shared scanio_shared ${ANN_LIBRARIES_SHARED} ${NEWMAT_LIBRARIES_SHARED} ${CXSPARSE_LIBRARIES})

### SLAM6D

add_executable(slam6D slam6D.cc)

if (WIN32)
  target_link_libraries(slam6D scan_static ${NEWMAT_LIBRARIES_SHARED} ${CXSPARSE_LIBRARIES} ${ANN_LIBRARIES_STATIC} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
else()
  target_link_libraries(slam6D scan_shared ${NEWMAT_LIBRARIES_SHARED} ${CXSPARSE_LIBRARIES} ${ANN_LIBRARIES_SHARED} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
endif()

#if(MSVC)
#	install(TARGETS slam6D RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/windows)
#endif()
