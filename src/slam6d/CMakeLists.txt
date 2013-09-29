### TOOLS

IF(WITH_TOOLS)
IF(WITH_FBR)
  ### SCAN_RED
  add_executable(scan_red scan_red.cc fbr/fbr_global.cc fbr/panorama.cc fbr/scan_cv.cc)

  IF(WITH_GLEE)
    SET(SHOW_LIBS ${SHOW_LIBS} glee)
  ENDIF(WITH_GLEE)
  
  IF(UNIX)
    target_link_libraries(scan_red scan dl ANN fbr_cv_io fbr_panorama ${OpenCV_LIBS} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} showstatic ${OPENGL_LIBRARIES} ${SHOW_LIBS} newmat)
  ENDIF(UNIX)

  IF (WIN32)
    target_link_libraries(scan_red scan ANN fbr_cv_io fbr_panorama ${OpenCV_LIBS} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} showstatic XGetopt newmat)
  ENDIF(WIN32)
ENDIF(WITH_FBR)

  ### SCAN_DIFF
  add_executable(scan_diff scan_diff.cc)
  #  add_executable(scan_diff2d scan_diff2d.cc ../show/colormanager.cc)
  
  IF(UNIX)
    target_link_libraries(scan_diff scan dl ANN newmat ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
    #    target_link_libraries(scan_diff2d scan dl ANN)
  ENDIF(UNIX)

  IF (WIN32)
    target_link_libraries(scan_diff scan ANN XGetopt newmat)
    # target_link_libraries(scan_diff2d scan dl ANN XGetopt)
  ENDIF(WIN32)

  add_executable(frame_to_graph frame_to_graph.cc)
  add_executable(convergence convergence.cc)
  add_executable(graph_balancer graph_balancer.cc)
  add_executable(exportPoints exportPoints.cc)
  add_executable(frames2riegl frames2riegl.cc)
  add_executable(frames2pose frames2pose.cc)
  add_executable(framesdiff2frames framesdiff2frames.cc)
  add_executable(pose2frames pose2frames.cc)
  add_executable(riegl2frames riegl2frames.cc)
  add_executable(toGlobal toGlobal.cc)

  IF(UNIX)
    target_link_libraries(graph_balancer scan ${Boost_GRAPH_LIBRARY} ${Boost_SERIALIZATION_LIBRARY} ${Boost_REGEX_LIBRARY} ${Boost_SYSTEM_LIBRARY})
    target_link_libraries(exportPoints scan dl ANN newmat ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
    target_link_libraries(toGlobal scan)
    target_link_libraries(convergence ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
    target_link_libraries(pose2frames ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
  ENDIF(UNIX)

  
  IF (WIN32)
    target_link_libraries(frame_to_graph XGetopt ${Boost_LIBRARIES})
    target_link_libraries(convergence XGetopt ${Boost_LIBRARIES})
    target_link_libraries(graph_balancer scan XGetopt  ${Boost_LIBRARIES})
    target_link_libraries(exportPoints scan ANN XGetopt  ${Boost_LIBRARIES} newmat)
    target_link_libraries(frames2pose XGetopt ${Boost_LIBRARIES})
    target_link_libraries(framesdiff2frames XGetopt ${Boost_LIBRARIES})
    target_link_libraries(pose2frames XGetopt ${Boost_LIBRARIES})
    target_link_libraries(frames2riegl XGetopt ${Boost_LIBRARIES})
    target_link_libraries(riegl2frames XGetopt ${Boost_LIBRARIES})
    target_link_libraries(toGlobal scan XGetopt ${Boost_LIBRARIES})
  ENDIF(WIN32)

ENDIF(WITH_TOOLS)

### SCANLIB

SET(SCANLIB_SRCS
  kd.cc             kdManaged.cc      kdMeta.cc         graphSlam6D.cc 
  graph.cc          icp6D.cc          icp6Dapx.cc       icp6Dsvd.cc 
  icp6Dortho.cc     icp6Dquat.cc      icp6Dhelix.cc     icp6Dlumeuler.cc 
  icp6Dlumquat.cc   icp6Ddual.cc      lum6Deuler.cc     lum6Dquat.cc
  ghelix6DQ2.cc     gapx6D.cc         ann_kd.cc         elch6D.cc
  elch6Dquat.cc     elch6DunitQuat.cc elch6Dslerp.cc    elch6Deuler.cc    
  point_type.cc	icp6Dquatscale.cc searchTree.cc     Boctree.cc
  scan.cc           basicScan.cc      managedScan.cc    metaScan.cc
  io_types.cc       io_utils.cc       pointfilter.cc    allocator.cc
  icp6Dnapx.cc      normals.cc
  )

if(WITH_METRICS)
  set(SCANLIB_SRCS ${SCANLIB_SRCS} metrics.cc)
endif(WITH_METRICS)

add_library(scan STATIC ${SCANLIB_SRCS})

FIND_PACKAGE(OpenCV REQUIRED)

target_link_libraries(scan scanclient scanio ANN newmat sparse)

IF(UNIX)
  target_link_libraries(scan dl pthread)
ENDIF(UNIX)

### EXPORT SHARED LIBS

IF(EXPORT_SHARED_LIBS)
add_library(scan_s SHARED ${SCANLIB_SRCS})
target_link_libraries(scan_s scanio scanclient_s newmat_s sparse_s ANN_s)
ENDIF(EXPORT_SHARED_LIBS)

### SLAM6D

add_executable(slam6D slam6D.cc)

IF(UNIX)
	target_link_libraries(slam6D scan newmat sparse ANN ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
ENDIF(UNIX)

IF(WIN32)
  target_link_libraries(slam6D scan newmat sparse ANN XGetopt ${Boost_LIBRARIES})
ENDIF(WIN32)

#IF(MSVC)
#	INSTALL(TARGETS slam6D RUNTIME DESTINATION ${CMAKE_SOURCE_DIR}/windows)
#ENDIF(MSVC)
