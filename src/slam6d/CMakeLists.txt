### SCAN_RED

IF(WITH_SCAN_REDUCTION) 
  add_executable(scan_red scan_red.cc)
  
  IF(UNIX)
    target_link_libraries(scan_red scanlib dl ANN)
  ENDIF(UNIX)

  IF (WIN32)
    target_link_libraries(scan_red scanlib ANN XGetopt)
  ENDIF(WIN32)
ENDIF(WITH_SCAN_REDUCTION)

### SCAN_DIFF
IF(WITH_SCAN_DIFF)
  add_executable(scan_diff scan_diff.cc)
  add_executable(scan_diff2d scan_diff2d.cc ../show/colormanager.cc)
  
  IF(UNIX)
    target_link_libraries(scan_diff scanlib dl ANN)
    target_link_libraries(scan_diff2d scanlib dl ANN)
  ENDIF(UNIX)

  IF (WIN32)
    target_link_libraries(scan_diff scanlib ANN XGetopt)
    target_link_libraries(scan_diff2d scanlib ANN XGetopt)
  ENDIF(WIN32)
ENDIF(WITH_SCAN_DIFF)

### TOOLS

IF(WITH_TOOLS)
  add_executable(frame_to_graph frame_to_graph.cc)
  add_executable(convergence convergence.cc)
  add_executable(graph_balancer graph_balancer.cc)
  add_executable(exportPoints exportPoints.cc)
  add_executable(frames2riegl frames2riegl.cc)
  add_executable(frames2pose frames2pose.cc)
  add_executable(pose2frames pose2frames.cc)
  add_executable(riegl2frames riegl2frames.cc)
  add_executable(toGlobal toGlobal.cc)

  IF(UNIX)
    target_link_libraries(graph_balancer scanlib ${Boost_GRAPH_LIBRARY} ${Boost_SERIALIZATION_LIBRARY} ${Boost_REGEX_LIBRARY})
    target_link_libraries(exportPoints scanlib dl ANN)
    target_link_libraries(toGlobal scanlib)
  ENDIF(UNIX)

  
  IF (WIN32)
    target_link_libraries(frame_to_graph XGetopt ${Boost_LIBRARIES})
    target_link_libraries(convergence XGetopt ${Boost_LIBRARIES})
    target_link_libraries(graph_balancer scanlib XGetopt  ${Boost_LIBRARIES})
    target_link_libraries(exportPoints scanlib ANN XGetopt  ${Boost_LIBRARIES})
    target_link_libraries(frames2pose XGetopt ${Boost_LIBRARIES})
    target_link_libraries(pose2frames XGetopt ${Boost_LIBRARIES})
    target_link_libraries(frames2riegl XGetopt ${Boost_LIBRARIES})
    target_link_libraries(riegl2frames XGetopt ${Boost_LIBRARIES})
	target_link_libraries(toGlobal XGetopt ${Boost_LIBRARIES})
  ENDIF(WIN32)
ENDIF(WITH_TOOLS)

### SCANLIB

SET(SCANLIB_SRCS
  kd.cc             kdManaged.cc      kdMeta.cc         graphSlam6D.cc 
  graph.cc          icp6Dapx.cc       icp6D.cc          icp6Dsvd.cc 
  icp6Dortho.cc     icp6Dquat.cc      icp6Dhelix.cc     icp6Dlumeuler.cc 
  icp6Dlumquat.cc   icp6Ddual.cc      lum6Deuler.cc     lum6Dquat.cc
  ghelix6DQ2.cc     gapx6D.cc         graphToro.cc      ann_kd.cc
  graphHOG-Man.cc   elch6D.cc         elch6Dquat.cc     elch6DunitQuat.cc 
  elch6Dslerp.cc    elch6Deuler.cc    loopToro.cc       loopHOG-Man.cc    
  point_type.cc	    icp6Dquatscale.cc searchTree.cc     Boctree.cc
  allocator.cc      d2tree.cc         
  scan.cc           basicScan.cc      managedScan.cc    metaScan.cc
  io_types.cc       pointfilter.cc
  )

if(WITH_METRICS)
  set(SCANLIB_SRCS ${SCANLIB_SRCS} metrics.cc)
endif(WITH_METRICS)

add_library(scanlib STATIC ${SCANLIB_SRCS})

target_link_libraries(scanlib scanclient scanio)

IF(UNIX)
  target_link_libraries(scanlib dl)
ENDIF(UNIX)

### EXPORT SHARED LIBS

IF(EXPORT_SHARED_LIBS)
add_library(scanlib_s SHARED ${SCANLIB_SRCS})
#target_link_libraries(scanlib_s ${Boost_LIBRARIES} newmat)
ENDIF(EXPORT_SHARED_LIBS)

### SLAM6D

IF(WITH_CUDA)
  CUDA_COMPILE(CUDA_FILES cuda/CIcpGpuCuda.cu )
  add_executable(slam6D slam6D.cc cuda/icp6Dcuda.cc ${CUDA_FILES})
  target_link_libraries(slam6D ${CUDA_LIBRARIES} ANN cudpp64)
  CUDA_ADD_CUBLAS_TO_TARGET(slam6D)
  CUDA_ADD_CUTIL_TO_TARGET(slam6D)
ELSE(WITH_CUDA)
  add_executable(slam6D slam6D.cc)
ENDIF(WITH_CUDA)

IF(UNIX)
  target_link_libraries(slam6D scanlib newmat sparse ANN)
ENDIF(UNIX)

IF(WIN32)
  target_link_libraries(slam6D scanlib newmat sparse ANN XGetopt ${Boost_LIBRARIES})
ENDIF(WIN32)

#IF(MSVC)
#	INSTALL(TARGETS slam6D RUNTIME DESTINATION ${CMAKE_SOURCE_DIR}/windows)
#ENDIF(MSVC)
