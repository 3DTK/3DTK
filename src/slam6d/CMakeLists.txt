IF(WITH_SCAN_REDUCTION) 
  add_executable(scan_red scan_red.cc)
  
  IF(UNIX)
    target_link_libraries(scan_red scanlib dl ANN)
  ENDIF(UNIX)

  IF (WIN32)
    target_link_libraries(scan_red scanlib ANN XGetopt)
  ENDIF(WIN32)
ENDIF(WITH_SCAN_REDUCTION)

IF(WITH_SCAN_DIFF) 
  add_executable(scan_diff scan_diff.cc)
  
  IF(UNIX)
    target_link_libraries(scan_diff scanlib dl ANN)
  ENDIF(UNIX)

  IF (WIN32)
    target_link_libraries(scan_diff scanlib ANN XGetopt)
  ENDIF(WIN32)
ENDIF(WITH_SCAN_DIFF)


add_library(scan_io_uos             SHARED scan_io_uos.cc            )
add_library(scan_io_uos_map         SHARED scan_io_uos_map.cc        )
add_library(scan_io_uos_frames      SHARED scan_io_uos_frames.cc     )
add_library(scan_io_uos_map_frames  SHARED scan_io_uos_map_frames.cc )
add_library(scan_io_old             SHARED scan_io_old.cc            )
add_library(scan_io_x3d             SHARED scan_io_x3d.cc            )
add_library(scan_io_asc             SHARED scan_io_asc.cc            )
add_library(scan_io_rts             SHARED scan_io_rts.cc            )
add_library(scan_io_iais            SHARED scan_io_iais.cc           )
add_library(scan_io_rts_map         SHARED scan_io_rts_map.cc        )
add_library(scan_io_front           SHARED scan_io_front.cc          )
add_library(scan_io_riegl_txt       SHARED scan_io_riegl_txt.cc      )
add_library(scan_io_riegl_rgb       SHARED scan_io_riegl_rgb.cc      )
add_library(scan_io_riegl_bin       SHARED scan_io_riegl_bin.cc      )
add_library(scan_io_zuf             SHARED scan_io_zuf.cc            )
add_library(scan_io_xyz             SHARED scan_io_xyz.cc            )
add_library(scan_io_ifp             SHARED scan_io_ifp.cc            )
add_library(scan_io_ply             SHARED scan_io_ply.cc            )
add_library(scan_io_wrl             SHARED scan_io_wrl.cc            )
add_library(scan_io_zahn            SHARED scan_io_zahn.cc           )
add_library(scan_io_ais             SHARED scan_io_ais.cc            )
add_library(scan_io_uos_rgb         SHARED scan_io_uos_rgb.cc        )
add_library(scan_io_oct             SHARED scan_io_oct.cc            )
 target_link_libraries(scan_io_oct scanlib)
add_library(scan_io_xyzr            SHARED scan_io_xyzr.cc           )

IF (WITH_RIVLIB)
  add_library(scan_io_rxp           SHARED scan_io_rxp.cc            )
  target_link_libraries(scan_io_rxp ${RIVLIB})
ENDIF(WITH_RIVLIB) 

IF(WITH_TOOLS)
  add_executable(frame_to_graph frame_to_graph.cc)
  add_executable(convergence convergence.cc)
  add_executable(graph_balancer graph_balancer.cc)
  add_executable(exportPoints exportPoints.cc)
  add_executable(frames2riegl frames2riegl.cc)
  add_executable(riegl2frames riegl2frames.cc)
  
  IF(UNIX)
    target_link_libraries(graph_balancer scanlib ${Boost_LIBRARIES})
    target_link_libraries(exportPoints scanlib dl ANN)
  ENDIF(UNIX)

  
  IF (WIN32)
    target_link_libraries(frame_to_graph XGetopt)
    target_link_libraries(convergence XGetopt)
    target_link_libraries(graph_balancer boost_graph-mt scanlib XGetopt)
    target_link_libraries(exportPoints scanlib ANN XGetopt)
    target_link_libraries(frames2riegl XGetopt)
    target_link_libraries(riegl2frames XGetopt)
  ENDIF(WIN32)

ENDIF(WITH_TOOLS)

SET(SCANLIB_SRCS
  kd.cc kdc.cc      scan.cc         d2tree.cc         graphSlam6D.cc 
  graph.cc          icp6Dapx.cc     icp6D.cc          icp6Dsvd.cc 
  icp6Dortho.cc     icp6Dquat.cc    icp6Dhelix.cc     icp6Dlumeuler.cc 
  icp6Dlumquat.cc   icp6Ddual.cc    lum6Deuler.cc     lum6Dquat.cc
  ghelix6DQ2.cc     gapx6D.cc       graphToro.cc      graphHOG-Man.cc 
  elch6D.cc         elch6Dquat.cc   elch6DunitQuat.cc elch6Dslerp.cc 
  elch6Deuler.cc    loopToro.cc     loopHOG-Man.cc    point_type.cc 
  )

add_library(scanlib STATIC ${SCANLIB_SRCS})
IF(EXPORT_SHARED_LIBS)
add_library(scanlib_s SHARED ${SCANLIB_SRCS})
ENDIF(EXPORT_SHARED_LIBS)


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
  target_link_libraries(slam6D dl scanlib newmat sparse)
ENDIF(UNIX)

IF(WIN32)
	target_link_libraries(slam6D scanlib newmat sparse XGetopt ${Boost_LIBRARIES})
ENDIF(WIN32)
