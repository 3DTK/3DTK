if(WITH_METRICS)
  message(STATUS "With metrics in slam6d.")
  add_definitions(-DWITH_METRICS)
else()
  message(STATUS "Without metrics in slam6d.")
endif()

### TOOLS

if(WITH_TOOLS)
  add_executable(b3dto3d b3dto3d.cc)
  IF(UNIX)
    target_link_libraries(b3dto3d b3dfile scan dl ann ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} showstatic ${OPENGL_LIBRARIES} ${SHOW_LIBS} newmat)
  ENDIF(UNIX)
  IF(WIN32)
    target_link_libraries(b3dto3d b3dfile scan ann ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} showstatic ${OPENGL_LIBRARIES} ${SHOW_LIBS} newmat)
  ENDIF(WIN32)
if(WITH_FBR)
  ### SCAN_RED
  add_executable(scan_red scan_red.cc ../scanio/writer.cc fbr/fbr_global.cc fbr/panorama.cc fbr/scan_cv.cc)

  if(WITH_GLEE)
    # TODO can we remove OpenGL extensions from the scan reduction program?
    set(SHOW_LIBS ${SHOW_LIBS} glee)
  endif()
  
  if(UNIX)
    target_link_libraries(scan_red scan dl ann fbr_cv_io fbr_panorama ${OpenCV_LIBS} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} showstatic ${OPENGL_LIBRARIES} ${SHOW_LIBS} newmat)
  endif()

    if(UNIX)
      target_link_libraries(scan_red scan dl ann fbr_cv_io fbr_panorama ${OpenCV_LIBS} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} showstatic ${OPENGL_LIBRARIES} ${SHOW_LIBS} newmat)
    endif()

    if(APPLE)
      target_link_libraries(scan_red scan dl ann fbr_cv_io fbr_panorama ${OpenCV_LIBS} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} showstatic ${OPENGL_LIBRARIES} ${SHOW_LIBS} newmat)
    endif()

    if (WIN32)
      target_link_libraries(scan_red scan ann fbr_cv_io fbr_panorama ${OpenCV_LIBS} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} showstatic XGetopt newmat)
    endif()
  endif()

  ### SCAN_DIFF
  add_executable(scan_diff scan_diff.cc)
  #  add_executable(scan_diff2d scan_diff2d.cc ../show/colormanager.cc)

  if(UNIX)
    target_link_libraries(scan_diff scan dl ann newmat ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
    #    target_link_libraries(scan_diff2d scan dl ann)
  endif()

  if(APPLE)
    target_link_libraries(scan_diff scan dl ann newmat ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
    #    target_link_libraries(scan_diff2d scan dl ann)
  endif()

  if (WIN32)
    target_link_libraries(scan_diff scan ann XGetopt newmat)
    # target_link_libraries(scan_diff2d scan dl ann XGetopt)
  endif()

  add_executable(bps2txt bps2txt.cc)
  target_link_libraries(bps2txt b3dfile)

  add_executable(frame_to_graph frame_to_graph.cc)
  add_executable(convergence convergence.cc)
  add_executable(graph_balancer graph_balancer.cc)
  add_executable(exportPoints exportPoints.cc ../scanio/writer.cc)
  add_executable(frames2riegl frames2riegl.cc)
  add_executable(frames2pose frames2pose.cc)
  add_executable(framesdiff2frames framesdiff2frames.cc)
  add_executable(pose2frames pose2frames.cc)
  add_executable(riegl2frames riegl2frames.cc)
  add_executable(transformFrames transformFrames.cc)
  #  add_executable(vigo23dtk vigo23dtk.cc)
  #  add_executable(g23dtk g23dtk.cc)
  add_executable(toGlobal toGlobal.cc)
  add_executable(average6DoFposes average6DoFposes.cc)

  if(UNIX)
    target_link_libraries(graph_balancer scan ${Boost_GRAPH_LIBRARY} ${Boost_SERIALIZATION_LIBRARY} ${Boost_REGEX_LIBRARY} ${Boost_SYSTEM_LIBRARY})
    target_link_libraries(exportPoints scan dl ann newmat ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
    target_link_libraries(transformFrames scan dl ann newmat ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
    target_link_libraries(toGlobal scan)
    target_link_libraries(convergence ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
    target_link_libraries(pose2frames ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
  endif()

  if(APPLE)
    target_link_libraries(graph_balancer scan ${Boost_GRAPH_LIBRARY} ${Boost_SERIALIZATION_LIBRARY} ${Boost_REGEX_LIBRARY} ${Boost_SYSTEM_LIBRARY})
    target_link_libraries(exportPoints scan dl ann newmat ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
    target_link_libraries(transformFrames scan dl ann newmat ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
    target_link_libraries(toGlobal scan)
    target_link_libraries(convergence ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
    target_link_libraries(pose2frames ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
  endif()

  if (WIN32)
    target_link_libraries(frame_to_graph XGetopt ${Boost_LIBRARIES})
    target_link_libraries(convergence XGetopt ${Boost_LIBRARIES})
    target_link_libraries(graph_balancer scan XGetopt ${Boost_GRAPH_LIBRARY} ${Boost_SERIALIZATION_LIBRARY} ${Boost_REGEX_LIBRARY} ${Boost_LIBRARIES})
    target_link_libraries(exportPoints scan ann XGetopt  ${Boost_LIBRARIES} newmat)
    target_link_libraries(transformFrames scan ann XGetopt  ${Boost_LIBRARIES} newmat)
    target_link_libraries(frames2pose XGetopt ${Boost_LIBRARIES})
    target_link_libraries(framesdiff2frames XGetopt ${Boost_LIBRARIES})
    target_link_libraries(pose2frames XGetopt ${Boost_LIBRARIES})
    target_link_libraries(frames2riegl XGetopt ${Boost_LIBRARIES})
    target_link_libraries(riegl2frames XGetopt ${Boost_LIBRARIES})
    target_link_libraries(toGlobal scan XGetopt ${Boost_LIBRARIES})
  endif()

  target_link_libraries(average6DoFposes newmat)

endif()

### SCANLIB

set(SCANLIB_SRCS
        kd.cc             kdManaged.cc      kdMeta.cc         graphSlam6D.cc
        graph.cc          icp6D.cc          icp6Dapx.cc       icp6Dsvd.cc
        icp6Dortho.cc     icp6Dquat.cc      icp6Dhelix.cc     icp6Dlumeuler.cc
        icp6Dlumquat.cc   icp6Ddual.cc      lum6Deuler.cc     lum6Dquat.cc
        ghelix6DQ2.cc     gapx6D.cc         ann_kd.cc         elch6D.cc
        elch6Dquat.cc     elch6DunitQuat.cc elch6Dslerp.cc    elch6Deuler.cc
        point_type.cc	icp6Dquatscale.cc searchTree.cc     Boctree.cc
        scan.cc           basicScan.cc      managedScan.cc    metaScan.cc
        io_types.cc       io_utils.cc       pointfilter.cc    allocator.cc
        icp6Dnapx.cc      normals.cc        kdIndexed.cc
        )

add_library(scan STATIC ${SCANLIB_SRCS})

target_link_libraries(scan scanclient scan_io_helper scanio ann newmat cxsparse)

if(UNIX)
  target_link_libraries(scan dl pthread)
endif()

if(APPLE)
  target_link_libraries(scan dl pthread)
endif()

### EXPORT SHARED LIBS

if(EXPORT_SHARED_LIBS OR WITH_PYTHON)
  add_library(scan_s SHARED ${SCANLIB_SRCS})
  if (WIN32)
    target_link_libraries(scan_s scan_io_helper scanio scanclient_s newmat_s cxsparse_s ann_s)
  elseif(APPLE)
    target_link_libraries(scan_s scan_io_helper scanio scanclient_s newmat_s cxsparse_s ann_s)
  else()
    target_link_libraries(scan_s scan_io_helper scanio scanclient_s newmat cxsparse ann)
  endif()
endif()

### SLAM6D

add_executable(slam6D slam6D.cc)

if(UNIX)
  target_link_libraries(slam6D scan newmat cxsparse ann ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
endif()

if(APPLE)
  target_link_libraries(slam6D scan newmat cxsparse ann ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
endif()

if(WIN32)
  target_link_libraries(slam6D scan newmat cxsparse ann XGetopt ${Boost_LIBRARIES})
endif()

#if(MSVC)
#	install(TARGETS slam6D RUNTIME DESTINATION ${CMAKE_SOURCE_DIR}/windows)
#endif()
