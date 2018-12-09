if (NOT MSVC AND WITH_OPENGL)
  include_directories(${CXSPARSE_INCLUDE_DIRS})
  include_directories(${NEWMAT_INCLUDE_DIRS})
  include_directories(${GLUT_INCLUDE_DIR})

  if(WITH_COMPACT_OCTREE)
    message(STATUS "Using compact octrees")
    add_definitions(-DUSE_COMPACT_TREE)
  else()
    message(STATUS "Not using compact octreees: Warning uses more memory")
  endif()

  if(WITH_GLEE)
    message(STATUS "Using opengl extensions")
    add_definitions(-DWITH_GLEE)
  else()
    message(STATUS "Not using opengl extensions")
  endif()

  if(WITH_METRICS)
    message(STATUS "With metrics.")
    add_definitions(-DWITH_METRICS)
  else()
    message(STATUS "Without metrics.")
  endif()

  add_executable(veloslam veloslam.cc veloscan.cc debugview.cc pcddump.cc tracker.cc
   trackermanager.cc drawtrackers.cc kalmanfilter.cc matrix.cc lap.cc)

  if(UNIX)
    target_link_libraries(veloslam dl scan_shared ${NEWMAT_LIBRARIES_STATIC} ${CXSPARSE_LIBRARIES_SHARED} ${ANN_LIBRARIES_SHARED} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${SHOW_LIBS_SHARED})
  endif()

  if(WIN32)
    target_link_libraries(veloslam scan_static ${CXSPARSE_LIBRARIES_STATIC} ${ANN_LIBRARIES_SHARED} XGetopt ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${SHOW_LIBS_SHARED})
  endif()
endif()

#if(WITH_VELOSLAM)
 #  set(SHOW_SRCS ../show/NurbsPath.cc ../show/PathGraph.cc ../show/vertexarray.cc
 #      ../show/viewcull.cc ../show/colormanager.cc ../show/compacttree.cc
 #      ../show/scancolormanager.cc ../show/display.cc)

 # add_executable(veloshow veloshow.cc veloscan.cc
 #     debugview.cc  pcddump.cc cluster_classification.cc
 #     tracker.cc  trackermanager.cc drawtrackers.cc
 #     svm.cc  clusterboundingbox.cc multiscan_random_field.cc
 #     kalmanfilter.cc matrix.cc lap.cc 
 #     intersection_detection.cc SegIter.model ${SHOW_SRCS})

#if(UNIX)
#  target_link_libraries(veloshow dl scan newmat sparse ${ANN_LIBRARIES}  ${SHOW_LIBS_SHARED})
#endif()

#if(WIN32)
#  target_link_libraries(veloshow scan newmat sparse ${ANN_LIBRARIES} XGetopt ${Boost_LIBRARIES}  ${SHOW_LIBS_SHARED})
#endif()

#endif()



