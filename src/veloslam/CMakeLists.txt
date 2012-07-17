SET(SHOW_LIBS ${OPENGL_LIBRARIES} glui scan ANN)
IF(WIN32)
 IF( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    SET(SHOW_LIBS ${SHOW_LIBS}  ${CMAKE_SOURCE_DIR}/3rdparty/windows/x64/freeglut.lib XGetopt)
  ELSE( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    SET(SHOW_LIBS ${SHOW_LIBS}  ${CMAKE_SOURCE_DIR}/3rdparty/windows/freeglut.lib XGetopt)
  ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 8 )
ENDIF(WIN32)
IF (UNIX)
  SET(SHOW_LIBS newmat dl ${GLUT_LIBRARIES} ${SHOW_LIBS})
ENDIF(UNIX)

IF(WITH_VELOSLAM)
  add_executable(veloslam veloslam.cc veloscan.cc debugview.cc pcddump.cc tracker.cc
   trackermanager.cc drawtrackers.cc kalmanfilter.cc matrix.cc lap.cc)

IF(UNIX)
  target_link_libraries(veloslam dl scan newmat sparse ANN ${Boost_LIBRARIES} ${SHOW_LIBS})
ENDIF(UNIX)

IF(WIN32)
  target_link_libraries(veloslam scan newmat sparse ANN XGetopt ${Boost_LIBRARIES} ${SHOW_LIBS})
ENDIF(WIN32)
ENDIF(WITH_VELOSLAM)

#IF(WITH_VELOSLAM)
 #  SET(SHOW_SRCS ../show/NurbsPath.cc ../show/PathGraph.cc ../show/vertexarray.cc
 #      ../show/viewcull.cc ../show/colormanager.cc ../show/compacttree.cc
 #      ../show/scancolormanager.cc ../show/display.cc)

 # add_executable(veloshow veloshow.cc veloscan.cc
 #     debugview.cc  pcddump.cc cluster_classification.cc
 #     tracker.cc  trackermanager.cc drawtrackers.cc
 #     svm.cc  clusterboundingbox.cc multiscan_random_field.cc
 #     kalmanfilter.cc matrix.cc lap.cc 
 #     intersection_detection.cc SegIter.model ${SHOW_SRCS})

#IF(UNIX)
#  target_link_libraries(veloshow dl scan newmat sparse ANN  ${SHOW_LIBS})
#ENDIF(UNIX)

#IF(WIN32)
#  target_link_libraries(veloshow scan newmat sparse ANN XGetopt ${Boost_LIBRARIES}  ${SHOW_LIBS})
#ENDIF(WIN32)

#ENDIF(WITH_VELOSLAM)



