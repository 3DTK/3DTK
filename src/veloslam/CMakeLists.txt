SET(SHOW_LIBS ${OPENGL_LIBRARIES} glui scanlib ANN)
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
  add_executable(veloslam veloslam.cc veloscan.cc debugview.cc pcddump.cc tracker.cc trackermanager.cc ${SHOW_SRCS})
  
IF(UNIX)
  target_link_libraries(veloslam dl scanlib newmat sparse ANN ${SHOW_LIBS})
ENDIF(UNIX)

IF(WIN32)
  target_link_libraries(veloslam scanlib newmat sparse ANN XGetopt ${Boost_LIBRARIES} ${SHOW_LIBS})
ENDIF(WIN32)

ENDIF(WITH_VELOSLAM)



