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
  
IF(WITH_GLEE)
  SET(SHOW_LIBS ${SHOW_LIBS} glee)
ENDIF(WITH_GLEE)

SET(SHOW_SRCS NurbsPath.cc  PathGraph.cc vertexarray.cc  viewcull.cc colormanager.cc compacttree.cc scancolormanager.cc display.cc)

IF (WITH_SHOW)
  add_executable(show show.cc ${SHOW_SRCS}) 
  target_link_libraries(show ${SHOW_LIBS})
ENDIF(WITH_SHOW)

IF(WITH_WXSHOW)
  add_executable(wxshow wxshow.cc selectionframe.cc ${SHOW_SRCS})
  target_link_libraries(wxshow ${wxWidgets_LIBRARIES} wxthings ${SHOW_LIBS})
ENDIF(WITH_WXSHOW)

IF(WITH_VELOSLAM)
  add_executable(veloshow ../veloslam/veloshow.cc ../veloslam/veloscan.cc ${SHOW_SRCS})
  target_link_libraries(veloshow ${SHOW_LIBS})
ENDIF(WITH_SHOW)  
  