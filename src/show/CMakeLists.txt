IF (WITH_SHOW)

  include_directories(${OPENGL_INCLUDE_DIR})
  add_executable(show show.cc  NurbsPath.cc  PathGraph.cc vertexarray.cc  viewcull.cc colormanager.cc compacttree.cc scancolormanager.cc display.cc)

  IF(WIN32)
    target_link_libraries(show glui ${CMAKE_SOURCE_DIR}/windows/glut32.lib)
    target_link_libraries(show scanlib ANN ${OPENGL_LIBRARIES} XGetopt)
  ENDIF(WIN32)

  IF (UNIX)
    target_link_libraries(show scanlib newmat ANN ${OPENGL_LIBRARIES} glui)
    target_link_libraries(show dl ${GLUT_LIBRARIES} )
  ENDIF(UNIX)

  IF(WITH_CUDA)
    target_link_libraries(show ANN)
  ENDIF(WITH_CUDA)

ENDIF(WITH_SHOW)
  
IF(WITH_WXSHOW)

  add_executable(wxshow wxshow.cc NurbsPath.cc PathGraph.cc vertexarray.cc viewcull.cc colormanager.cc compacttree.cc scancolormanager.cc selectionframe.cc display.cc)

  target_link_libraries(wxshow ${wxWidgets_LIBRARIES} wxthings)

  IF(WIN32)
    target_link_libraries(wxshow glui ${CMAKE_SOURCE_DIR}/windows/glut32.lib)
    target_link_libraries(wxshow scanlib ANN ${OPENGL_LIBRARIES} XGetopt)
  ENDIF(WIN32)

  IF (UNIX)
    target_link_libraries(wxshow scanlib newmat ANN ${OPENGL_LIBRARIES} glui)
    target_link_libraries(wxshow dl ${GLUT_LIBRARIES} )
  ENDIF(UNIX)

  IF(WITH_CUDA)
    target_link_libraries(wxshow ANN)
  ENDIF(WITH_CUDA)

ENDIF(WITH_WXSHOW)
