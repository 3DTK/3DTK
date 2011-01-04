IF (WITH_SHOW)

  include_directories(${OPENGL_INCLUDE_DIR})
  add_executable(show show.cc  NurbsPath.cc  PathGraph.cc vertexarray.cc  viewcull.cc colormanager.cc compacttree.cc scancolormanager.cc)

  IF(WIN32)
    target_link_libraries(show glui ${CMAKE_SOURCE_DIR}/windows/glut32.lib)
    target_link_libraries(show scanlib ${OPENGL_LIBRARIES} XGetopt)
  ENDIF(WIN32)

  IF (UNIX)
    target_link_libraries(show scanlib newmat ${OPENGL_LIBRARIES} glui)
    target_link_libraries(show dl ${GLUT_LIBRARIES} )
  ENDIF(UNIX)

  IF(WITH_CUDA)
    target_link_libraries(show ANN)
  ENDIF(WITH_CUDA)

ENDIF(WITH_SHOW)
