IF (WITH_SHOW)

  add_executable(show show.cc  NurbsPath.cc  PathGraph.cc vertexarray.cc  viewcull.cc colormanager.cc )

  IF(WIN32)
    target_link_libraries(show glui ${CMAKE_SOURCE_DIR}/windows/glut32.lib)
    target_link_libraries(show scanlib ${OPENGL_LIBRARIES} XGetopt)
  ENDIF(WIN32)

  IF (UNIX)
    target_link_libraries(show scanlib newmat GL GLU glui)
    target_link_libraries(show dl glut)
  ENDIF(UNIX)

  IF(WITH_CUDA)
    target_link_libraries(show ANN)
  ENDIF(WITH_CUDA)

ENDIF(WITH_SHOW)
