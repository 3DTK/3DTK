IF (WITH_DETECTOR)

  FIND_PACKAGE(OpenCV REQUIRED )
  add_executable(detector detector.cc )
  target_link_libraries(detector cv highgui)


ENDIF(WITH_DETECTOR)
