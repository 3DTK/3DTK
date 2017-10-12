if (WITH_OPENCV)
  find_package(OpenCV REQUIRED )

  add_executable(david_scanner david_scanner.cc )
  target_link_libraries(david_scanner  ${OpenCV_LIBS} )
endif()
