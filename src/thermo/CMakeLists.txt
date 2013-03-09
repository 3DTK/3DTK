IF (WITH_THERMO)

  include_directories(${CMAKE_SOURCE_DIR}/3rdparty/cvblob)
  include_directories(${CMAKE_SOURCE_DIR}/include/shapes/)
  include_directories(${CMAKE_SOURCE_DIR}/include/thermo/)
  include_directories(/usr/include/)
  include_directories(/usr/include/opencv)
  
  add_executable(caliboard caliboard.cc) 
  add_executable(thermo thermo.cc) 
  
  IF(UNIX)
    target_link_libraries(caliboard scan shape newmat dl ANN)
    target_link_libraries(thermo scan shape newmat dl ANN)
    target_link_libraries(thermo cvblob ${OpenCV_LIBS} scan ANN)
  ENDIF(UNIX)

  IF (WIN32)
    target_link_libraries(caliboard scan shape newmat XGetopt ANN)
    target_link_libraries(thermo scan shape newmat XGetopt ANN)
    target_link_libraries(thermo cvblob ${OpenCV_LIBS} scan ANN)
  ENDIF(WIN32)
  

ENDIF(WITH_THERMO)
