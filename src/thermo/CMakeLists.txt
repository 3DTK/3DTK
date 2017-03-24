if (WITH_THERMO)

  include_directories(${CMAKE_SOURCE_DIR}/3rdparty/cvblob)
  include_directories(${CMAKE_SOURCE_DIR}/include/shapes/)
  include_directories(${CMAKE_SOURCE_DIR}/include/thermo/)
  include_directories(/usr/include/)
  include_directories(/usr/include/opencv)
  
  add_executable(caliboard caliboard.cc) 
  add_executable(thermo thermo.cc) 
  
  if(UNIX)
    target_link_libraries(caliboard scan shape newmat dl ${ANN_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
    target_link_libraries(thermo scan shape newmat dl ${ANN_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
    target_link_libraries(thermo cvblob ${OpenCV_LIBS} scan ${ANN_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
  endif()

  if (WIN32)
    target_link_libraries(caliboard scan shape newmat XGetopt ann)
    target_link_libraries(thermo scan shape newmat XGetopt ann)
    target_link_libraries(thermo cvblob ${OpenCV_LIBS} scan ann)
  endif()
  

endif()
