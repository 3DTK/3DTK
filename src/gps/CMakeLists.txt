
include_directories(${CMAKE_SOURCE_DIR}/3rdparty/minmea/)
include_directories(${CMAKE_SOURCE_DIR}/include/gps/)
include_directories(/usr/include/)
include_directories(/usr/include/opencv)

add_executable(scan2utm ../scanio/writer.cc gpstools.cc scan2utm.cc)

if (WIN32)
  target_link_libraries(scan2utm scan_static ${ANN_LIBRARIES_STATIC} ${OpenCV_LIBS})
else()
  target_link_libraries(scan2utm scan_shared ${Boost_LIBRARIES} ${ANN_LIBRARIES_SHARED} ${OpenCV_LIBS})
endif()
