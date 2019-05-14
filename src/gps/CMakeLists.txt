# need opencv because include/scanio/writer.h includes slam6d/fbr/scan_cv.h
if (WITH_OPENCV)
include_directories(${CMAKE_SOURCE_DIR}/3rdparty/minmea/)
include_directories(${CMAKE_SOURCE_DIR}/include/gps/)
include_directories(/usr/include/)
include_directories(/usr/include/opencv)

add_executable(scan2utm ../scanio/writer.cc gpstools.cc scan2utm.cc)
target_include_directories(scan2utm PUBLIC ${PROJECT_SOURCE_DIR}/3rdparty/rply-1.1.4)

add_executable(rtk2pose ../scanio/writer.cc gpstools.cc rtk2pose.cc)
target_include_directories(rtk2pose PUBLIC ${PROJECT_SOURCE_DIR}/3rdparty/rply-1.1.4)

target_link_libraries(scan2utm scan ${Boost_LIBRARIES} ${ANN_LIBRARIES} ${OpenCV_LIBS} rply)
target_link_libraries(rtk2pose scan ${Boost_LIBRARIES} ${ANN_LIBRARIES_SHARED} ${OpenCV_LIBS} rply)
if (WIN32)
  target_link_libraries(rtk2pose XGetopt)
endif()
endif()
