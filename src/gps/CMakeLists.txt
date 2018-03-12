
include_directories(${CMAKE_SOURCE_DIR}/3rdparty/minmea/)
include_directories(${CMAKE_SOURCE_DIR}/include/gps/)
include_directories(/usr/include/)
include_directories(/usr/include/opencv)

add_executable(scan2utm ../scanio/writer.cc gpstools.cc scan2utm.cc)
target_link_libraries(scan2utm scan_shared ${ANN_LIBRARIES_SHARED})    
