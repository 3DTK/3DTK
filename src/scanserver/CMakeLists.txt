# CLIENT LIBRARY

# build by source
set(CLIENT_SRCS
  clientInterface.cc sharedScan.cc cache/cacheObject.cc
  cache/cacheDataAccess.cc
)

if(WITH_METRICS)
  message(STATUS "With metrics.")
  add_definitions(-DWITH_METRICS)
  set(CLIENT_SRCS ${CLIENT_SRCS} ../slam6d/metrics.cc)
else()
  message(STATUS "Without metrics.")
endif()

add_library(scanclient_objects OBJECT ${CLIENT_SRCS})

# compile with -fPIC so that the result can be made into a shared library
set_property(TARGET scanclient_objects PROPERTY POSITION_INDEPENDENT_CODE 1)

add_library(scanclient_static STATIC $<TARGET_OBJECTS:scanclient_objects>)
add_library(scanclient_shared SHARED $<TARGET_OBJECTS:scanclient_objects>)

set_target_properties(scanclient_static PROPERTIES OUTPUT_NAME scanclient CLEAN_DIRECT_OUTPUT 1)
set_target_properties(scanclient_shared PROPERTIES OUTPUT_NAME scanclient CLEAN_DIRECT_OUTPUT 1)

# add libraries
# boost::interprocess
set(CLIENT_LIBS ${Boost_LIBRARIES} pointfilter)

if(UNIX AND NOT APPLE)
  # boost::interprocess uses pthread, requiring librt
  set(CLIENT_LIBS ${CLIENT_LIBS} rt)
endif()

target_link_libraries(scanclient_static ${CLIENT_LIBS} ${Boost_FILESYSTEM_LIBRARY})
target_link_libraries(scanclient_shared ${CLIENT_LIBS} ${Boost_FILESYSTEM_LIBRARY})

# SERVER EXECUTABLE

# build by source
set(SERVER_SRCS
  scanserver.cc serverInterface.cc frame_io.cc serverScan.cc
  cache/cacheManager.cc cache/cacheHandler.cc scanHandler.cc
  temporaryHandler.cc cacheIO.cc
)

add_executable(scanserver ${SERVER_SRCS})

# add libraries
# boost::interprocess/filesystem
# scanclient basic functionality
# scanio for ScanHandler input
if(MSVC)
  set(SERVER_LIBS ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} scanclient_static scanio_static)
else()
  set(SERVER_LIBS ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} scanclient_shared scanio_shared)
endif()

if(UNIX AND NOT APPLE)
  # boost::interprocess uses pthread, requiring librt
  set(SERVER_LIBS ${SERVER_LIBS} pthread rt)
endif()

if(WIN32)
  # 3rd party getopt library
  set(SERVER_LIBS ${SERVER_LIBS} XGetopt)
endif()

target_link_libraries(scanserver ${SERVER_LIBS})
