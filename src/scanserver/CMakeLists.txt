# CLIENT LIBRARY

# build by source
set(CLIENT_SRCS
  clientInterface.cc sharedScan.cc cache/cacheObject.cc
  cache/cacheDataAccess.cc
)

if(WITH_METRICS)
  set(CLIENT_SRCS ${CLIENT_SRCS} ../slam6d/metrics.cc)
endif(WITH_METRICS)

add_library(scanclient STATIC ${CLIENT_SRCS})
### EXPORT SHARED LIBS

IF(EXPORT_SHARED_LIBS)
add_library(scanclient_s SHARED ${CLIENT_SRCS})
target_link_libraries(scanclient_s pointfilter ${Boost_LIBRARIES})
ENDIF(EXPORT_SHARED_LIBS)

# add libraries
# boost::interprocess
set(CLIENT_LIBS ${Boost_LIBRARIES} pointfilter)

if(UNIX AND NOT APPLE)
  # boost::interprocess uses pthread, requiring librt                                                                                                                                                           
  set(CLIENT_LIBS ${CLIENT_LIBS} rt)
endif(UNIX AND NOT APPLE)

target_link_libraries(scanclient ${CLIENT_LIBS})

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
set(SERVER_LIBS ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} scanclient scanio)

if(UNIX AND NOT APPLE)
  # boost::interprocess uses pthread, requiring librt
  set(SERVER_LIBS ${SERVER_LIBS} rt)
endif(UNIX AND NOT APPLE)

if(WIN32)
  # 3rd party getopt library
  set(SERVER_LIBS ${SERVER_LIBS} XGetopt)
endif(WIN32)

target_link_libraries(scanserver ${SERVER_LIBS})
