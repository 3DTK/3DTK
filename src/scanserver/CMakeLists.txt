# SHARED LIBRARIES

# used for ScanIO filtering in each of the scan_io libraries and setting up in SharedScan
# distinguish between windows and linux: the latter one doesn't have problems with linking against a shared library in CMake, windows wants a stub-.lib library to link against, which relays to the .dll, which is never created by CMake
if(WIN32)
  add_library(pointfilter STATIC pointfilter.cc)
else(WIN32)
  add_library(pointfilter SHARED pointfilter.cc)
endif(WIN32)

# scan_io libraries used in the scanserver
set(SCANIO_LIBNAMES
  uos uos_rgb ks ks_rgb riegl_txt riegl_rgb rts
)
# TEMP: _ss suffix to indicate scanserver association
foreach(libname ${SCANIO_LIBNAMES})
  add_library(scan_io_${libname}_ss SHARED scan_io/scan_io_${libname}.cc)
  target_link_libraries(scan_io_${libname}_ss pointfilter)
endforeach(libname)

if(WITH_RIVLIB)
  add_library(scan_io_rxp_ss SHARED scan_io/scan_io_rxp.cc)
  target_link_libraries(scan_io_rxp_ss pointfilter ${RIVLIB})
endif(WITH_RIVLIB)

# CLIENT LIBRARY

# build by source
set(CLIENT_SRCS
  clientInterface.cc io_types.cc sharedScan.cc cache/cacheObject.cc
  cache/cacheDataAccess.cc
)

if(WITH_METRICS)
  set(CLIENT_SRCS ${CLIENT_SRCS} metrics.cc)
endif(WITH_METRICS)

add_library(scanclient STATIC ${CLIENT_SRCS})

# add libraries
# boost::interprocess
set(CLIENT_LIBS ${Boost_LIBRARIES} pointfilter)

if(UNIX)
  # boost::interprocess uses pthread, requiring librt
  set(CLIENT_LIBS ${CLIENT_LIBS} rt)
endif(UNIX)

target_link_libraries(scanclient ${CLIENT_LIBS})

# SERVER EXECUTABLE

# build by source
set(SERVER_SRCS
  scanserver.cc serverInterface.cc scan_io/scan_io.cc frame_io.cc serverScan.cc
  cache/cacheManager.cc cache/cacheHandler.cc scanHandler.cc
  temporaryHandler.cc cacheIO.cc
)

add_executable(scanserver ${SERVER_SRCS})

# add libraries
# boost::interprocess/filesystem
# scanclient basic functionality
# pointfilter in SharedScan and ScanHandler
set(SERVER_LIBS ${Boost_LIBRARIES} scanclient)

if(UNIX)
  # boost::interprocess uses pthread, requiring librt
  # opening shared libraries requires libdl
  set(SERVER_LIBS ${SERVER_LIBS} rt dl)
endif(UNIX)

if(WIN32)
  # 3rd party getopt library
  set(SERVER_LIBS ${SERVER_LIBS} XGetopt)
endif(WIN32)

target_link_libraries(scanserver ${SERVER_LIBS})
