if(WIN32)
  add_library(pointfilter STATIC ../slam6d/pointfilter.cc)
else()
  add_library(pointfilter SHARED ../slam6d/pointfilter.cc)
endif()

set(SCANIO_LIBNAMES
  uos uosr uos_rgb uos_rrgb uos_rrgbt xyz xyzr leica_xyzr xyz_rgb xyz_rgba xyz_rrgb faro_xyz_rgbr ply ks ks_rgb riegl_txt riegl_rgb rts velodyne b3d
)

if(WITH_RIVLIB)
  set(SCANIO_LIBNAMES ${SCANIO_LIBNAMES} rxp)
endif()

if(WITH_LASLIB)
  set(SCANIO_LIBNAMES ${SCANIO_LIBNAMES} laz)
  include_directories(${CMAKE_SOURCE_DIR}/3rdparty/lastools/)
endif()

if(WIN32)
  add_library(scan_io_helper STATIC helper.cc)
else()
  add_library(scan_io_helper SHARED helper.cc)
endif()
target_link_libraries(scan_io_helper ${LIBZIP_LIBRARY} ${Boost_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} pointfilter )

foreach(libname ${SCANIO_LIBNAMES})
if(WIN32)
  add_library(scan_io_${libname} SHARED scan_io_${libname}.cc)
else()
  add_library(scan_io_${libname} SHARED scan_io_${libname}.cc)
endif()  
  target_link_libraries(scan_io_${libname} ${Boost_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} scan_io_helper pointfilter )
endforeach()
target_link_libraries(scan_io_b3d b3dfile)
target_link_libraries(scan_io_ply rply)

if(WITH_RIVLIB)
  target_link_libraries(scan_io_rxp ${RiVLib_SCANLIB_LIBRARY})
endif()

if(WITH_LASLIB)
target_link_libraries(scan_io_laz LAZ)
endif()

if(WIN32)
  add_library(scanio STATIC scan_io.cc ../slam6d/io_types.cc)
else()
  add_library(scanio SHARED scan_io.cc ../slam6d/io_types.cc)
endif() 

if(UNIX)
  target_link_libraries(scanio dl ${Boost_SYSTEM_LIBRARY})
endif()
