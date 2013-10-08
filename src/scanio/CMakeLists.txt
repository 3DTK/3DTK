if(WIN32)
  add_library(pointfilter STATIC ../slam6d/pointfilter.cc)
else(WIN32)
  add_library(pointfilter SHARED ../slam6d/pointfilter.cc)
endif(WIN32)

set(SCANIO_LIBNAMES
  uos uosr uos_rgb uos_rrgb uos_rrgbt xyz xyzr xyz_rgb xyz_rrgb ply ks ks_rgb riegl_txt riegl_rgb rts velodyne laz
)

if(WITH_RIVLIB)
  set(SCANIO_LIBNAMES ${SCANIO_LIBNAMES} rxp)
  if(LIBXML2_FOUND)
    include_directories(${LIBXML2_INCLUDE_DIR})
#    set(SCANIO_LIBNAMES ${SCANIO_LIBNAMES} riegl_project)
#    target_link_libraries(scan_io_riegl_project ${RIVLIB} scan_io_rxp ${LIBXML2_LIBRARIES})
  endif(LIBXML2_FOUND)
endif(WITH_RIVLIB)
    

foreach(libname ${SCANIO_LIBNAMES})
if(WIN32)
  add_library(scan_io_${libname} SHARED scan_io_${libname}.cc)
else(WIN32)
  add_library(scan_io_${libname} SHARED scan_io_${libname}.cc)
endif(WIN32)  
  target_link_libraries(scan_io_${libname} pointfilter ${Boost_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
endforeach(libname)

if(WITH_RIVLIB)
  target_link_libraries(scan_io_rxp ${RIVLIB})
  if(LIBXML2_FOUND)
    target_link_libraries(scan_io_rxp ${LIBXML2_LIBRARIES}) #scan_io_riegl_project ${RIVLIB})
  endif(LIBXML2_FOUND)
endif(WITH_RIVLIB)

target_link_libraries(scan_io_laz LAZ)

if(WIN32)
  add_library(scanio STATIC scan_io.cc ../slam6d/io_types.cc)
else(WIN32)
  add_library(scanio SHARED scan_io.cc ../slam6d/io_types.cc)
endif(WIN32) 

if(UNIX)
  target_link_libraries(scanio dl)
endif(UNIX)


