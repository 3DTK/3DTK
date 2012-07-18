if(WIN32)
  add_library(pointfilter STATIC ../slam6d/pointfilter.cc)
else(WIN32)
  add_library(pointfilter SHARED ../slam6d/pointfilter.cc)
endif(WIN32)

set(SCANIO_LIBNAMES
  uos uos_rgb ks ks_rgb riegl_txt riegl_rgb rts velodyne
)

if(WITH_RIVLIB)
  set(SCANIO_LIBNAMES ${SCANIO_LIBNAMES} rxp)
  if(LIBXML2_FOUND)
    include_directories(${LIBXML2_INCLUDE_DIR})
#    set(SCANIO_LIBNAMES ${SCANIO_LIBNAMES} riegl_project)
#    target_link_libraries(scan_io_riegl_project ${RIVLIB} scan_io_rxp ${LIBXML2_LIBRARIES})
  endif(LIBXML2_FOUND)
endif(WITH_RIVLIB)
    
#IF (WITH_CAD)
#  IF(NOT WIN32)
#    add_library(scan_io_cad             SHARED scan_io_cad.cc)
#    target_link_libraries(scan_io_cad ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
#  ENDIF(NOT WIN32)
#ENDIF (WITH_CAD)

foreach(libname ${SCANIO_LIBNAMES})
if(WIN32)
  #add_library(scan_io_${libname} STATIC scan_io_${libname}.cc)
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

if(WIN32)
  add_library(scanio STATIC scan_io.cc ../slam6d/io_types.cc)
else(WIN32)
  add_library(scanio SHARED scan_io.cc ../slam6d/io_types.cc)
endif(WIN32) 

if(UNIX)
  target_link_libraries(scanio dl)
endif(UNIX)


