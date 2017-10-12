if(WIN32)
  add_library(pointfilter STATIC ../slam6d/pointfilter.cc)
else()
  add_library(pointfilter SHARED ../slam6d/pointfilter.cc)
endif()

set(SCANIO_LIBNAMES
  faro_xyz_rgbr ks ks_rgb leica_xyzr ply pts ptsr pts_rgb pts_rgbr pts_rrgb riegl_rgb riegl_txt rts uos uosr uos_rgb uos_rgbr uos_rrgb uos_rrgbt velodyne xyz xyzr xyz_rgb xyz_rgba xyz_rgbr xyz_rrgb
)

if(WITH_B3D)
  set(SCANIO_LIBNAMES ${SCANIO_LIBNAMES} b3d)
endif()

if(WITH_RIVLIB)
  set(SCANIO_LIBNAMES ${SCANIO_LIBNAMES} rxp)
endif()

if(WITH_LASLIB)
  set(SCANIO_LIBNAMES ${SCANIO_LIBNAMES} laz)
  include_directories(${PROJECT_SOURCE_DIR}/3rdparty/lastools/)
endif()

if (WITH_LIBZIP)
  find_package(LibZip)
  if (${LibZip_FOUND})
    if(${LIBZIP_VERSION} VERSION_LESS 0.11.2)
      set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DLIBZIP_OLD")
    endif()
  else()
    include_directories(${PROJECT_SOURCE_DIR}/3rdparty/libzip-1.0.1/lib/)
    set (LIBZIP_LIBRARY zip)
  endif()
  if(APPLE)
    include_directories("${LIBZIP_INCLUDE_DIR_ZIPCONF}")
  endif()
  add_definitions(-DWITH_LIBZIP)
else()
  unset (LIBZIP_LIBRARY CACHE)
endif()

add_library(scan_io_helper_static STATIC helper.cc)
add_library(scan_io_helper_shared SHARED helper.cc)
target_link_libraries(scan_io_helper_static ${LIBZIP_LIBRARY} ${Boost_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} pointfilter )
target_link_libraries(scan_io_helper_shared ${LIBZIP_LIBRARY} ${Boost_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} pointfilter )
set_target_properties(scan_io_helper_static PROPERTIES OUTPUT_NAME scan_io_helper CLEAN_DIRECT_OUTPUT 1)
set_target_properties(scan_io_helper_shared PROPERTIES OUTPUT_NAME scan_io_helper CLEAN_DIRECT_OUTPUT 1)


foreach(libname ${SCANIO_LIBNAMES})
  add_library(scan_io_${libname} SHARED scan_io_${libname}.cc)
  target_link_libraries(scan_io_${libname} ${Boost_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} scanio_shared scan_io_helper_shared pointfilter )
endforeach()
target_include_directories(scan_io_ply PUBLIC ${PROJECT_SOURCE_DIR}/3rdparty/rply-1.1.4)
target_link_libraries(scan_io_ply rply)

if(WITH_RIVLIB)
  target_link_libraries(scan_io_rxp ${RiVLib_SCANLIB_LIBRARY})
endif()

if(WITH_LASLIB)
target_link_libraries(scan_io_laz laslib_shared)
endif()

add_library(scanio_static STATIC scan_io.cc ../slam6d/io_types.cc)
add_library(scanio_shared SHARED scan_io.cc ../slam6d/io_types.cc)

set_target_properties(scanio_static PROPERTIES OUTPUT_NAME scanio CLEAN_DIRECT_OUTPUT 1)
set_target_properties(scanio_shared PROPERTIES OUTPUT_NAME scanio CLEAN_DIRECT_OUTPUT 1)

target_link_libraries(scanio_static scan_io_helper_static dl ${Boost_SYSTEM_LIBRARY})
target_link_libraries(scanio_shared scan_io_helper_shared dl ${Boost_SYSTEM_LIBRARY})
