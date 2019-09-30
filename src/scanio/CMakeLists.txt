add_library(pointfilter SHARED ../slam6d/pointfilter.cc)
add_library(range_set_parser SHARED ../parsers/range_set_parser.cc)

set(SCANIO_LIBNAMES
  faro_xyz_rgbr ks ks_rgb leica_xyzr ply pts ptsr pts_rgb pts_rgbr pts_rrgb riegl_rgb riegl_txt rts uos uosr uos_rgb uos_rgbr uos_rrgb uos_rrgbt velodyne xyz xyzr xyz_rgb xyz_rgba xyz_rgbr xyz_rrgb
  uos_normal
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
  find_package(LibZip REQUIRED)
  if(${LIBZIP_VERSION} VERSION_LESS 0.11.2)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DLIBZIP_OLD")
  endif()
  if(APPLE)
    include_directories("${LIBZIP_INCLUDE_DIR_ZIPCONF}")
  endif()
  add_definitions(-DWITH_LIBZIP)
else()
  unset (LIBZIP_LIBRARY CACHE)
endif()

add_library(scanio scan_io.cc ../slam6d/io_types.cc helper.cc)
set_property(TARGET scanio PROPERTY POSITION_INDEPENDENT_CODE 1)

set(SCANIO_LINK_LIBRARIES ${LIBZIP_LIBRARY} ${Boost_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} pointfilter range_set_parser)
if (UNIX)
  list(APPEND SCANIO_LINK_LIBRARIES dl)
endif()

target_link_libraries(scanio ${SCANIO_LINK_LIBRARIES})

foreach(libname ${SCANIO_LIBNAMES})
  add_library(scan_io_${libname} SHARED scan_io_${libname}.cc)
  target_link_libraries(scan_io_${libname} ${Boost_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} scanio pointfilter range_set_parser)
endforeach()
target_include_directories(scan_io_ply PUBLIC ${PROJECT_SOURCE_DIR}/3rdparty/rply-1.1.4)
target_link_libraries(scan_io_ply rply)

if(WITH_RIVLIB)
  target_link_libraries(scan_io_rxp ${RiVLib_SCANLIB_LIBRARY})
endif()

if(WITH_LASLIB)
target_link_libraries(scan_io_laz laslib)
endif()
