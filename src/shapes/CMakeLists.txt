include_directories(${NEWMAT_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/gocr-0.48/src)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/gocr-0.48/include)
include_directories(${PROJECT_SOURCE_DIR}/include/shapes/)

#  # Compile gocr library
#  set(GOCR_DIR ${PROJECT_SOURCE_DIR}/3rdparty/gocr-0.48/src/)
#  add_library(Pgm2asc SHARED  ${GOCR_DIR}gocr.c ${GOCR_DIR}pgm2asc.c ${GOCR_DIR}box.c ${GOCR_DIR}database.c 
#                              ${GOCR_DIR}detect.c ${GOCR_DIR}barcode.c ${GOCR_DIR}lines.c ${GOCR_DIR}list.c 
#                              ${GOCR_DIR}ocr0.c ${GOCR_DIR}ocr0n.c ${GOCR_DIR}ocr1.c ${GOCR_DIR}otsu.c 
#                              ${GOCR_DIR}output.c ${GOCR_DIR}pixel.c ${GOCR_DIR}unicode.c ${GOCR_DIR}remove.c 
#                              ${GOCR_DIR}pnm.c ${GOCR_DIR}pcx.c ${GOCR_DIR}progress.c ${GOCR_DIR}job.c)

#  add_executable(shapes shapes.cc  geom_math.cc numberrec.cc  scan_ransac.cc )
#
#  if(WIN32)
#    target_link_libraries(shapes scan XGetopt netpbm Pgm2asc)
#  endif()
#
#  if (UNIX)
#    target_link_libraries(shapes scan newmat dl netpbm Pgm2asc )
#  endif()
#  

add_executable(planes plane.cc)
#  add_executable(image toImage.cc image.cc hough.cc convexplane.cc accumulator.cc hsm3d.cc ConfigFileHough.cc parascan.cc quadtree.cc geom_math.cc )
#  add_executable(matchMarker matchMarker.cc)

if(UNIX)
  target_link_libraries(planes scan_shared shape_shared ${NEWMAT_LIBRARIES_SHARED} ${ANN_LIBRARIES_SHARED} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
#    target_link_libraries(image scan newmat sparse dl ${ANN_LIBRARIES} )
#    target_link_libraries(matchMarker scan newmat sparse dl ${ANN_LIBRARIES} )
endif()

if (WIN32)
  target_link_libraries(planes scan_static XGetopt shape_shared ${NEWMAT_LIBRARIES_SHARED} ${ANN_LIBRARIES_STATIC})
#    target_link_libraries(image scan newmat sparse ${ANN_LIBRARIES} XGetopt)
endif()

set(SHAPELIB_SRCS
  hough.cc convexplane.cc accumulator.cc hsm3d.cc ConfigFileHough.cc parascan.cc quadtree.cc geom_math.cc )

add_library(shape_objects OBJECT ${SHAPELIB_SRCS})

# compile with -fPIC so that the result can be made into a shared library
set_property(TARGET shape_objects PROPERTY POSITION_INDEPENDENT_CODE 1)

add_library(shape_static STATIC $<TARGET_OBJECTS:shape_objects>)
add_library(shape_shared SHARED $<TARGET_OBJECTS:shape_objects>)

set_target_properties(shape_static PROPERTIES OUTPUT_NAME shape CLEAN_DIRECT_OUTPUT 1)
set_target_properties(shape_shared PROPERTIES OUTPUT_NAME shape CLEAN_DIRECT_OUTPUT 1)

target_link_libraries(shape_shared ${Boost_LIBRARIES} ${NEWMAT_LIBRARIES_SHARED})
target_link_libraries(shape_shared ${Boost_LIBRARIES} ${NEWMAT_LIBRARIES_STATIC})
