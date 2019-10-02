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

target_link_libraries(planes scan shape ${NEWMAT_LIBRARIES} ${ANN_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
if (WIN32)
  target_link_libraries(planes XGetopt)
endif()

set(SHAPELIB_SRCS
  hough.cc convexplane.cc accumulator.cc hsm3d.cc ConfigFileHough.cc parascan.cc quadtree.cc geom_math.cc )

add_library(shape ${SHAPELIB_SRCS})
target_link_libraries(shape ${Boost_LIBRARIES} ${NEWMAT_LIBRARIES})
