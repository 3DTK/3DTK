IF (WITH_SHAPE_DETECTION)
  include_directories(${CMAKE_SOURCE_DIR}/3rdparty/gocr-0.48/src)
  include_directories(${CMAKE_SOURCE_DIR}/3rdparty/gocr-0.48/include)
  include_directories(${CMAKE_SOURCE_DIR}/include/shapes/)
  
#  # Compile gocr library
#  SET(GOCR_DIR ${CMAKE_SOURCE_DIR}/3rdparty/gocr-0.48/src/)
#  add_library(Pgm2asc SHARED  ${GOCR_DIR}gocr.c ${GOCR_DIR}pgm2asc.c ${GOCR_DIR}box.c ${GOCR_DIR}database.c 
#                              ${GOCR_DIR}detect.c ${GOCR_DIR}barcode.c ${GOCR_DIR}lines.c ${GOCR_DIR}list.c 
#                              ${GOCR_DIR}ocr0.c ${GOCR_DIR}ocr0n.c ${GOCR_DIR}ocr1.c ${GOCR_DIR}otsu.c 
#                              ${GOCR_DIR}output.c ${GOCR_DIR}pixel.c ${GOCR_DIR}unicode.c ${GOCR_DIR}remove.c 
#                              ${GOCR_DIR}pnm.c ${GOCR_DIR}pcx.c ${GOCR_DIR}progress.c ${GOCR_DIR}job.c)

#  add_executable(shapes shapes.cc  geom_math.cc numberrec.cc  scan_ransac.cc )
#
#  IF(WIN32)
#    target_link_libraries(shapes scan XGetopt netpbm Pgm2asc)
#  ENDIF(WIN32)
#
#  IF (UNIX)
#    target_link_libraries(shapes scan newmat dl netpbm Pgm2asc )
#  ENDIF(UNIX)
#  
  
  add_executable(planes plane.cc)
#  add_executable(image toImage.cc image.cc hough.cc convexplane.cc accumulator.cc hsm3d.cc ConfigFileHough.cc parascan.cc quadtree.cc geom_math.cc )
#  add_executable(matchMarker matchMarker.cc)
  
  IF(UNIX)
    target_link_libraries(planes scan shape newmat dl ANN)
#    target_link_libraries(image scan newmat sparse dl ANN )
#    target_link_libraries(matchMarker scan newmat sparse dl ANN )
  ENDIF(UNIX)

  IF (WIN32)
	  target_link_libraries(planes scan newmat XGetopt shape ANN)
#    target_link_libraries(image scan newmat sparse ANN XGetopt)
  ENDIF(WIN32)

SET(SHAPELIB_SRCS
  hough.cc convexplane.cc accumulator.cc hsm3d.cc ConfigFileHough.cc parascan.cc quadtree.cc geom_math.cc )

add_library(shape STATIC ${SHAPELIB_SRCS})

#target_link_libraries(shapelib)
IF(EXPORT_SHARED_LIBS)
add_library(shape_s SHARED ${SHAPELIB_SRCS})
target_link_libraries(shape_s ${Boost_LIBRARIES} newmat)
#target_link_libraries(scan_s ${Boost_LIBRARIES} newmat)
ENDIF(EXPORT_SHARED_LIBS)

ENDIF(WITH_SHAPE_DETECTION)
