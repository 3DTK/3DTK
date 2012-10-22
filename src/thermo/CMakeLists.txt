IF (WITH_THERMO)
  find_package(OpenCV REQUIRED)

  include_directories(${CMAKE_SOURCE_DIR}/3rdparty/cvblob)
  include_directories(${CMAKE_SOURCE_DIR}/3rdparty/gocr-0.48/src)
  include_directories(${CMAKE_SOURCE_DIR}/3rdparty/gocr-0.48/include)
  include_directories(${CMAKE_SOURCE_DIR}/include/shapes/)
  include_directories(${CMAKE_SOURCE_DIR}/include/thermo/)
  include_directories(/usr/include/)
  include_directories(/usr/include/opencv)
  
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
  
  add_executable(caliboard caliboard.cc) 
#  add_executable(thermo thermo.cc) 
#  add_executable(thermo thermo.cc src/cvaux.cpp  src/cvblob.cpp  src/cvcolor.cpp src/cvcontour.cpp  src/cvlabel.cpp  src/cvtrack.cpp)
#  add_executable(thermo thermo.cc)
  
  IF(UNIX)
    target_link_libraries(caliboard scan shape newmat dl ANN)
#    target_link_libraries(thermo scan shape newmat dl ANN)
#    target_link_libraries(thermo scan shape newmat dl ANN)
#    target_link_libraries(thermo cv cvaux cxcore GL GLU highgui cvblob scan ANN)
  ENDIF(UNIX)

  IF (WIN32)
    target_link_libraries(caliboard scan shape newmat XGetopt ANN)
#    target_link_libraries(thermo scan shape newmat XGetopt ANN)
#    target_link_libraries(thermo cv cvaux cxcore GL GLU highgui cvblob scan ANN)
  ENDIF(WIN32)
  

ENDIF(WITH_THERMO)
