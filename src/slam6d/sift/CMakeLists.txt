IF(WITH_SIFT)

  ADD_DEFINITIONS("-DHAS_PANO13")
  
  IF(NOT WIN32)
    FIND_PACKAGE(LibXml2 REQUIRED)
  ENDIF(NOT WIN32)

  FIND_PACKAGE( OpenCV REQUIRED )  

  SET(TERSCANREG_DIR library/)
  SET(SIFTPROGRAMS_DIR programs/)

  include_directories(${TERSCANREG_DIR})
  include_directories(${CMAKE_SOURCE_DIR}/include/)	
  include_directories(${CMAKE_SOURCE_DIR}/3rdparty/)	

  # dependant on libpano13-dev
  FIND_PATH(LIBPANO13_INCLUDE_DIR pano13/filter.h)

  SET(TERSCANREG_SRCS
    ${TERSCANREG_DIR}Coord.cc
    ${TERSCANREG_DIR}PanoramaMap.cc
    ${TERSCANREG_DIR}PointC.cc
    ${TERSCANREG_DIR}PointCloud.cc
    ${TERSCANREG_DIR}PolarPointCloud.cc
    ${TERSCANREG_DIR}Reader_RIEGL.cc
    ${TERSCANREG_DIR}Feature.cc
    ${TERSCANREG_DIR}FeatureBase.cc
    ${TERSCANREG_DIR}FeatureSet.cc
    ${TERSCANREG_DIR}FeatureMatch.cc
    ${TERSCANREG_DIR}FeatureMatchSet.cc
    ${TERSCANREG_DIR}FeatureMatchSetGroup.cc
    ${TERSCANREG_DIR}Register.cc
    ${TERSCANREG_DIR}ScanTransform.cc
    ../icp6Dquat.cc
  )

  add_library(terscanreg STATIC ${TERSCANREG_SRCS})

  add_executable(generatesiftfeatures ${SIFTPROGRAMS_DIR}generatesiftfeatures.cc)
  add_executable(panoramacreator ${SIFTPROGRAMS_DIR}panoramacreator.cc)
  add_executable(matchsiftfeatures ${SIFTPROGRAMS_DIR}matchsiftfeatures.cc)
  add_executable(registerscans ${SIFTPROGRAMS_DIR}registerscans.cc)
  add_executable(readscan ${SIFTPROGRAMS_DIR}readscan.cc)
  add_executable(opencv-panorama ${SIFTPROGRAMS_DIR}opencv-panorama.cpp)

  SET(ADDSIFT_LIBS vigraimpex xml2 ${OpenCV_LIBS})

  IF(UNIX)
    target_link_libraries(generatesiftfeatures terscanreg libsift ${ADDSIFT_LIBS})
    target_link_libraries(panoramacreator terscanreg libsift ${ADDSIFT_LIBS})
    target_link_libraries(matchsiftfeatures terscanreg libsift ${ADDSIFT_LIBS})
    target_link_libraries(registerscans terscanreg libsift ${ADDSIFT_LIBS})
    target_link_libraries(readscan terscanreg libsift ${ADDSIFT_LIBS})
    target_link_libraries(opencv-panorama scan_io_rxp terscanreg libsift ${ADDSIFT_LIBS})
  ENDIF(UNIX)

ENDIF(WITH_SIFT)

