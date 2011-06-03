IF(WITH_SIFT)

  ADD_DEFINITIONS("-DHAS_PANO13")
  
  IF(NOT WIN32)
    FIND_PACKAGE(LibXml2 REQUIRED)
  ENDIF(NOT WIN32)

  SET(TERSCANREG_DIR library/)
  SET(SIFTPROGRAMS_DIR programs/)

  include_directories(${TERSCANREG_DIR})
  include_directories(${CMAKE_SOURCE_DIR}/include/)	
  include_directories(${CMAKE_SOURCE_DIR}/3rdparty/)	

  # dependant on libpano13-dev
  FIND_PATH(LIBPANO13_INCLUDE_DIR pano13/filter.h)

  SET(TERSCANREG_SRCS
    ${TERSCANREG_DIR}Coord.cpp
    ${TERSCANREG_DIR}PanoramaMap.cpp
    ${TERSCANREG_DIR}PointC.cpp
    ${TERSCANREG_DIR}PointCloud.cpp
    ${TERSCANREG_DIR}PolarPointCloud.cpp
    ${TERSCANREG_DIR}Reader_RIEGL.cpp
    ${TERSCANREG_DIR}Feature.cpp
    ${TERSCANREG_DIR}FeatureBase.cpp
    ${TERSCANREG_DIR}FeatureSet.cpp
    ${TERSCANREG_DIR}FeatureMatch.cpp
    ${TERSCANREG_DIR}FeatureMatchSet.cpp
    ${TERSCANREG_DIR}FeatureMatchSetGroup.cpp
    ${TERSCANREG_DIR}Register.cpp
    ${TERSCANREG_DIR}ScanTransform.cpp
    ../icp6Dquat.cc
  )

  add_library(terscanreg STATIC ${TERSCANREG_SRCS})

  add_executable(generatesiftfeatures ${SIFTPROGRAMS_DIR}generatesiftfeatures.cpp)
  add_executable(panoramacreator ${SIFTPROGRAMS_DIR}panoramacreator.cpp)
  add_executable(matchsiftfeatures ${SIFTPROGRAMS_DIR}matchsiftfeatures.cpp)
  add_executable(registerscans ${SIFTPROGRAMS_DIR}registerscans.cpp)
  add_executable(readscan ${SIFTPROGRAMS_DIR}readscan.cpp)

  SET(ADDSIFT_LIBS vigraimpex xml2 )

  IF(UNIX)
    target_link_libraries(generatesiftfeatures terscanreg libsift ${ADDSIFT_LIBS})
    target_link_libraries(panoramacreator terscanreg libsift ${ADDSIFT_LIBS})
    target_link_libraries(matchsiftfeatures terscanreg libsift ${ADDSIFT_LIBS})
    target_link_libraries(registerscans terscanreg libsift ${ADDSIFT_LIBS})
    target_link_libraries(readscan terscanreg libsift ${ADDSIFT_LIBS})
  ENDIF(UNIX)

ENDIF(WITH_SIFT)

