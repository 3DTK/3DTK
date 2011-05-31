IF(WITH_SIFT)
  ADD_DEFINITIONS("-DHAS_PANO13")

  SET(AUTOPANOCPP_DIR autopano-sift-c/APSCpp/)
  SET(AUTOPANO_DIR autopano-sift-c/)

  include_directories(${AUTOPANO_DIR})

  # dependant on libpano13-dev
  FIND_PATH(LIBPANO13_INCLUDE_DIR pano13/filter.h)

  FIND_PATH(LIBVIGRA_INCLUDE_DIR vigra/impex.hxx)

  FIND_PATH(XML20_INCLUDE_DIR
    NAMES libxml/xmlreader.h libxml/xmlwriter.h  
    PATHS 
    /usr/include/libxml2/
    /usr/local/include
    NO_DEFAULT_PATH)
  include_directories(${XML20_INCLUDE_DIR})

  add_library(libsift STATIC 
    ${AUTOPANO_DIR}APSCmain.c
    ${AUTOPANO_DIR}AreaFilter.c
    ${AUTOPANO_DIR}ArrayList.c
    ${AUTOPANO_DIR}BondBall.c
    ${AUTOPANO_DIR}DisplayImage.c
    ${AUTOPANO_DIR}GaussianConvolution.c
    ${AUTOPANO_DIR}HashTable.c
    ${AUTOPANO_DIR}ImageMap.c
    ${AUTOPANO_DIR}ImageMatchModel.c
    ${AUTOPANO_DIR}KDTree.c
    ${AUTOPANO_DIR}KeypointXML.c
    ${AUTOPANO_DIR}LoweDetector.c
    ${AUTOPANO_DIR}MatchKeys.c
    ${AUTOPANO_DIR}Random.c
    ${AUTOPANO_DIR}RANSAC.c
    ${AUTOPANO_DIR}ScaleSpace.c
    ${AUTOPANO_DIR}SimpleMatrix.c
    ${AUTOPANO_DIR}Transform.c
    ${AUTOPANO_DIR}Utils.c   )
  
  add_executable(autopano  ${AUTOPANO_DIR}AutoPano.c  ) 
  target_link_libraries(autopano libsift jpeg tiff png z pano13 xml2)

  add_executable(generatekeys  ${AUTOPANO_DIR}GenerateKeys.c  ) 
  target_link_libraries(generatekeys libsift jpeg tiff png z pano13 xml2)

  add_executable(autopano-sift-c  
    ${AUTOPANOCPP_DIR}ANNkd_wrap.cpp
    ${AUTOPANOCPP_DIR}APSCpp.c
    ${AUTOPANOCPP_DIR}APSCpp_main.c
    ${AUTOPANOCPP_DIR}CamLens.c
    ${AUTOPANOCPP_DIR}HermiteSpline.c
    ${AUTOPANOCPP_DIR}saInterp.c
    ${AUTOPANOCPP_DIR}saRemap.c ) 
  target_link_libraries(autopano-sift-c libsift jpeg tiff png z pano13 xml2 ANN stdc++)

  IF(NOT WIN32)
    FIND_PACKAGE(LibXml2 REQUIRED)
  ENDIF(NOT WIN32)

  SET(TERSCANREG_DIR library/)
  SET(SIFTPROGRAMS_DIR programs/)

  include_directories(${TERSCANREG_DIR})
  include_directories(${CMAKE_SOURCE_DIR}/include/slam6d)	
  include_directories(.)	

  SET(TERSCANREG_SRCS
    ${TERSCANREG_DIR}Coord.cpp
    ${TERSCANREG_DIR}PanoramaMap.cpp
    ${TERSCANREG_DIR}PointC.cpp
    ${TERSCANREG_DIR}PointCloud.cpp
    ${TERSCANREG_DIR}PolarPoint.cpp
    ${TERSCANREG_DIR}PolarPointCloud.cpp
    ${TERSCANREG_DIR}Reader.cpp
    ${TERSCANREG_DIR}Reader_RIEGL.cpp
    ${TERSCANREG_DIR}SuperPixel.cpp
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

  SET(ADDSIFT_LIBS vigraimpex freetype xml2 )

  IF(UNIX)
    target_link_libraries(generatesiftfeatures terscanreg libsift ${ADDSIFT_LIBS})
    target_link_libraries(panoramacreator terscanreg libsift ${ADDSIFT_LIBS})
    target_link_libraries(matchsiftfeatures terscanreg libsift ${ADDSIFT_LIBS})
    target_link_libraries(registerscans terscanreg libsift ${ADDSIFT_LIBS})
    target_link_libraries(readscan terscanreg libsift ${ADDSIFT_LIBS})
  ENDIF(UNIX)

ENDIF(WITH_SIFT)

