IF(WITH_SIFT)
  ADD_DEFINITIONS("-DHAS_PANO13")

  SET(AUTOPANOCPP_DIR autopano-sift-c/APSCpp/)
  SET(AUTOPANO_DIR autopano-sift-c/)
  include_directories(${AUTOPANO_DIR})
  
  # dependant on libpano13-dev
  FIND_PATH(LIBPANO13_INCLUDE_DIR pano13/filter.h)
  
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
ENDIF(WITH_SIFT)
