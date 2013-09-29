IF(WITH_FBR)
SET(FBR_IO_SRC scan_cv.cc)
add_library(fbr_cv_io STATIC ${FBR_IO_SRC})

SET(FBR_PANORAMA_SRC panorama.cc)
add_library(fbr_panorama STATIC ${FBR_PANORAMA_SRC} fbr_global.cc)

IF(OpenCV_VERSION_MAJOR GREATER 1 AND OpenCV_VERSION_MINOR GREATER 2)
SET(FBR_FEATURE_SRC feature.cc)
add_library(fbr_feature STATIC ${FBR_FEATURE_SRC})

SET(FBR_FEATURE_MATCHER_SRC feature_matcher.cc)
add_library(fbr_feature_matcher STATIC ${FBR_FEATURE_MATCHER_SRC})

SET(FBR_REGISTRATION_SRC registration.cc)
add_library(fbr_registration STATIC ${FBR_REGISTRATION_SRC})

SET(FBR_SRC scan_cv.cc panorama.cc feature.cc feature_matcher.cc registration.cc fbr_global.cc feature_drawer.cc)
add_library(fbr STATIC ${FBR_SRC})

SET(FBR_LIBS scan ANN ${OpenCV_LIBS})

add_executable(featurebasedregistration feature_based_registration.cc fbr_global.cc)
#target_link_libraries(featurebasedregistration fbr_cv_io fbr_panorama fbr_feature fbr_feature_matcher fbr_registration ${FBR_LIBS})

IF(WITH_GLEE)
  SET(SHOW_LIBS ${SHOW_LIBS} glee)
ENDIF(WITH_GLEE)

IF (WIN32)
  target_link_libraries(featurebasedregistration fbr newmat ${FBR_LIBS} XGetopt showstatic ${SHOW_LIBS} ${OPENGL_LIBRARIES})
ENDIF(WIN32)

IF(UNIX)
  target_link_libraries(featurebasedregistration fbr newmat ${FBR_LIBS} showstatic ${SHOW_LIBS} ${OPENGL_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
ENDIF(UNIX)

add_executable(panorama_matcher panorama_matcher.cc fbr_global.cc)

  IF(UNIX)
   target_link_libraries(panorama_matcher fbr ${FBR_LIBS} ${Boost_SYSTEM_LIBRARY})
  ENDIF(UNIX)

  IF (WIN32)
   target_link_libraries(panorama_matcher fbr ${FBR_LIBS} ${Boost_SYSTEM_LIBRARY} XGetOpt) 
  ENDIF(WIN32)

add_executable(panorama_creator panorama_creator.cc fbr_global.cc)

  IF(UNIX)
  target_link_libraries(panorama_creator fbr newmat ${FBR_LIBS} showstatic ${SHOW_LIBS} ${OPENGL_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
  ENDIF(UNIX)

  IF (WIN32)
  target_link_libraries(panorama_creator fbr newmat ${FBR_LIBS} showstatic ${SHOW_LIBS} ${OPENGL_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} XGetOpt) 
  ENDIF(WIN32)

### EXPORT SHARED LIBS

IF(EXPORT_SHARED_LIBS)
add_library(fbr_s SHARED ${FBR_SRC})
target_link_libraries(fbr_s scan_s ANN_s ${OpenCV_LIBS} show_s)
ENDIF(EXPORT_SHARED_LIBS)

ELSE(OpenCV_VERSION_MAJOR GREATER 1 AND OpenCV_VERSION_MINOR GREATER 2)
  MESSAGE("OpenCV Version > 2.2 required for FBR")
ENDIF(OpenCV_VERSION_MAJOR GREATER 1 AND OpenCV_VERSION_MINOR GREATER 2)
ENDIF(WITH_FBR)
