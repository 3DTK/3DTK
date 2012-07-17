FIND_PACKAGE(OpenCV REQUIRED)

SET(FBR_IO_SRC scan_cv.cc)
add_library(fbr_cv_io STATIC ${FBR_IO_SRC})

SET(FBR_PANORAMA_SRC panorama.cc)
add_library(fbr_panorama STATIC ${FBR_PANORAMA_SRC})

SET(FBR_FEATURE_SRC feature.cc)
add_library(fbr_feature STATIC ${FBR_FEATURE_SRC})

SET(FBR_FEATURE_MATCHER_SRC feature_matcher.cc)
add_library(fbr_feature_matcher STATIC ${FBR_FEATURE_MATCHER_SRC})

SET(FBR_REGISTRATION_SRC registration.cc)
add_library(fbr_registration STATIC ${FBR_REGISTRATION_SRC})

IF(WITH_FBR)
SET(FBR_LIBS scan ANN ${OpenCV_LIBS})

add_executable(featurebasedregistration feature_based_registration.cc fbr_global.cc)
target_link_libraries(featurebasedregistration fbr_cv_io fbr_panorama fbr_feature fbr_feature_matcher fbr_registration ${FBR_LIBS})
ENDIF(WITH_FBR)