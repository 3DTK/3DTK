if(WITH_FBR)
  set(FBR_LIBS scan ${OpenCV_LIBS})

  if(WITH_OPENCV_NONFREE)
    message(STATUS "Using non-free OpenCV functionality")
    add_definitions(-DWITH_OPENCV_NONFREE)
  else()
    message(STATUS "Not using non-free OpenCV functionality")
  endif()

  ### Export Static Libs
  set(FBR_IO_SRC scan_cv.cc)
  add_library(fbr_cv_io STATIC ${FBR_IO_SRC})
  target_link_libraries(fbr_cv_io ${FBR_LIBS})

  set(FBR_PANORAMA_SRC panorama.cc projection.cc)
  add_library(fbr_panorama STATIC ${FBR_PANORAMA_SRC} fbr_global.cc)
  target_link_libraries(fbr_panorama ${FBR_LIBS})

  if(OpenCV_VERSION VERSION_GREATER 2.1.0)

    set(FBR_FEATURE_SRC feature.cc)
    add_library(fbr_feature STATIC ${FBR_FEATURE_SRC})
    target_link_libraries(fbr_feature ${FBR_LIBS})

    set(FBR_FEATURE_MATCHER_SRC feature_matcher.cc)
    add_library(fbr_feature_matcher STATIC ${FBR_FEATURE_MATCHER_SRC})
    target_link_libraries(fbr_feature_matcher ${FBR_LIBS})

    set(FBR_REGISTRATION_SRC registration.cc)
    add_library(fbr_registration STATIC ${FBR_REGISTRATION_SRC})
    target_link_libraries(fbr_registration ${FBR_LIBS})

    set(FBR_GEOREFRENCING_SRC geoRefrencing.cc)
    add_library(fbr_geoRefrencing STATIC ${FBR_GEOREFRENCING_SRC} fbr_global.cc)
    target_link_libraries(fbr_geoRefrencing ${FBR_LIBS})


    set(FBR_SRC scan_cv.cc panorama.cc projection.cc feature.cc feature_matcher.cc registration.cc fbr_global.cc feature_drawer.cc geoRefrencing.cc)
    add_library(fbr STATIC ${FBR_SRC} fbr_global.cc)
    target_link_libraries(fbr ${FBR_LIBS})



    ### EXPORT SHARED LIBS
    if(EXPORT_SHARED_LIBS)
      add_library(fbr_s SHARED ${FBR_SRC} fbr_global.cc)
      if (WIN32)
      target_link_libraries(fbr_s scan_s ANN_s ${OpenCV_LIBS} show_s)
      else()
      target_link_libraries(fbr_s scan_s ann ${OpenCV_LIBS} show_s)
      endif()
    endif()



    ###feature based registration
    add_executable(featurebasedregistration feature_based_registration.cc fbr_global.cc)
    #target_link_libraries(featurebasedregistration fbr_cv_io fbr_panorama fbr_feature fbr_feature_matcher fbr_registration ${FBR_LIBS})

    if(WITH_GLEE)
      set(SHOW_LIBS ${SHOW_LIBS} glee)
    endif()

    if (WIN32)
      target_link_libraries(featurebasedregistration fbr newmat XGetopt showstatic ${SHOW_LIBS} ${OPENGL_LIBRARIES})
    endif()

    if(UNIX)
      target_link_libraries(featurebasedregistration fbr newmat showstatic ${SHOW_LIBS} ${OPENGL_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
    endif()


    ###panorama matcher
    add_executable(panorama_matcher panorama_matcher.cc fbr_global.cc)

    if(UNIX)
      target_link_libraries(panorama_matcher fbr ${Boost_SYSTEM_LIBRARY})
    endif()

    if (WIN32)
      target_link_libraries(panorama_matcher fbr ${Boost_SYSTEM_LIBRARY} XGetopt)
    endif()


    ###scan to panorama
    add_executable(scan_to_panorama scan_to_panorama.cc fbr_global.cc)

    if(UNIX)
      target_link_libraries(scan_to_panorama fbr newmat showstatic ${SHOW_LIBS} ${OPENGL_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
    endif()

    if (WIN32)
      target_link_libraries(scan_to_panorama fbr newmat showstatic ${SHOW_LIBS} ${OPENGL_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} XGetopt)
    endif()


    ###panorama reader
    add_executable(panorama_to_scan panorama_to_scan.cc fbr_global.cc)

    if(UNIX)
      target_link_libraries(panorama_to_scan fbr newmat showstatic ${SHOW_LIBS} ${OPENGL_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
    endif()

    if (WIN32)
      target_link_libraries(panorama_to_scan fbr newmat showstatic ${SHOW_LIBS} ${OPENGL_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} XGetopt)
    endif()


  else()
    message("OpenCV Version > 2.2 required for FBR")
  endif()
endif()


####TEST
#add_executable(fbr-icp fbr-icp.cc fbr_global.cc)

#if(UNIX)
#target_link_libraries(fbr-icp fbr newmat showstatic ${SHOW_LIBS} ${OPENGL_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
#target_link_libraries(fbr-icp fbr newmat sparse showstatic ${SHOW_LIBS} ${OPENGL_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
#endif()

