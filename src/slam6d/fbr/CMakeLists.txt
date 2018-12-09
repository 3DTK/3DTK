if (NOT MSVC AND WITH_OPENCV)
  find_package(OpenCV REQUIRED)

  set(FBR_LIBS scan_shared show_shared ${OpenCV_LIBS})

  if(WITH_OPENCV_NONFREE)
    message(STATUS "Using non-free OpenCV functionality")
    add_definitions(-DWITH_OPENCV_NONFREE)
  else()
    message(STATUS "Not using non-free OpenCV functionality")
  endif()

  ### Export Static Libs
  set(FBR_IO_SRC fbr_global.cc scan_cv.cc)
  add_library(fbr_cv_io_shared SHARED ${FBR_IO_SRC})
  target_link_libraries(fbr_cv_io_shared ${FBR_LIBS})
  set_target_properties(fbr_cv_io_shared PROPERTIES OUTPUT_NAME fbr_cv_io CLEAN_DIRECT_OUTPUT 1)

  set(FBR_PANORAMA_SRC panorama.cc projection.cc)
  add_library(fbr_panorama_shared SHARED ${FBR_PANORAMA_SRC} fbr_global.cc)
  target_link_libraries(fbr_panorama_shared ${FBR_LIBS})
  set_target_properties(fbr_panorama_shared PROPERTIES OUTPUT_NAME fbr_panorama CLEAN_DIRECT_OUTPUT 1)

  if(OpenCV_VERSION VERSION_GREATER 2.1.0)

    set(FBR_FEATURE_SRC feature.cc)
    add_library(fbr_feature_shared SHARED ${FBR_FEATURE_SRC})
    target_link_libraries(fbr_feature_shared fbr_cv_io_shared ${FBR_LIBS})
    set_target_properties(fbr_feature_shared PROPERTIES OUTPUT_NAME fbr_feature CLEAN_DIRECT_OUTPUT 1)

    set(FBR_FEATURE_MATCHER_SRC feature_matcher.cc feature.cc)
    add_library(fbr_feature_matcher_shared SHARED ${FBR_FEATURE_MATCHER_SRC})
    target_link_libraries(fbr_feature_matcher_shared fbr_cv_io_shared ${FBR_LIBS})
    set_target_properties(fbr_feature_matcher_shared PROPERTIES OUTPUT_NAME fbr_feature_matcher CLEAN_DIRECT_OUTPUT 1)

    set(FBR_REGISTRATION_SRC registration.cc)
    add_library(fbr_registration_shared SHARED ${FBR_REGISTRATION_SRC})
    target_link_libraries(fbr_registration_shared fbr_cv_io_shared ${FBR_LIBS})
    set_target_properties(fbr_registration_shared PROPERTIES OUTPUT_NAME fbr_registration CLEAN_DIRECT_OUTPUT 1)

    set(FBR_GEOREFRENCING_SRC geoRefrencing.cc)
    add_library(fbr_geoRefrencing_shared SHARED ${FBR_GEOREFRENCING_SRC} fbr_global.cc)
    target_link_libraries(fbr_geoRefrencing_shared ${FBR_LIBS})
    set_target_properties(fbr_geoRefrencing_shared PROPERTIES OUTPUT_NAME fbr_geoRefrencing CLEAN_DIRECT_OUTPUT 1)


    add_library(fbr_objects OBJECT scan_cv.cc panorama.cc projection.cc feature.cc feature_matcher.cc registration.cc fbr_global.cc feature_drawer.cc geoRefrencing.cc fbr_global.cc)
    # compile with -fPIC so that the result can be made into a shared library
    set_property(TARGET fbr_objects PROPERTY POSITION_INDEPENDENT_CODE 1)

    add_library(fbr_static STATIC $<TARGET_OBJECTS:fbr_objects>)
    add_library(fbr_shared SHARED $<TARGET_OBJECTS:fbr_objects>)

    set_target_properties(fbr_static PROPERTIES OUTPUT_NAME fbr CLEAN_DIRECT_OUTPUT 1)
    set_target_properties(fbr_shared PROPERTIES OUTPUT_NAME fbr CLEAN_DIRECT_OUTPUT 1)

    target_link_libraries(fbr_static ${FBR_LIBS})
    if(MSVC)
      target_link_libraries(fbr_shared scan_static ${ANN_LIBRARIES_STATIC} ${OpenCV_LIBS} show_shared)
    else()
      target_link_libraries(fbr_shared scan_shared ${ANN_LIBRARIES} ${OpenCV_LIBS} show_shared)
    endif()


    ###feature based registration
    add_executable(featurebasedregistration feature_based_registration.cc fbr_global.cc)
    #target_link_libraries(featurebasedregistration fbr_cv_io fbr_panorama fbr_feature fbr_feature_matcher fbr_registration ${FBR_LIBS})

    if (WIN32)
	    target_link_libraries(featurebasedregistration fbr_shared ${NEWMAT_LIBRARIES_SHARED} XGetopt show_shared ${SHOW_LIBS_SHARED} ${OPENGL_LIBRARIES})
    endif()

    if(UNIX)
	    target_link_libraries(featurebasedregistration fbr_shared ${FBR_LIBS} ${NEWMAT_LIBRARIES_SHARED} show_shared ${SHOW_LIBS_SHARED} ${OPENGL_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
    endif()


    ###panorama matcher
    add_executable(panorama_matcher panorama_matcher.cc fbr_global.cc)

    if(UNIX)
      target_link_libraries(panorama_matcher fbr_shared ${Boost_SYSTEM_LIBRARY} ${SHOW_LIBS_SHARED})
    endif()

    if (WIN32)
      target_link_libraries(panorama_matcher fbr_shared ${Boost_SYSTEM_LIBRARY} ${SHOW_LIBS_SHARED} XGetopt)
    endif()


    ###scan to panorama
    add_executable(scan_to_panorama scan_to_panorama.cc fbr_global.cc)

    if(UNIX)
	    target_link_libraries(scan_to_panorama fbr_shared ${FBR_LIBS} ${NEWMAT_LIBRARIES_SHARED} show_shared ${SHOW_LIBS_SHARED} ${OPENGL_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
    endif()

    if (WIN32)
	    target_link_libraries(scan_to_panorama fbr_shared ${NEWMAT_LIBRARIES_SHARED} show_shared ${SHOW_LIBS_SHARED} ${OPENGL_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} XGetopt)
    endif()

    ###click tool
    add_executable(click_tool click_tool.cc fbr_global.cc)
    if(UNIX)
	    target_link_libraries(click_tool fbr_shared ${NEWMAT_LIBRARIES_SHARED} ${FBR_LIBS} show_shared ${SHOW_LIBS_SHARED} ${OPENGL_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
    endif()

    if (WIN32)
	    target_link_libraries(click_tool fbr_shared ${NEWMAT_LIBRARIES_SHARED} ${FBR_LIBS} show_shared ${SHOW_LIBS_SHARED} ${OPENGL_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} XGetopt) 
    endif()

    ###panorama reader
    add_executable(panorama_to_scan panorama_to_scan.cc fbr_global.cc)

    if(UNIX)
	    target_link_libraries(panorama_to_scan fbr_shared ${NEWMAT_LIBRARIES_SHARED} show_shared ${SHOW_LIBS_SHARED} ${OPENGL_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
    endif()

    if (WIN32)
	    target_link_libraries(panorama_to_scan fbr_shared ${NEWMAT_LIBRARIES_SHARED} show_shared ${SHOW_LIBS_SHARED} ${OPENGL_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} XGetopt)
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

