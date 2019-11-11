if (WITH_GLFW AND WITH_OPENCV AND WITH_XMLRPC AND WITH_LIBCONFIG AND WITH_FTGL  AND OpenCV_VERSION VERSION_LESS 4)
  find_package(glfw3 REQUIRED)

  include_directories(${GLFW_INCLUDE_DIRS})

  include_directories(${PROJECT_SOURCE_DIR}/include/pmd/pmdaccess2)
  include_directories(${PROJECT_SOURCE_DIR}/include/pmd/pose)
  include_directories(${PROJECT_SOURCE_DIR}/include/pmd)

  if(APPLE)
    find_library(LIBCONFIG_LIBRARY_DIR libconfig.dylib)
  elseif(WIN32)
    find_library(LIBCONFIG_LIBRARY_DIR libconfig.lib)
  else()
    find_library(LIBCONFIG_LIBRARY_DIR libconfig.so)
  endif()
  if (NOT LIBCONFIG_LIBRARY_DIR)
    message(FATAL_ERROR "libconfig not found")
  endif()

  if(NOT WIN32)
    find_package(LibXml2 REQUIRED)
  endif()

  # the QUIET option has to be added to work around this bug:
  # https://gitlab.kitware.com/cmake/cmake/issues/16912
  find_package(XMLRPC QUIET REQUIRED COMPONENTS c++)

  find_package(OpenCV REQUIRED )

  find_path(FTGL_INCLUDE_DIR
    NAMES ftgl.h
    PATHS /usr/local/include
    /usr/include
    /usr/include/FTGL
    /usr/include/freetype2
    NO_DEFAULT_PATH)

  # freetype is already required by opencv, so we don't specifically check for it anymore
  find_path(freetype_INCLUDE_DIRS
    NAMES freetype/config/ftheader.h
    PATHS /usr/local/include
    /usr/include/freetype2
    NO_DEFAULT_PATH)

  #message(STATUS ${FTGL_INCLUDE_DIR})
  #message(STATUS ${freetype_INCLUDE_DIRS})
  include_directories(${FTGL_INCLUDE_DIR})
  include_directories(${freetype_INCLUDE_DIRS})


  set(PMDACCESS_DIR   pmdaccess2/)
  set(POSE_DIR        pose/)
  set(OFFLINE_DIR     offline/)
  set(CALIBRATE_DIR   calibrate/)

  add_library(pmdaccess2 ${PMDACCESS_DIR}pmdaccess.cc)

  set(PMD_LIBS pmdaccess2 config GL GLU ftgl xmlrpc_util glfw xmlrpc_client xmlrpc xmlrpc++ xmlrpc_client++ xmlrpc_xmltok xmlrpc_xmlparse ${OpenCV_LIBS} )


  add_executable(grabVideoAnd3D  ${OFFLINE_DIR}grabVideoAnd3D.cc cvpmd.cc pmdWrap.cc )
  target_link_libraries(grabVideoAnd3D scan dl newmat ${PMD_LIBS})

  add_executable(convertToSLAM6D  ${OFFLINE_DIR}convertToSLAM6D.cc cvpmd.cc pmdWrap.cc )
  target_link_libraries(convertToSLAM6D scan dl newmat ${PMD_LIBS})

  add_executable(calibratePMD  ${CALIBRATE_DIR}calibrate.cc )
  target_link_libraries(calibratePMD ${PMD_LIBS})

  add_executable(grabFramesCam  ${CALIBRATE_DIR}grabFramesCam.cc )
  target_link_libraries(grabFramesCam ${PMD_LIBS})

  add_executable(grabFramesPMD  ${CALIBRATE_DIR}grabFramesPMD.cc cvpmd.cc pmdWrap.cc)
  target_link_libraries(grabFramesPMD scan newmat ${PMD_LIBS})

  add_executable(extrinsicPMD  ${CALIBRATE_DIR}extrinsic.cc )
  target_link_libraries(extrinsicPMD ${PMD_LIBS})

  add_executable(pose  ${POSE_DIR}pose.cc ${POSE_DIR}history.cc cvpmd.cc pmdWrap.cc)
  target_link_libraries(pose scan newmat ${PMD_LIBS})
endif()
