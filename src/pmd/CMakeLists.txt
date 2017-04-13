if(WITH_PMD)
  include_directories(${PROJECT_SOURCE_DIR}/include/pmd/pmdaccess2)
  include_directories(${PROJECT_SOURCE_DIR}/include/pmd/pose)
  include_directories(${PROJECT_SOURCE_DIR}/include/pmd)
  

  find_library(LIBCONFIG_LIBRARY_DIR libconfig.so)
  find_library(LIBGLFW_LIBRARY_DIR libglfw.so)

  if(NOT WIN32)
    find_package(LibXml2 REQUIRED)
  endif()

  find_package(XMLRPC REQUIRED c++)
  find_package(OpenCV REQUIRED )

  find_path(FTGL_INCLUDE_DIR
    NAMES ftgl.h
    PATHS /usr/local/include
    /usr/include
    /usr/include/FTGL
    /usr/include/freetype2
    NO_DEFAULT_PATH)
  
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

  add_library(pmdaccess2 STATIC ${PMDACCESS_DIR}pmdaccess.cc)
  
  set(PMD_LIBS pmdaccess2 config glfw GL GLU ftgl xmlrpc_util glfw xmlrpc_client xmlrpc xmlrpc++ xmlrpc_client++ xmlrpc_xmltok xmlrpc_xmlparse ${OpenCV_LIBS} )


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
