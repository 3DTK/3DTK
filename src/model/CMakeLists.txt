if (WITH_CGAL AND WITH_EIGEN3 AND WITH_OPENCV)
  include_directories(${NEWMAT_INCLUDE_DIRS})
  find_package(CGAL REQUIRED)
  find_package(Eigen3 REQUIRED)

  # Module name
  set(MODULE "Model")
  if(MSVC)
    set(UNIX_LIBS shape scan_static ${NEWMAT_LIBRARIES_SHARED} dl ${ANN_LIBRARIES_STATIC})
  else()
    set(UNIX_LIBS shape scan_shared ${NEWMAT_LIBRARIES_SHARED} dl ${ANN_LIBRARIES_SHARED})
  endif()
  set(WIN32_LIBS shape scan_shared ${NEWMAT_LIBRARIES_SHARED} XGetopt)

  # Set CXX flags for CGAL
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frounding-math")

  # Sources
  file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.cc")

  # Binaries
  add_executable(model ${SOURCES})

  if (WITH_OPENCV)
    # Find libraries
    find_library(LIB_GMP gmp)
    find_package(Boost REQUIRED)
    find_package(OpenCV REQUIRED)

    if(UNIX)
        target_link_libraries(model scan_shared ${UNIX_LIBS} ${LIB_CGAL} ${Boost_LIBS} ${LIB_GMP} ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
    endif()

    if (WIN32)
        target_link_libraries(model ${UNIX_LIBS} ${LIB_CGAL} ${Boost_LIBS} ${LIB_GMP} ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
    endif()
  endif()
endif()
