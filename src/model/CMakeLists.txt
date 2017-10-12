if (WITH_CGAL AND WITH_EIGEN3 AND WITH_OPENCV)
  find_package(CGAL REQUIRED)
  find_package(Eigen3 REQUIRED)

  # Module name
  set(MODULE "Model")
  set(UNIX_LIBS shape scan_shared newmat dl ann)
  set(WIN32_LIBS shape scan_shared newmat XGetopt)

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
