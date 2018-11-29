if (WITH_CGAL AND WITH_EIGEN3 AND WITH_OPENCV AND WITH_GMP)
  include_directories(${NEWMAT_INCLUDE_DIRS})
  find_package(CGAL REQUIRED)
  find_package(Eigen3 REQUIRED)

  # Module name
  set(MODULE "Model")

  # Set CXX flags for CGAL
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frounding-math")

  # Sources
  file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.cc")

  # Binaries
  add_executable(model ${SOURCES})

  if (WITH_OPENCV)
    # Find libraries
    find_library(LIB_GMP gmp)
    find_package(CGAL REQUIRED)
    find_package(Boost REQUIRED)
    find_package(OpenCV REQUIRED)

    target_link_libraries(model scan shape ${NEWMAT_LIBRARIES} dl ${ANN_LIBRARIES} ${CGAL_LIBRARY} ${Boost_LIBS} ${LIB_GMP} ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
  endif()
endif()
