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
    if (NOT WIN32)
      find_library(LIB_GMP gmp)
    endif()
    find_package(CGAL REQUIRED COMPONENTS Core)
    find_package(Boost REQUIRED)
    find_package(OpenCV REQUIRED)

    target_link_libraries(model CGAL::CGAL CGAL::CGAL_Core scan shape ${NEWMAT_LIBRARIES} ${ANN_LIBRARIES} ${Boost_LIBS} ${LIB_GMP} ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
    if (UNIX)
      target_link_libraries(model dl)
    endif()
    if (WIN32)
      target_link_libraries(model XGetopt)
    endif()
  endif()
endif()
