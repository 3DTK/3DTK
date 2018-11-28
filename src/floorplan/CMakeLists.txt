if (WITH_OPENCV AND WITH_CGAL AND WITH_EIGEN3)
  # Module name
  set(MODULE "Floorplan")

  # Set CXX flags for CGAL
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frounding-math")

  # Sources
  set(SOURCES colorGradient.cc  floorPlan.cc  floorplan_extractor.cc plane3d.cc point3d.cc  rotation3d.cc  scene.cc  util.cc  vector3d.cc)

  # Binaries
  add_executable(floorplan ${SOURCES})

  # Find libraries
  find_library(LIB_CGAL CGAL REQUIRED)
  find_library(LIB_GMP gmp REQUIRED)
  find_package(Boost REQUIRED)
  find_package(OpenCV REQUIRED)

  find_package(Eigen3 REQUIRED)

  target_link_libraries(floorplan scan dl ${ANN_LIBRARIES} ${NEWMAT_LIBRARIES} ${LIB_CGAL} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_LIBS} ${LIB_GMP} ${OpenCV_LIBS})
endif()
