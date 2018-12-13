if (WITH_OPENCV AND WITH_CGAL AND WITH_EIGEN3 AND WITH_GMP)
  # Module name
  set(MODULE "Floorplan")

  # Set CXX flags for CGAL
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frounding-math")

  # Sources
  set(SOURCES colorGradient.cc  floorPlan.cc  floorplan_extractor.cc plane3d.cc point3d.cc  rotation3d.cc  scene.cc  util.cc  vector3d.cc)

  # Binaries
  add_executable(floorplan ${SOURCES})

  # Find libraries
  if (NOT WIN32)
  find_library(LIB_GMP gmp REQUIRED)
  endif()
  find_package(CGAL REQUIRED)
  find_package(Boost REQUIRED)
  find_package(OpenCV REQUIRED)

  find_package(Eigen3 REQUIRED)

  target_link_libraries(floorplan scan ${ANN_LIBRARIES} ${NEWMAT_LIBRARIES} ${CGAL_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_LIBS} ${LIB_GMP} ${OpenCV_LIBS})
  if (UNIX)
    target_link_libraries(floorplan dl)
  endif()
  if (WIN32)
    target_link_libraries(floorplan XGetopt)
  endif()
endif()
