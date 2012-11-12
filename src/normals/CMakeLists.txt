IF(WITH_NORMALS)
  FIND_PACKAGE(OpenCV REQUIRED)

  add_library(normals normals.cc)
  target_link_libraries(normals newmat scan ANN fbr_cv_io fbr_panorama ${OpenCV_LIBS})

  add_executable(calc_normals calc_normals.cc)
  target_link_libraries(calc_normals normals ${Boost_LIBRARIES})
ENDIF(WITH_NORMALS)
