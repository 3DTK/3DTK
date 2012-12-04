add_library(normals normals.cc)
target_link_libraries(normals newmat scan ANN fbr_cv_io fbr_panorama ${OpenCV_LIBS})


IF(WITH_TOOLS)
  add_executable(calc_normals calc_normals.cc)
  target_link_libraries(calc_normals normals ${Boost_LIBRARIES})
ENDIF(WITH_TOOLS)
