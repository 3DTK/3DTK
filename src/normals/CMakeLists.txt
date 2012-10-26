IF(WITH_NORMALS)
  add_executable(normals normals.cc)

  FIND_PACKAGE(OpenCV REQUIRED)

  target_link_libraries(normals newmat scan ANN fbr_cv_io fbr_panorama ${Boost_LIBRARIES} ${OpenCV_LIBS})
ENDIF(WITH_NORMALS)
