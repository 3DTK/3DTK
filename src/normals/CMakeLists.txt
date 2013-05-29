add_library(normals STATIC normals.cc)
target_link_libraries(normals newmat scan ANN fbr_cv_io fbr_panorama ${OpenCV_LIBS})

IF(EXPORT_SHARED_LIBS)
add_library(normals_s SHARED normals.cc)
ENDIF(EXPORT_SHARED_LIBS)


IF(WITH_TOOLS)
  add_executable(calc_normals calc_normals.cc)
  target_link_libraries(calc_normals normals ${Boost_LIBRARIES})
ENDIF(WITH_TOOLS)
