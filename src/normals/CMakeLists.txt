add_library(normals STATIC normals.cc)
add_library(normals_panorama STATIC normals_panorama.cc)
target_link_libraries(normals newmat ANN)
target_link_libraries(normals_panorama newmat scan fbr_cv_io fbr_panorama ${OpenCV_LIBS})

IF(EXPORT_SHARED_LIBS)
add_library(normals_s SHARED normals.cc)
add_library(normals_panorama_s SHARED normals_panorama.cc)
ENDIF(EXPORT_SHARED_LIBS)


IF(WITH_TOOLS)
  add_executable(calc_normals calc_normals.cc)
  target_link_libraries(calc_normals normals normals_panorama ${Boost_LIBRARIES})
ENDIF(WITH_TOOLS)
