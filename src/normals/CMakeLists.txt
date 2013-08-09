add_library(normals_panorama STATIC normals_panorama.cc)
target_link_libraries(normals_panorama newmat scan fbr_cv_io fbr_panorama ${OpenCV_LIBS})

IF(EXPORT_SHARED_LIBS)
add_library(normals_panorama_s SHARED normals_panorama.cc)
target_link_libraries(normals_panorama_s newmat_s scan_s fbr_s ${OpenCV_LIBS} ${Boost_LIBRARIES})
ENDIF(EXPORT_SHARED_LIBS)


IF(WITH_TOOLS)
  add_executable(calc_normals calc_normals.cc)
  #target_link_libraries(calc_normals normals normals_panorama ${Boost_LIBRARIES})
  target_link_libraries(calc_normals scan normals_panorama ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
ENDIF(WITH_TOOLS)
