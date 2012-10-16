IF(WITH_SEGMENTATION)
  add_executable(scan2segments scan2segments.cc)
  add_executable(fhsegmentation fhsegmentation.cc FHGraph.cc disjoint-set.cc segment-graph.cc)

  FIND_PACKAGE(OpenCV REQUIRED)

  target_link_libraries(scan2segments scan ANN fbr_cv_io fbr_panorama fbr_feature fbr_feature_matcher fbr_registration ${Boost_LIBRARIES} ${OpenCV_LIBS})
  target_link_libraries(fhsegmentation scan ANN ${Boost_LIBRARIES} ${OpenCV_LIBS})


ENDIF(WITH_SEGMENTATION)
