IF(WITH_SEGMENTATION)
IF(OpenCV_VERSION_MAJOR GREATER 1 AND OpenCV_VERSION_MINOR GREATER 2)
  add_executable(scan2segments scan2segments.cc ../slam6d/fbr/fbr_global.cc)
  target_link_libraries(scan2segments scan ANN fbr_cv_io fbr_panorama fbr_feature fbr_feature_matcher fbr_registration ${Boost_LIBRARIES} ${OpenCV_LIBS})
ELSE(OpenCV_VERSION_MAJOR GREATER 1 AND OpenCV_VERSION_MINOR GREATER 2)
  MESSAGE("OpenCV Version > 2.2 required for scan2segmentation")
ENDIF(OpenCV_VERSION_MAJOR GREATER 1 AND OpenCV_VERSION_MINOR GREATER 2)
  add_executable(fhsegmentation fhsegmentation.cc FHGraph.cc disjoint-set.cc segment-graph.cc)

  target_link_libraries(fhsegmentation scan ANN ${Boost_LIBRARIES} ${OpenCV_LIBS})


ENDIF(WITH_SEGMENTATION)
