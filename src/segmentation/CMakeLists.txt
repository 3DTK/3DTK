if(WITH_SEGMENTATION)
if(OpenCV_VERSION VERSION_GREATER 2.1.0)
  add_executable(scan2segments scan2segments.cc ../slam6d/fbr/fbr_global.cc)
  target_link_libraries(scan2segments scan ${ANN_LIBRARIES} fbr_cv_io fbr_panorama fbr_feature fbr_feature_matcher fbr_registration ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${OpenCV_LIBS})
else()
  message("OpenCV Version > 2.2 required for scan2segmentation")
endif()
  add_executable(fhsegmentation fhsegmentation.cc FHGraph.cc disjoint-set.cc segment-graph.cc)

  target_link_libraries(fhsegmentation scan ${ANN_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${OpenCV_LIBS})


endif()
