if (WITH_OPENCV AND WITH_OPENMP)
  find_package(OpenCV REQUIRED)

  include_directories(${ANN_INCLUDE_DIRS})

  if(OpenCV_VERSION VERSION_GREATER 2.1.0)
    add_executable(scan2segments scan2segments.cc ../slam6d/fbr/fbr_global.cc)
    target_link_libraries(scan2segments scan ${ANN_LIBRARIES} fbr_cv_io fbr_panorama fbr_feature fbr_feature_matcher fbr_registration ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${OpenCV_LIBS} ${SHOW_LIBS})
  else()
    message("OpenCV Version > 2.2 required for scan2segmentation")
  endif()
  add_executable(fhsegmentation fhsegmentation.cc FHGraph.cc disjoint-set.cc segment-graph.cc)
  target_link_libraries(fhsegmentation scan ${ANN_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${OpenCV_LIBS})


  add_executable(graph_cut_segmentation graph_cut/graph_cut.cc graph_cut/util.cc graph_cut/blob_color.cc)
  target_include_directories(graph_cut_segmentation PUBLIC ${NEWMAT_INCLUDE_DIRS})
  target_link_libraries(graph_cut_segmentation scan fbr_panorama fbr_cv_io newmat ${ANN_LIBRARIES} ${OpenCV_LIBS} ${FBR_LIBS} ${SHOW_LIBS} ${Boost_LIBRARIES})
  if (CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 6.3)
	  set_target_properties (graph_cut_segmentation PROPERTIES COMPILE_FLAGS "-std=gnu++11 -fext-numeric-literals")
  endif()
endif()
