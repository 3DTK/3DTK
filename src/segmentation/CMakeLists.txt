
if (WITH_OPENCV AND WITH_OPENMP)
  find_package(OpenCV REQUIRED)

  include_directories(${ANN_INCLUDE_DIRS})

  if(OpenCV_VERSION VERSION_GREATER 2.1.0)
    add_executable(scan2segments scan2segments.cc ../slam6d/fbr/fbr_global.cc)
    target_link_libraries(scan2segments scan_shared ${ANN_LIBRARIES_SHARED} fbr_cv_io_shared fbr_panorama_shared fbr_feature_shared fbr_feature_matcher_shared fbr_registration_shared ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${OpenCV_LIBS} ${SHOW_LIBS_SHARED})
  else()
    message("OpenCV Version > 2.2 required for scan2segmentation")
  endif()
  add_executable(fhsegmentation fhsegmentation.cc FHGraph.cc disjoint-set.cc segment-graph.cc)
  target_link_libraries(fhsegmentation scan_shared ${ANN_LIBRARIES_SHARED} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${OpenCV_LIBS})
  
  add_executable(graph_cut_segmentation graph_cut/graph_cut.cc graph_cut/util.cc graph_cut/blob_color.cc)
  target_link_libraries(graph_cut_segmentation scan fbr_panorama_shared fbr_cv_io_shared newmat ${OpenCV_LIBS} ${FBR_LIBS} ${SHOW_LIBS_SHARED} Boost::program_options)
  option(GCS_DEBUG "graph cut segmentation debug output")
  if(NOT GCS_DEBUG)
    target_compile_definitions(graph_cut_segmentation PUBLIC NDEBUG)
  endif()
endif()
