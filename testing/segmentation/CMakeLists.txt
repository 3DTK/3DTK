add_executable(test_segmentation_disjoint-set disjoint-set.cc ../../src/segmentation/disjoint-set.cc)
target_link_libraries(test_segmentation_disjoint-set ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(test_segmentation_segment-graph segment-graph.cc ../../src/segmentation/segment-graph.cc ../../src/segmentation/disjoint-set.cc)
target_link_libraries(test_segmentation_segment-graph ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_test(test_segmentation_disjoint-set_run ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test_segmentation_disjoint-set)
add_test(test_segmentation_disjoint-set_build "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target test_segmentation_disjoint-set)
set_tests_properties(test_segmentation_disjoint-set_run PROPERTIES DEPENDS test_segmentation_disjoint-set_build)

add_test(test_segmentation_segment-graph_run ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test_segmentation_segment-graph)
add_test(test_segmentation_segment-graph_build "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target test_segmentation_segment-graph)
set_tests_properties(test_segmentation_segment-graph_run PROPERTIES DEPENDS test_segmentation_segment-graph_build)
