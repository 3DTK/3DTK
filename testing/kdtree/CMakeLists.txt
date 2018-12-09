add_executable(test_kdtree_indexed kdtree_indexed.cc)
target_link_libraries(test_kdtree_indexed scan ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(test_kdtree_indexed_random kdtree_indexed_random.cc)
target_link_libraries(test_kdtree_indexed_random scan ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(test_kdtree kdtree.cc)
target_link_libraries(test_kdtree scan ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY} ${Boost_SYSTEM_LIBRARY})

add_test(test_kdtree_indexed_run ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test_kdtree_indexed)
add_test(test_kdtree_indexed_build "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target test_kdtree_indexed)
set_tests_properties(test_kdtree_indexed_run PROPERTIES DEPENDS test_kdtree_indexed_build)

add_test(test_kdtree_indexed_random_run ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test_kdtree_indexed_random)
add_test(test_kdtree_indexed_random_build "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target test_kdtree_indexed_random)
set_tests_properties(test_kdtree_indexed_random_run PROPERTIES DEPENDS test_kdtree_indexed_random_build)

add_test(test_kdtree_run ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test_kdtree)
add_test(test_kdtree_build "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target test_kdtree)
set_tests_properties(test_kdtree_run PROPERTIES DEPENDS test_kdtree_build)
