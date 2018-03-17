add_executable(test_model_rotation3d test_rotation3d.cc ../../src/model/rotation3d.cc)
target_link_libraries(test_model_rotation3d ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(test_model_point3d test_point3d.cc ../../src/model/point3d.cc ../../src/model/point3d.cc ../../src/model/rotation3d.cc ../../src/model/vector3d.cc ../../src/model/plane3d.cc)
target_link_libraries(test_model_point3d ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_test(test_model_rotation3d_run ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test_model_rotation3d)
add_test(test_model_rotation3d_build "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target test_model_rotation3d)
set_tests_properties(test_model_rotation3d_run PROPERTIES DEPENDS test_model_rotation3d_build)

add_test(test_model_point3d_run ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test_model_point3d)
add_test(test_model_point3d_build "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target test_model_point3d)
set_tests_properties(test_model_point3d_run PROPERTIES DEPENDS test_model_point3d_build)
