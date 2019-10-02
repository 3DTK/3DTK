add_executable(test_line_endings line_endings.cc)
target_link_libraries(test_line_endings ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_REGEX_LIBRARY})
add_test(test_line_endings_build "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target test_line_endings)

add_executable(test_trailing_whitespace trailing_whitespace.cc)
target_link_libraries(test_trailing_whitespace ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_REGEX_LIBRARY})
add_test(test_trailing_whitespace_build "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target test_trailing_whitespace)

add_test(test_line_endings_run ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test_line_endings "${PROJECT_SOURCE_DIR}/src" "${PROJECT_SOURCE_DIR}/include")
set_tests_properties(test_line_endings_run PROPERTIES DEPENDS "test_line_endings_build")

add_test(test_trailing_whitespace_run ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test_trailing_whitespace "${PROJECT_SOURCE_DIR}/src" "${PROJECT_SOURCE_DIR}/include")
set_tests_properties(test_trailing_whitespace_run PROPERTIES DEPENDS "test_trailing_whitespace_build")
