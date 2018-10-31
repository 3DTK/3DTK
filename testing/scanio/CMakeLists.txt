add_executable(test_scanio_helper helper.cc)
if(MSVC)
  target_link_libraries(test_scanio_helper scan_static scanio_shared ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
else()
  target_link_libraries(test_scanio_helper scan_shared scanio_shared ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
endif()

if (WITH_LIBZIP)
  add_executable(test_scanio_zipreader zipreader.cc)
  if(MSVC)
    target_link_libraries(test_scanio_zipreader scan_static scanio_shared ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
  else()
    target_link_libraries(test_scanio_zipreader scan_shared scanio_shared ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
  endif()
endif()

add_executable(test_scanio_readscans readscans.cc)
if(MSVC)
  target_link_libraries(test_scanio_readscans scan_static scanio_shared ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
else()
  target_link_libraries(test_scanio_readscans scan_shared scanio_shared ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
endif()

# The only way to add a dependency from a test to target building the binary
# required for the test is by formulating the binary compilation as yet another
# test and then adding a dependency between the two. See:
# http://stackoverflow.com/questions/733475/
# http://public.kitware.com/Bug/view.php?id=8774
# https://gitlab.kitware.com/cmake/cmake/issues/8774
#
# Disadvantages:
#    - possibly many more tests than "real tests", cluttering the output
#    - tests depending on each other have to be defined in the same CMakeLists.txt
#    - compilation output is hidden from the user
#
# Advantages:
#    - "make test" will always build and run the up-to-date test

add_test(test_icosphere "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target test_icosphere)

# The --target option only allows one target to be specified, so we need one
# test per scanio library
add_test(test_libscan_io_uos_build "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target scan_io_uos)
add_test(test_libscan_io_xyz_build "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target scan_io_xyz)
add_test(test_libscan_io_uos_rgb_build "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target scan_io_uos_rgb)

add_test(test_scanio_helper_run ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test_scanio_helper)
add_test(test_scanio_helper_build "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target test_scanio_helper)
set_tests_properties(test_scanio_helper_run PROPERTIES DEPENDS test_scanio_helper_build)

add_test(test_scanio_readscans_run ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test_scanio_readscans "${PROJECT_SOURCE_DIR}")
add_test(test_scanio_readscans_build "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target test_scanio_readscans)
set_tests_properties(test_scanio_readscans_run PROPERTIES DEPENDS "test_scanio_readscans_build;test_libscan_io_uos_build;test_libscan_io_xyz_build test_icosphere")

if (WITH_LIBZIP)
  add_test(test_icosphere_zip "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target test_icosphere_zip)
  add_test(test_scanio_zipreader_run ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test_scanio_zipreader)
  add_test(test_scanio_zipreader_build "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target test_scanio_zipreader)
  set_tests_properties(test_scanio_zipreader_run PROPERTIES DEPENDS "test_scanio_zipreader_build;test_libscan_io_uos_rgb_build;test_icosphere_zip")
endif()
