if (WITH_OPENCV)
	add_executable(test_apriltag main.cc)
	target_link_libraries(test_apriltag apriltag ${OpenCV_LIBRARIES})
	add_test(test_apriltag_build "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target test_apriltag)
	# on MSVC:
	# EXEC : OpenCV(3.4.3) error : Assertion failed (dims >= 3) in cv::Mat::ptr, file C:\Tools\vcpkg\installed\x64-windows\include\opencv2/core/mat.inl.hpp, line 998 [C:\slam6d\RUN_TESTS.vcxproj]
	if (NOT MSVC)
		add_test(test_apriltag ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test_apriltag)
		set_tests_properties(test_apriltag PROPERTIES DEPENDS test_apriltag_build)
	endif()
endif()
