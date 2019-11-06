# on mingw images have a few random pixels difference
if (MINGW)
	find_package(PNG REQUIRED)
	add_executable(test_compare_png compare_png.cc)
	target_include_directories(test_compare_png PUBLIC ${PNG_INCLUDE_DIRS})
	target_compile_definitions(test_compare_png PUBLIC ${PNG_DEFINITIONS})
	target_link_libraries(test_compare_png ${PNG_LIBRARIES})

	add_test(test_compare_png_build "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target test_compare_png)
endif()

add_test(test_show_build "${CMAKE_COMMAND}" --build "${CMAKE_BINARY_DIR}" --target showbin)

if (APPLE)
	# since show segfaults on apple when cleaning up scanio, it stalls
	# (maybe a gui dialog that wants to be clicked?) and we have to kill it
	# ourselves
	set(MACOSTIMEOUT sh -c "\"$@\" & pid=$!\; sleep 10\; kill -9 $pid || true\; exit 0\;" --)
endif()

if (WIN32)
	set(DIR win32)
elseif(APPLE)
	set(DIR macos)
else()
	# * execute test under xvfb to allow running this test on headless CI
	#   systems
	# * using xvfb also prevents a window from shortly opening on systems
	#   with Xorg
	# * using xvfb also produces more consistent results because the mesa
	#   software OpenGL renderer will output the same results independent
	#   on the system's graphics hardware and drivers
	# * use --auto-servernum so that tests can be run in parallel
	# * use --server-args to override bad defaults on stretch, xenial,
	#   bionic and cygwin
	set(XVFB xvfb-run --auto-servernum "--server-args=-screen 0 1280x1024x24")
	set(DIR xvfb)
endif()

# FIXME: windows aborts on xyz_rgb
# FIXME: show on macos segfaults when closing

foreach(format IN ITEMS uos uosr uos_rgb xyz xyzr xyz_rgb)
	if (format MATCHES ".*_rgb$")
		set(formatarg --color)
	elseif(format MATCHES ".*r$")
		set(formatarg --reflectance --colormap grey)
	else()
		unset(formatarg)
	endif()
	if (MSVC AND format STREQUAL xyz_rgb)
		# xyz_rgb crashes on msvc
		continue()
	endif()
	if (MSVC AND format MATCHES ".*r$")
		# uosr and xyzr segfault on msvc
		continue()
	endif()
	if (MINGW AND format MATCHES ".*r$")
		# uosr and xyzr render as if they were uos or xyz
		continue()
	endif()
	add_test(test_show_${format}_run
		${MACOSTIMEOUT}
		${XVFB}
		"${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/show"
		-s 0 -e 0 -f ${format}
		--screenshot
		--scale=0.5
		--position=0,0,-2.2
		${formatarg}
		--screenshot-filename=show_${format}.png
		--hide-label
		--pointsize=4
		--no-config
		"../data/icosphere/${format}/")
	set_tests_properties(test_show_${format}_run PROPERTIES DEPENDS "test_show_build;test_icosphere")

	if (MINGW AND format MATCHES ".*_rgb$")
		# the *_rgb formats on mingw produce slightly different output
		# in the blue channel every time
		add_test(test_show_${format}_compare ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test_compare_png 50 "show_${format}.png" "${PROJECT_SOURCE_DIR}/testing/show/${DIR}/show_${format}.png")
		set_tests_properties(test_show_${format}_compare PROPERTIES DEPENDS "test_show_${format}_run;test_compare_png_build")
	else()
		add_test(test_show_${format}_compare "${CMAKE_COMMAND}" -E compare_files "show_${format}.png" "${PROJECT_SOURCE_DIR}/testing/show/${DIR}/show_${format}.png")
		set_tests_properties(test_show_${format}_compare PROPERTIES DEPENDS test_show_${format}_run)
	endif()
endforeach()
