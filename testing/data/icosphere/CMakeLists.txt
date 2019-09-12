add_executable(test_create_icosphere create_icosphere.cc)
target_link_libraries(test_create_icosphere)

if (WITH_LIBZIP)
  find_package(LibZip REQUIRED)
endif()

if (WITH_LIBZIP)
  add_executable(test_zip zip.cc)
  target_link_libraries(test_zip ${LIBZIP_LIBRARY})
endif()

add_custom_target(test_icosphere DEPENDS
	"uos/scan000.3d" "uos/scan000.frames" "uos/scan000.pose"
	"uosr/scan000.3d" "uosr/scan000.frames" "uosr/scan000.pose"
	"uos_rgb/scan000.3d" "uos_rgb/scan000.frames" "uos_rgb/scan000.pose"
	"xyz/scan000.xyz" "xyz/scan000.frames" "xyz/scan000.pose"
	"xyzr/scan000.xyz" "xyzr/scan000.frames" "xyzr/scan000.pose"
	"xyz_rgb/scan000.xyz" "xyz_rgb/scan000.frames" "xyz_rgb/scan000.pose"
	)
add_custom_command(OUTPUT
	"uos/scan000.3d" "uos/scan000.frames" "uos/scan000.pose"
	"uosr/scan000.3d" "uosr/scan000.frames" "uosr/scan000.pose"
	"uos_rgb/scan000.3d" "uos_rgb/scan000.frames" "uos_rgb/scan000.pose"
	"xyz/scan000.xyz" "xyz/scan000.frames" "xyz/scan000.pose"
	"xyzr/scan000.xyz" "xyzr/scan000.frames" "xyzr/scan000.pose"
	"xyz_rgb/scan000.xyz" "xyz_rgb/scan000.frames" "xyz_rgb/scan000.pose"
	COMMAND test_create_icosphere)

if (WITH_LIBZIP)
  add_custom_target(test_icosphere_zip DEPENDS "uos/normals.zip")
  add_custom_command(OUTPUT "uos/normals.zip" COMMAND test_zip "./uos/" DEPENDS "uos/normals/scan000.3d" "uos/normals/scan000.pose")
endif()

add_custom_target(test_icosphere_normals DEPENDS "uos/normals/scan000.3d" "uos/normals/scan000.pose")
add_custom_command(OUTPUT "uos/normals/scan000.3d" "uos/normals/scan000.pose" COMMAND calc_normals -s 0 -e 0 -f uos -g AKNN -k 20 "./uos/")
