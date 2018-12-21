add_test(test_change_detection_build "${CMAKE_COMMAND}" --build "${CMAKE_BINARY_DIR}" --target test_change_detection)
add_test(test_peopleremover_build "${CMAKE_COMMAND}" --build "${CMAKE_BINARY_DIR}" --target peopleremover)

add_test(test_peopleremover_run "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/peopleremover" -s 0 -e 9 --voxel-size=0.1 --maxrange-method=normals --normal-method=angle -f uosr "../data/change_detection")
set_tests_properties(test_peopleremover_run PROPERTIES DEPENDS "test_change_detection_build;test_peopleremover_build")

set(FILES_TO_HASH)
foreach(i RANGE 0 9)
        set(FILES_TO_HASH ${FILES_TO_HASH} ../data/change_detection/pplremover/scan00${i}.mask ../data/change_detection/static/scan00${i}.3d ../data/change_detection/static/scan00${i}.pose)
endforeach()

# We add the COMMAND directly to add_custom_target() and not to
# add_custom_command() because the former is always out of date and will thus
# always run. If we would use the latter, then the command would not run if
# hashes.txt already existed. Specifying DEPENDS in add_custom_command() would
# also not work because the files are created by add_test() which does not
# allow one to specify the output, so cmake would not know how to generate
# scan000.3d and the others.
add_custom_target(test_peopleremover_generate_md5 COMMAND "${CMAKE_COMMAND}" -E md5sum scan000.3d scan000.pose scan001.3d scan001.pose ${FILES_TO_HASH} > hashes.txt)

add_test(test_peopleremover_generate_md5 "${CMAKE_COMMAND}" --build "${CMAKE_BINARY_DIR}" --target test_peopleremover_generate_md5)
set_tests_properties(test_peopleremover_generate_md5 PROPERTIES DEPENDS test_peopleremover_run)

# to regenerate hashes.txt in case the input changed, use:
# ( cd ./.build/testing/peopleremover/; md5sum scan000.3d scan000.pose scan001.3d scan001.pose; for i in $(seq 0 9); do md5sum "$(printf "../data/change_detection/pplremover/scan%03d.mask" $i)" "$(printf "../data/change_detection/static/scan%03d.3d" $i)" "$(printf "../data/change_detection/static/scan%03d.pose" $i)"; done; ) > testing/peopleremover/hashes.txt
add_test(test_peopleremover_compare_md5 "${CMAKE_COMMAND}" -E compare_files hashes.txt "${PROJECT_SOURCE_DIR}/testing/peopleremover/hashes.txt")
