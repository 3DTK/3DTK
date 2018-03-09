add_executable(test_generate_change_detection generate.cc)
target_link_libraries(test_generate_change_detection)

set(FILES)
foreach(i RANGE 0 9)
        set(FILES ${FILES} scan00${i}.3d scan00${i}.pose)
endforeach()

add_custom_target(test_change_detection DEPENDS ${FILES})
add_custom_command(OUTPUT ${FILES} COMMAND test_generate_change_detection)
