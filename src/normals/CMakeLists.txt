if (WITH_OPENCV)
  add_definitions(-DWITH_OPENCV)
  include_directories(${NEWMAT_INCLUDE_DIRS})

  add_library(normals_panorama normals_panorama.cc)

  target_link_libraries(normals_panorama ${NEWMAT_LIBRARIES} scan fbr ${OpenCV_LIBS} ${Boost_LIBRARIES})

  set(NORMALS_PANORAMA_LIBRARIES normals_panorama)
endif()

add_executable(calc_normals calc_normals.cc)
target_link_libraries(calc_normals scan ${NORMALS_PANORAMA_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${SHOW_LIBS})
