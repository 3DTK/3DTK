IF(WITH_COLLISION)
  add_executable(collision_sphere collision_sphere.cc)
  target_link_libraries(collision_sphere scan ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
ENDIF(WITH_COLLISION)
