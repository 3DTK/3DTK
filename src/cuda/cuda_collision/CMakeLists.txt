IF(WITH_CUDA)
  add_executable(cuda_collision_model cuda_collision_model.cc)
  target_link_libraries(cuda_collision_model cudaslam scan ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
ENDIF(WITH_CUDA)
