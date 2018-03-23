
include_directories(${CMAKE_SOURCE_DIR}/include/curvefusion/)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(fusion fusion.cc curves.cc)
 target_link_libraries(fusion ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY})

