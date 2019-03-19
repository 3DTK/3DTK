if(WITH_ROS)
  project("fusion")

  find_package(catkin REQUIRED COMPONENTS
      roscpp
      rosbag
      tf
      laser_geometry
      pcl_conversions
      tf_conversions
      tf2_sensor_msgs
      urdf
      kdl_parser
      robot_state_publisher
      message_filters
      geometry_msgs
      sensor_msgs
      std_msgs
  )

  find_package(Eigen3 REQUIRED)
  find_package(PCL REQUIRED)
  find_package(PkgConfig)

  include_directories(${CATKIN_DEVEL_PREFIX}/include)
  include_directories(${catkin_INCLUDE_DIRS})

  #include_directories(${CMAKE_SOURCE_DIR}/include/curvefusion/)


  include_directories(${EIGEN3_INCLUDE_DIR})

  include_directories(${NEWMAT_INCLUDE_DIRS})

  if(WITH_RIVLIB)
    if(${RiVLib_FOUND})
      add_executable(fusion fusion.cc common.cc extractTrajectory.cc curves.cc timestamps.cc tf_broadcaster.cc)
      target_link_libraries(fusion ${catkin_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${NEWMAT_LIBRARIES} ${OpenCV_LIBRARIES} ${RiVLib_CTRLLIB_LIBRARY} ${RiVLib_SCANLIB_LIBRARY})
    else()
      message("Cannot build curvefusion. Need RiVLib version 2.2 or newer!")
    endif()
  endif()
endif()

