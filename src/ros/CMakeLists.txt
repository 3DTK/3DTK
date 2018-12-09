if(WITH_ROS)
  # we need to set a new project name because otherwise we get:
  #
  #   ERROR: package name '3DTK' is illegal and cannot be used in message generation.
  #   Please see http://ros.org/wiki/Names
  project("bag2scan")

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
      nav_msgs
  )
  find_package(Eigen3 REQUIRED)
  find_package(PCL REQUIRED)
  find_package(PkgConfig)

  # Necessary on Debian stretch for linking PCL correctly
  find_package(Qt5Widgets)

  include_directories(${CATKIN_DEVEL_PREFIX}/include)
  include_directories(${catkin_INCLUDE_DIRS})

  # only necessary until this catkin fix propagated into Debian Buster:
  # https://salsa.debian.org/science-team/ros-catkin/commit/68154962414a4c7dfcedce0cd183135cd71a33c8
  include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})

  pkg_check_modules(OROCOS_BFL REQUIRED orocos-bfl)
  include_directories(${OROCOS_BFL_INCLUDE_DIRS})
  link_directories(${OROCOS_BFL_LIBRARY_DIRS})

  add_subdirectory(riegl)
  add_subdirectory(volksbotRT)

  if(WITH_RIVLIB)
  add_executable(bag2scan bag2scan.cc calibration.cc importer.cc ekf_filter.cc powell.cc nrutil.cc my_robot_pose_ekf/odom_estimation.cpp my_robot_pose_ekf/nonlinearanalyticconditionalgaussianodo.cpp odometry.cc)
  target_link_libraries(bag2scan scan ${catkin_LIBRARIES} ${RiVLib_SCANLIB_LIBRARY} ${OROCOS_BFL_LIBRARIES})
  add_dependencies(bag2scan riegl_generate_messages_cpp)
  add_dependencies(bag2scan volksbotRT_generate_messages_cpp)
  endif()

  add_executable(cartographer2scan cartographer2scan.cc)
  target_link_libraries(cartographer2scan ${catkin_LIBRARIES} ${PCL_LIBRARIES})

  add_library(tdtkroswrapper
    tdtkroswrapper.cc
    #slam6Dwrapper.cc
    icp6Dwrapper.cc
  )
  target_link_libraries(tdtkroswrapper scan ${catkin_LIBRARIES} ${PCL_LIBRARIES})

  add_executable(scan2bag scan2bag.cc)
  target_link_libraries(scan2bag ${catkin_LIBRARIES} ${PCL_LIBRARIES} scan tdtkroswrapper)

  #add_executable(slam6D_node slam6D_node.cc)
  #target_link_libraries(slam6D_node ${catkin_LIBRARIES} tdtkroswrapper)

  add_executable(icp6D_node icp6D_node.cc)
  target_link_libraries(icp6D_node ${catkin_LIBRARIES} tdtkroswrapper)

endif()
