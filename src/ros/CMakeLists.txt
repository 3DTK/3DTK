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
  )
  find_package(Eigen3 REQUIRED)
  find_package(PkgConfig)

  include_directories(${CATKIN_DEVEL_PREFIX}/include)
  include_directories(${catkin_INCLUDE_DIRS})
  include_directories(${EIGEN3_INCLUDE_DIRS})

  pkg_check_modules(BFL REQUIRED bfl)
  include_directories(${BFL_INCLUDE_DIRS})
  link_directories(${BFL_LIBRARY_DIRS})

  #set(ENV{ROS_LANG_DISABLE} "genlisp:genpy")

  add_subdirectory(riegl)
  add_subdirectory(volksbotRT)

  #catkin_package(CATKIN_DEPENDS rosbag)

  add_executable(bag2scan bag2scan.cc calibration.cc importer.cc ekf_filter.cc powell.cc nrutil.cc my_robot_pose_ekf/odom_estimation.cpp my_robot_pose_ekf/nonlinearanalyticconditionalgaussianodo.cpp odometry.cc)
  #target_link_libraries(bag2scan scan roslib rosconsole rostime rosbag roslz4 rosbag_storage roscpp_serialization roscpp cpp_common tf2_ros tf_conversions tf2 tf ${RiVLib_SCANLIB_LIBRARY} xmlrpcpp laser_geometry newmat orocos-bfl)
  target_link_libraries(bag2scan scan ${catkin_LIBRARIES} ${RiVLib_SCANLIB_LIBRARY} ${BFL_LIBRARIES})
endif()
