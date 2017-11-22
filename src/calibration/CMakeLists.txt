if (WITH_OPENCV AND WITH_EIGEN3)
  include_directories(${APRILTAG_INCLUDE_DIRS})

  set(CAMCAL_SOURCES
      CalibrationPattern.cc
      PictureHandler.cc
      CalibrationToolbox.cc
      AprilTag.cc
      Chessboard.cc
  )
  add_executable(camcal camcal.cc ${CAMCAL_SOURCES})
  target_link_libraries(camcal apriltag ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})

  add_executable(undistortImage undistortImage.cc)
  target_link_libraries(undistortImage ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})

  add_executable(undistortPoints undistortPoints.cc)
  target_link_libraries(undistortPoints ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})

  add_executable(planeFromExtrinsics planeFromExtrinsics.cc)
  target_link_libraries(planeFromExtrinsics ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})

  add_executable(transformsFromExtrinsics transformsFromExtrinsics.cc)
  target_link_libraries(transformsFromExtrinsics ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})

  configure_file(createAprilGrid.py ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/createAprilGrid COPYONLY)
endif()
