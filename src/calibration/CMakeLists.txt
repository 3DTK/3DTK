if (WITH_OPENCV AND WITH_EIGEN3)
  include_directories(${APRILTAG_INCLUDE_DIRS})

  set(CAMCAL_SOURCES
      CalibrationPattern.cc
      PictureHandler.cc
      CalibrationToolbox.cc
      AprilTag.cc
      Chessboard.cc)

  set(DETECT_SOURCES
          AprilTag.cc
          Chessboard.cc
          Detector.cpp)

  set(CALIBRATE_SOURCES
          AprilTag.cc
          Calibrator.cc)

  add_executable(camcal camcal.cc ${CAMCAL_SOURCES})
  if (WIN32)
    target_link_libraries(camcal winmm)
  endif()
  target_link_libraries(camcal apriltag ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})

  add_executable(undistortImage undistortImage.cc)
  target_link_libraries(undistortImage ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})

  add_executable(undistortPoints undistortPoints.cc)
  target_link_libraries(undistortPoints ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})

  add_executable(planeFromExtrinsics planeFromExtrinsics.cc)
  target_link_libraries(planeFromExtrinsics ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})

  add_executable(transformsFromExtrinsics transformsFromExtrinsics.cc)
  target_link_libraries(transformsFromExtrinsics ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})

  add_executable(detect detect.cc ${DETECT_SOURCES})
  if (WIN32)
    target_link_libraries(detect winmm)
  endif()
  target_link_libraries(detect apriltag ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})

  add_executable(calibrate calibrate.cc ${CALIBRATE_SOURCES})
  target_link_libraries(calibrate apriltag ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})

  configure_file(createAprilGrid.py ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/createAprilGrid COPYONLY)
endif()
