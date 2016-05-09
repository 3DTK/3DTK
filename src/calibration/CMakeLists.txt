if(WITH_CALIBRATION)
    include_directories("${CMAKE_SOURCE_DIR}/3rdparty/apriltag/")

    set(CAMCAL_SOURCES
        CalibrationPattern.cc
        PictureHandler.cc
        CalibrationToolbox.cc
        AprilTag.cc
        Chessboard.cc
    )
    add_executable(camcal camcal.cc ${CAMCAL_SOURCES})

    target_link_libraries(camcal apriltag ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

    add_executable(undistort undistort.cc)
    target_link_libraries(undistort ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})
endif()
