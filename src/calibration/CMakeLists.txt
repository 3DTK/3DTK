if(WITH_CALIBRATION)
    include_directories("${CMAKE_SOURCE_DIR}/3rdparty/apriltag-2015-03-18/")

    set(CAMCAL_SOURCES
        CalibrationPattern.cc
        PictureHandler.cc
        CalibrationToolbox.cc
        AprilTag.cc
        Chessboard.cc
    )
    add_executable(camcal camcal.cc ${CAMCAL_SOURCES})
    target_link_libraries(camcal apriltag ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})

    add_executable(undistort undistort.cc)
    target_link_libraries(undistort ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
endif()
