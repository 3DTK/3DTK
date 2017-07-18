if(WITH_CALIBRATION)
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

    add_executable(undistort undistort.cc)
    target_link_libraries(undistort ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})

    add_executable(lasercal lasercal.cc CalibrationPattern.cc PictureHandler.cc AprilTag.cc LaserTools.cc)
    target_link_libraries(lasercal apriltag ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})

    configure_file(createAprilGrid.py ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/createAprilGrid COPYONLY)
endif()
