if(WITH_RIVLIB)
  if(${RiVLib_FOUND})
    include_directories(${RiVLib_INCLUDE_DIRS})

    find_package(Eigen3 REQUIRED)
    include_directories(${EIGEN3_INCLUDE_DIRS})

    add_executable(rxp2pose rxp2pose.cc)
    target_link_libraries(rxp2pose ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${RiVLib_CTRLLIB_LIBRARY} ${RiVLib_SCANLIB_LIBRARY})

    add_executable(rxp2gps rxp2gps.cc)
    target_link_libraries(rxp2gps ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${RiVLib_CTRLLIB_LIBRARY} ${RiVLib_SCANLIB_LIBRARY})

    add_executable(rxpinfo rxpinfo.cc)
    target_link_libraries(rxpinfo ${RiVLib_SCANLIB_LIBRARY})

  else()
      message("Cannot build rxptools. Need RiVLib version 2.2 or newer!")
  endif()
endif()
