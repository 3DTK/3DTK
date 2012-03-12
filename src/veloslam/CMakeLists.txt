IF(WITH_VELOSLAM)
  add_executable(veloslam veloslam.cc veloscan.cc debugview.cc pcddump.cc tracker.cc trackermanager.cc)
  
IF(UNIX)
  target_link_libraries(veloslam dl scanlib newmat sparse ANN)
ENDIF(UNIX)

IF(WIN32)
  target_link_libraries(veloslam scanlib newmat sparse ANN XGetopt ${Boost_LIBRARIES})
ENDIF(WIN32)

ENDIF(WITH_VELOSLAM)