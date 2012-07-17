IF(WITH_GRIDDER)
  add_executable(2DGridder 2DGridder.cc line.cc gridlines.cc hough.cc viewpointinfo.cc gridWriter.cc parcelmanager.cc parcel.cc parcelinfo.cc scanGrid.cc grid.cc scanToGrid.cc gridPoint.cc scanmanager.cc) 

  IF (UNIX)
    target_link_libraries(2DGridder scan dl ANN)
  ENDIF(UNIX)

  IF (WIN32)
    target_link_libraries(2DGridder scan ANN XGetopt)
  ENDIF(WIN32)

ENDIF(WITH_GRIDDER)
