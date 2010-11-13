IF(WITH_GRIDDER)
  add_executable(2DGridder 2DGridder.cc line.cc gridlines.cc hough.cc viewpointinfo.cc gridWriter.cc parcelmanager.cc parcel.cc parcelinfo.cc scanGrid.cc grid.cc scanToGrid.cc gridPoint.cc scanmanager.cc) 
  target_link_libraries(2DGridder scanlib dl ANN)
ENDIF(WITH_GRIDDER)
