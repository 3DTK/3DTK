add_executable(2DGridder 2DGridder.cc line.cc gridlines.cc hough.cc viewpointinfo.cc gridWriter.cc parcelmanager.cc parcel.cc parcelinfo.cc scanGrid.cc grid.cc scanToGrid.cc gridPoint.cc scanmanager.cc)

target_link_libraries(2DGridder scan ${ANN_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
if (WIN32)
  target_link_libraries(2DGridder XGetopt)
endif()
