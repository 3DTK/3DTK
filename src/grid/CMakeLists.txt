add_executable(2DGridder 2DGridder.cc line.cc gridlines.cc hough.cc viewpointinfo.cc gridWriter.cc parcelmanager.cc parcel.cc parcelinfo.cc scanGrid.cc grid.cc scanToGrid.cc gridPoint.cc scanmanager.cc) 

if (UNIX)
  target_link_libraries(2DGridder scan_shared ${ANN_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
endif()

if (WIN32)
  target_link_libraries(2DGridder scan_static ${ANN_LIBRARIES_STATIC} XGetopt)
endif()
