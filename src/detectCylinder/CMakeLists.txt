add_executable(detectCylinder detectCylinder.cc cylinderDetector.cc cylinderAxisDetector.cc circleDetector.cc configFileCylinderDetector.cc)

target_link_libraries(detectCylinder scan newmat ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
