add_subdirectory(scanio)
add_subdirectory(kdtree)
add_subdirectory(data/icosphere)
# the peopleremover test timeouts with MSVC
# with MinGW output precision degrades by another two digits
if (NOT WIN32)
  add_subdirectory(data/change_detection)
  add_subdirectory(peopleremover)
endif()
add_subdirectory(segmentation)
add_subdirectory(model)
add_subdirectory(apriltag)
add_subdirectory(show)
add_subdirectory(codestyle)
