add_executable(peopleremover peopleremover.cc common.cc)
target_link_libraries(peopleremover scan spherical_quadtree ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
