if (WITH_PEOPLEREMOVER)
	add_executable(peopleremover peopleremover.cc)
	target_link_libraries(peopleremover scan ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
endif()
