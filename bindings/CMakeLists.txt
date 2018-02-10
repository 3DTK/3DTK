if(WITH_PYTHON)
	find_package(PythonLibs 3 REQUIRED)
	set(version ${PYTHONLIBS_VERSION_STRING})
	STRING( REGEX REPLACE "([0-9.]+).[0-9]+" "\\1" version ${version} )
	STRING( REGEX REPLACE "[^0-9]" "" version ${version} )
	# the following will issue a warning
	# see: https://gitlab.kitware.com/cmake/cmake/issues/16335
	find_package(Boost 1.45.0 COMPONENTS "python-py${version}" REQUIRED)
	add_subdirectory(python)
endif()
