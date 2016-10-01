if(WITH_PYTHON)
	find_package(PythonLibs 3.5 REQUIRED)
	# the following will issue a warning
	# see: https://gitlab.kitware.com/cmake/cmake/issues/16335
	find_package(Boost 1.45.0 COMPONENTS python-py35 REQUIRED)
	add_subdirectory(python)
endif()
