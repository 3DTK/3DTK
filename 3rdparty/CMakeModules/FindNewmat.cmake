# - Find newmat
# Find the native newmat includes and library
# This module defines
#  NEWMAT_INCLUDE_DIR, where to find newmat/newmat.h, etc.
#  NEWMAT_LIBRARIES, the libraries needed to use newmat.
#  NEWMAT_FOUND, If false, do not try to use newmat.
# also defined, but not for general use are
#  NEWMAT_LIBRARY, where to find the newmat library.

include(FindPackageHandleStandardArgs)

find_path(NEWMAT_INCLUDE_DIR newmat/newmat.h)
find_library(NEWMAT_LIBRARY newmat)

find_package_handle_standard_args(NEWMAT DEFAULT_MSG
	NEWMAT_LIBRARY NEWMAT_INCLUDE_DIR)

if(NEWMAT_FOUND)
	set(NEWMAT_INCLUDE_DIRS ${NEWMAT_INCLUDE_DIR})
	set(NEWMAT_LIBRARIES ${NEWMAT_LIBRARY})
endif()
