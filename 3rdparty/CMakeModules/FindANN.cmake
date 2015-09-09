# Finds ANN
#
# This module defines
#  ANN_INCLUDE_DIRS
#  ANN_LIBRARIES
#  ANN_FOUND

include(FindPackageHandleStandardArgs)

find_path(ANN_INCLUDE_DIR ANN/ANN.h)
find_library(ANN_LIBRARY ann)

find_package_handle_standard_args(ANN DEFAULT_MSG
    ANN_LIBRARY ANN_INCLUDE_DIR)

if(ANN_FOUND)
    set(ANN_INCLUDE_DIRS ${ANN_INCLUDE_DIR})
    set(ANN_LIBRARIES ${ANN_LIBRARY})
endif()
