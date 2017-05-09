include(FindPackageHandleStandardArgs)

find_path(OPENEXR_INCLUDE_DIR
    NAMES OpenEXR/half.h
    HINTS
    $ENV{OPENEXR_DIR}
    PATH_SUFFIXES include
    PATHS
    /usr
    /usr/local
    /opt
)

find_library(OPENEXR_LIBRARY
    NAMES Half
    HINTS
    $ENV{OPENEXR_DIR}
    PATH_SUFFIXES lib64 lib
    PATHS
    /usr
    /usr/local
    /opt
)

find_package_handle_standard_args(OPENEXR DEFAULT_MSG OPENEXR_LIBRARY OPENEXR_INCLUDE_DIR)

if(OPENEXR_FOUND)
    set(OPENEXR_INCLUDE_DIRS ${OPENEXR_INCLUDE_DIR})
    set(OPENEXR_LIBRARIES ${OPENEXR_LIBRARY})
    mark_as_advanced(OPENEXR_INCLUDE_DIRS OPENEXR_LIBRARIES)
endif()
