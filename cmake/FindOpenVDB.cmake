include(FindPackageHandleStandardArgs)

find_path(OPENVDB_INCLUDE_DIR
    NAMES openvdb/openvdb.h
    HINTS
    $ENV{OPENVDB_DIR}
    PATH_SUFFIXES include openvdb/include
    PATHS
    /usr
    /usr/local
    /opt
)

find_library(OPENVDB_LIBRARY
    NAMES openvdb
    HINTS
    $ENV{OPENVDB_DIR}
    PATH_SUFFIXES lib64 lib openvdb/lib
    PATHS
    /usr
    /usr/local
    /opt
)

find_package_handle_standard_args(OPENVDB DEFAULT_MSG OPENVDB_LIBRARY OPENVDB_INCLUDE_DIR)

if(OPENVDB_FOUND)
    set(OPENVDB_INCLUDE_DIRS ${OPENVDB_INCLUDE_DIR})
    set(OPENVDB_LIBRARIES ${OPENVDB_LIBRARY})
    mark_as_advanced(OPENVDB_INCLUDE_DIRS OPENVDB_LIBRARIES)
endif()
