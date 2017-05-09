include(FindPackageHandleStandardArgs)

find_path(TBB_INCLUDE_DIR
    NAMES tbb/tbb.h
    HINTS
    $ENV{TBB_DIR}
    PATH_SUFFIXES include
    PATHS
    /usr
    /usr/local
    /opt
)

find_library(TBB_LIBRARY
    NAMES tbb
    HINTS
    $ENV{TBB_DIR}
    PATH_SUFFIXES lib64 lib
    PATHS
    /usr
    /usr/local
    /opt
)

find_package_handle_standard_args(TBB DEFAULT_MSG TBB_LIBRARY TBB_INCLUDE_DIR)

if(TBB_FOUND)
    set(TBB_INCLUDE_DIRS ${TBB_INCLUDE_DIR})
    set(TBB_LIBRARIES ${TBB_LIBRARY})
    mark_as_advanced(TBB_INCLUDE_DIRS TBB_LIBRARIES)
endif()
