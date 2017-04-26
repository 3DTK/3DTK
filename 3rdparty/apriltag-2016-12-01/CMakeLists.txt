include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(SOURCES
    apriltag.c
    apriltag_quad_thresh.c
    tag16h5.c
    tag25h7.c
    tag25h9.c
    tag36artoolkit.c
    tag36h10.c
    tag36h11.c

    common/g2d.c
    common/getopt.c
    common/homography.c
    common/image_f32.c
    common/image_u8.c
    common/image_u8x3.c
    common/matd.c
    common/pnm.c
    common/string_util.c
    common/svd22.c
    common/time_util.c
    common/unionfind.c
    common/workerpool.c
    common/zarray.c
    common/zhash.c
    common/zmaxheap.c
)

add_library(apriltag ${SOURCES})
target_link_libraries(apriltag pthread)
set_target_properties(apriltag PROPERTIES EXCLUDE_FROM_ALL true)

set(APRILTAG_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}" CACHE INTERNAL "AprilTag include directories" FORCE)
