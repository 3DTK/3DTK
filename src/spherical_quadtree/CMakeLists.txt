add_library(spherical_quadtree_objects OBJECT spherical_quadtree.cc)
# compile with -fPIC so that the result can be made into a shared library
set_property(TARGET spherical_quadtree_objects PROPERTY POSITION_INDEPENDENT_CODE 1)
add_library(spherical_quadtree_static STATIC $<TARGET_OBJECTS:spherical_quadtree_objects>)
add_library(spherical_quadtree_shared SHARED $<TARGET_OBJECTS:spherical_quadtree_objects>)
set_target_properties(spherical_quadtree_static PROPERTIES OUTPUT_NAME spherical_quadtree CLEAN_DIRECT_OUTPUT 1)
set_target_properties(spherical_quadtree_shared PROPERTIES OUTPUT_NAME spherical_quadtree CLEAN_DIRECT_OUTPUT 1)
