ADD_LIBRARY(py3dtk SHARED py3dtk.cc)
# make sure not to add the "lib" prefix
SET_TARGET_PROPERTIES(py3dtk PROPERTIES PREFIX "")
target_include_directories(py3dtk PUBLIC ${pyincl})
TARGET_LINK_LIBRARIES(py3dtk scan spherical_quadtree ${Boost_LIBRARIES} ${pylibs})
