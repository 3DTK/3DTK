include_directories(${PYTHON_INCLUDE_DIRS})

ADD_LIBRARY(py3dtk SHARED py3dtk.cc)
# make sure not to add the "lib" prefix
SET_TARGET_PROPERTIES(py3dtk PROPERTIES PREFIX "")
TARGET_LINK_LIBRARIES(py3dtk scan spherical_quadtree ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
