# Copyright (C) 2007 by Cristóbal Carnero Liñán
# grendel.ccl@gmail.com
#
# This file is part of cvBlob.
#
# cvBlob is free software: you can redistribute it and/or modify
# it under the terms of the Lesser GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# cvBlob is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# Lesser GNU General Public License for more details.
#
# You should have received a copy of the Lesser GNU General Public License
# along with cvBlob.  If not, see <http://www.gnu.org/licenses/>.
#

cmake_minimum_required(VERSION 2.4)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

MESSAGE(STATUS "With cvblob")
find_package(OpenCV REQUIRED)

add_library(cvblob_objects OBJECT
  cvblob.cpp
  cvlabel.cpp
  cvaux.cpp
  cvcontour.cpp
  cvtrack.cpp
  cvcolor.cpp
)

# compile with -fPIC so that the result can be made into a shared library
set_property(TARGET cvblob_objects PROPERTY POSITION_INDEPENDENT_CODE 1)

add_library(cvblob_static STATIC $<TARGET_OBJECTS:cvblob_objects>)
add_library(cvblob_shared SHARED $<TARGET_OBJECTS:cvblob_objects>)

set_target_properties(cvblob_static PROPERTIES OUTPUT_NAME cvblob CLEAN_DIRECT_OUTPUT 1)
set_target_properties(cvblob_shared PROPERTIES OUTPUT_NAME cvblob CLEAN_DIRECT_OUTPUT 1)

target_link_libraries(cvblob_static ${OpenCV_LIBS})
target_link_libraries(cvblob_shared ${OpenCV_LIBS})
