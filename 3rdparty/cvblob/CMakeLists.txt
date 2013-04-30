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
IF (WITH_THERMO)

MESSAGE(STATUS "With cvblob")
find_package(OpenCV REQUIRED)

set(CVBLOB_SRCS 
  cvblob.cpp
  cvlabel.cpp
  cvaux.cpp
  cvcontour.cpp
  cvtrack.cpp
  cvcolor.cpp
)

set_source_files_properties(${cvBlob_SRCS}
                            PROPERTIES
                            COMPILE_FLAGS "-O3 ${ADDITIONAL_OPENCV_FLAGS}"
)

add_library(cvblob STATIC ${CVBLOB_SRCS})

IF(EXPORT_SHARED_LIBS)
add_library(cvblob_s SHARED ${CVBLOB_SRCS})
target_link_libraries(cvblob_s ${OpenCV_LIBS})
ENDIF(EXPORT_SHARED_LIBS)

ENDIF (WITH_THERMO)
