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
if(APPLE)
  set(CMAKE_MACOSX_RPATH 1)
endif()
cmake_minimum_required(VERSION 2.4)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

MESSAGE(STATUS "With cvblob")
find_package(OpenCV REQUIRED)

add_library(cvblob
  cvblob.cpp
  cvlabel.cpp
  cvaux.cpp
  cvcontour.cpp
  cvtrack.cpp
  cvcolor.cpp
)

target_link_libraries(cvblob ${OpenCV_LIBS})
