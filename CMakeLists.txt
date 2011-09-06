cmake_minimum_required (VERSION 2.6.1)
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/3rdparty/CMakeModules" ${CMAKE_MODULE_PATH})
project (Slam6D)


#include_directories(OPENGL_INCLUDE_DIR)
set(Boost_USE_STATIC_LIBS TRUE)
SET(Boost_ADDITIONAL_VERSIONS "1.42" "1.42.0" "1.44" "1.44.0" "1.45.0" "1.45")
IF(WIN32)
  # for some unknown reason no one variant works on all windows platforms
  find_package( Boost REQUIRED)
  #find_package( Boost COMPONENTS serialization graph regex REQUIRED)
ELSE(WIN32)
  find_package( Boost COMPONENTS serialization graph regex REQUIRED)
ENDIF(WIN32)
if(Boost_FOUND)
  link_directories(${BOOST_LIBRARY_DIRS})
  include_directories(${Boost_INCLUDE_DIRS})
  add_definitions(${Boost_LIB_DIAGNOSTIC_DEFINITIONS})
endif()

#################################################
# Declare Options and modify build accordingly ##
#################################################

## Show 
OPTION(WITH_SHOW "Whether to build Show. This is the Visualization program of slam6d. ON/OFF" ON)

IF(WITH_SHOW)
  FIND_PACKAGE(OpenGL REQUIRED)
  FIND_PACKAGE(GLUT REQUIRED)
  MESSAGE(STATUS "With show")
ELSE(WITH_SHOW)
  #  SET (WITH_OCTREE_DISPLAY "ON" CACHE INTERNAL "" FORCE)
  MESSAGE(STATUS "Without show")
ENDIF(WITH_SHOW)

## WXShow 
OPTION(WITH_WXSHOW "Whether to build WXShow. This is the wxwidgets variant of Show. ON/OFF" OFF)

IF(WITH_WXSHOW)
  FIND_PACKAGE(OpenGL REQUIRED)
  FIND_PACKAGE(GLUT REQUIRED)
  find_package(wxWidgets COMPONENTS core base gl REQUIRED)
  # set wxWidgets_wxrc_EXECUTABLE to be ignored in the configuration
  SET (wxWidgets_wxrc_EXECUTABLE " " CACHE INTERNAL "" FORCE) 
  # wxWidgets include (this will do all the magic to configure everything)
  include( ${wxWidgets_USE_FILE})
  MESSAGE(STATUS "With wxshow")
ELSE(WITH_XWSHOW)
  MESSAGE(STATUS "Without wxshow")
ENDIF(WITH_WXSHOW)

IF(WITH_SHOW)
  FIND_PACKAGE(OpenGL REQUIRED)
  FIND_PACKAGE(GLUT REQUIRED)
  MESSAGE(STATUS "With show")
ELSE(WITH_SHOW)
  #  SET (WITH_OCTREE_DISPLAY "ON" CACHE INTERNAL "" FORCE)
  MESSAGE(STATUS "Without show")
ENDIF(WITH_SHOW)

## Shapes
OPTION(WITH_SHAPE_DETECTION "Whether to build shapes and planes executable for detecting planes. ON/OFF" OFF)
IF(WITH_SHAPE_DETECTION)
  MESSAGE(STATUS "With shape detection")
ELSE(WITH_SHAPE_DETECTION)
  MESSAGE(STATUS "Without shape detection")
ENDIF(WITH_SHAPE_DETECTION)


## Octree
OPTION(WITH_OCTREE_DISPLAY "Whether to use octree display for efficiently culling scans ON/OFF" ON)

IF(WITH_OCTREE_DISPLAY)
  MESSAGE(STATUS "Using octree display")
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_GL_POINTS")
ELSE(WITH_OCTREE_DISPLAY)
  MESSAGE(STATUS "Using displaylists: Warning potentially much slower")
ENDIF(WITH_OCTREE_DISPLAY)
#SET (WITH_OCTREE_DISPLAY ${WITH_OCTREE_DISPLAY} CACHE BOOL 
#"Whether to use octree display for efficiently culling scans ON/OFF" FORCE)

## Gridder 
OPTION(WITH_GRIDDER "Whether to build the 2DGridder binary ON/OFF" ON)

IF(WITH_GRIDDER)
  MESSAGE(STATUS "With 2DGridder")
ELSE(WITH_GRIDDER)
  MESSAGE(STATUS "Without 2DGridder")
ENDIF(WITH_GRIDDER)

## Home-made Laserscanner 
OPTION(WITH_DAVID_3D_SCANNER "Whether to build the David scanner app for homemade laser scanners binary ON/OFF" OFF)

IF(WITH_DAVID_3D_SCANNER)
  MESSAGE(STATUS "With David scanner")
ELSE(WITH_DAVID_3D_SCANNER)
  MESSAGE(STATUS "Without David scanner")
ENDIF(WITH_DAVID_3D_SCANNER)

## Tools 
OPTION(WITH_TOOLS "Whether to build additional tools like convergence frame_to_graph etc. ON/OFF" ON)

IF(WITH_TOOLS)
  MESSAGE(STATUS "With Tools")
ELSE(WITH_TOOLS)
  MESSAGE(STATUS "Without Tools")
ENDIF(WITH_TOOLS)



## Scan reduction
OPTION(WITH_SCAN_REDUCTION "Whether to build the scan reduction binary scan_red ON/OFF" ON)

IF(WITH_SCAN_REDUCTION)
  MESSAGE(STATUS "With scan_red")
ELSE(WITH_SCAN_REDUCTION)
  MESSAGE(STATUS "Without scan_red")
ENDIF(WITH_SCAN_REDUCTION)

## Scan difference
OPTION(WITH_SCAN_DIFF "Whether to build the scan_diff binary ON/OFF" ON)

IF(WITH_SCAN_DIFF)
  MESSAGE(STATUS "With scan_diff")
ELSE(WITH_SCAN_DIFF)
  MESSAGE(STATUS "Without scan_diff")
ENDIF(WITH_SCAN_DIFF)


## RivLib
OPTION(WITH_RIVLIB "Whether the RIEGL rivlib is present ON/OFF" OFF)

IF(WITH_RIVLIB)
  MESSAGE(STATUS "Compiling a scan IO for RXP files")
  include_directories(${CMAKE_SOURCE_DIR}/3rdparty)
  link_directories(${CMAKE_SOURCE_DIR}/3rdparty)
  SET(RIEGL_DIR ${CMAKE_SOURCE_DIR}/3rdparty/riegl/)
  IF(WIN32)
    SET(RIVLIB ${RIEGL_DIR}libscanlib-mt.lib ${RIEGL_DIR}libctrllib-mt.lib ${RIEGL_DIR}libboost_system-mt-1_43_0-vns.lib) 
  ELSE(WIN32)
    SET(RIVLIB ${RIEGL_DIR}libscanlib-mt-s.a ${RIEGL_DIR}libctrllib-mt-s.a ${RIEGL_DIR}libboost_system-mt-s-1_43_0-vns.a pthread)
  ENDIF(WIN32)
  FIND_PACKAGE(LibXml2 )

ELSE(WITH_RIVLIB)
  MESSAGE(STATUS "Do NOT compile a scan IO for RXP")
ENDIF(WITH_RIVLIB)

## CUDA support, TODO depend on CUDA_FIND
OPTION(WITH_CUDA "Compile with CUDA support" OFF)
IF(WITH_CUDA)
  MESSAGE(STATUS "Compiling WITH CUDA support")
  FIND_PACKAGE(CUDA)
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_CUDA")
ELSE(WITH_CUDA)
  MESSAGE(STATUS "Compiling WITHOUT CUDA support")
ENDIF(WITH_CUDA)

## PMD 
OPTION(WITH_PMD "Whether to build the PMD tools like grabVideoAnd3D calibrate etc. ON/OFF" OFF)

IF(WITH_PMD)
  FIND_PACKAGE(OpenGL REQUIRED)
  MESSAGE(STATUS "With Tools")
ELSE(WITH_PMD)
  MESSAGE(STATUS "Without Tools")
ENDIF(WITH_PMD)

## SIFT 
OPTION(WITH_SIFT "Whether to compile registration based on sift features and autopano. ON/OFF" OFF)

IF(WITH_SIFT)
  MESSAGE(STATUS "With Sift ")
ELSE(WITH_SIFT)
  MESSAGE(STATUS "Without Sift")
ENDIF(WITH_SIFT)



## Multiple Cores 
IF(APPLE)
  SET(PROCESSOR_COUNT 2)
ELSE(APPLE)
  INCLUDE(CountProcessors)
  SET(NUMBER_OF_CPUS "${PROCESSOR_COUNT}" CACHE STRING "The number of processors to use (default: ${PROCESSOR_COUNT})" )
ENDIF(APPLE)

# OPEN
FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
  OPTION(WITH_OPENMP "Whether to use parallel processing capabilities of OPENMP. ON/OFF" ON)
ENDIF(OPENMP_FOUND)

IF(OPENMP_FOUND AND WITH_OPENMP)
  MESSAGE(STATUS "With OpenMP ")
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMAX_OPENMP_NUM_THREADS=${PROCESSOR_COUNT} -DOPENMP_NUM_THREADS=${PROCESSOR_COUNT} ${OpenMP_CXX_FLAGS} -DOPENMP")
ELSE(OPENMP_FOUND AND WITH_OPENMP)
  MESSAGE(STATUS "Without OpenMP")
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMAX_OPENMP_NUM_THREADS=1 -DOPENMP_NUM_THREADS=1")
ENDIF(OPENMP_FOUND AND WITH_OPENMP)

## TORO 
OPTION(WITH_TORO "Whether to use TORO. ON/OFF" OFF)

IF(WITH_TORO)
  IF(WIN32)
    SET(Subversion_SVN_EXECUTABLE "svn.exe")
  ENDIF(WIN32)
  cmake_minimum_required (VERSION 2.8)
  include(ExternalProject)
  ExternalProject_Add(toro3d
    SVN_REPOSITORY https://www.openslam.org/data/svn/toro/trunk
    SOURCE_DIR "${CMAKE_SOURCE_DIR}/3rdparty/toro"
    CONFIGURE_COMMAND "" 
    BUILD_COMMAND make
    BUILD_IN_SOURCE 1
    INSTALL_COMMAND cp ${CMAKE_SOURCE_DIR}/3rdparty/toro/toro3d ${CMAKE_SOURCE_DIR}/bin/
    )
  MESSAGE(STATUS "With TORO ")
ELSE(WITH_TORO)
  MESSAGE(STATUS "Without TORO")
ENDIF(WITH_TORO)


## HOGMAN 
OPTION(WITH_HOGMAN "Whether to use HOGMAN. ON/OFF" OFF)

IF(WITH_HOGMAN)
  # dependant on libqt4-devi
  find_package( Qt4 REQUIRED)
  # CMake of earlier versions do not have external project capabilities
  cmake_minimum_required (VERSION 2.8)
  include(ExternalProject)
  ExternalProject_Add(hogman
    SVN_REPOSITORY https://svn.openslam.org/data/svn/hog-man/trunk
    SOURCE_DIR "${CMAKE_SOURCE_DIR}/3rdparty/hogman"
    CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR>
    BUILD_COMMAND LD_LIBRARY_PATH=${CMAKE_SOURCE_DIR}/3rdparty/hogman/lib make
    BUILD_IN_SOURCE 1
    INSTALL_COMMAND cp ${CMAKE_SOURCE_DIR}/3rdparty/hogman/bin/hogman3d ${CMAKE_SOURCE_DIR}/bin/ && 
    cp ${CMAKE_SOURCE_DIR}/3rdparty/hogman/lib/libhogman_csparse.so ${CMAKE_SOURCE_DIR}/lib/ &&
    cp ${CMAKE_SOURCE_DIR}/3rdparty/hogman/lib/libhogman_graph_optimizer_hogman.so ${CMAKE_SOURCE_DIR}/lib/ &&
    cp ${CMAKE_SOURCE_DIR}/3rdparty/hogman/lib/libhogman_graph_optimizer.so ${CMAKE_SOURCE_DIR}/lib/ &&
    cp ${CMAKE_SOURCE_DIR}/3rdparty/hogman/lib/libhogman_graph.so ${CMAKE_SOURCE_DIR}/lib/ &&
    cp ${CMAKE_SOURCE_DIR}/3rdparty/hogman/lib/libhogman_graph_viewer.so ${CMAKE_SOURCE_DIR}/lib/ &&
    cp ${CMAKE_SOURCE_DIR}/3rdparty/hogman/lib/libhogman_math.so ${CMAKE_SOURCE_DIR}/lib/ &&
    cp ${CMAKE_SOURCE_DIR}/3rdparty/hogman/lib/libhogman_qglviewer.so ${CMAKE_SOURCE_DIR}/lib/ &&
    cp ${CMAKE_SOURCE_DIR}/3rdparty/hogman/lib/libhogman_stuff.so ${CMAKE_SOURCE_DIR}/lib/
    )
  MESSAGE(STATUS "With HOGMAN: Currently hogman needs to be compiled manually, please make sure hogman3d is somewhere in your PATH")
ELSE(WITH_HOGMAN)
  MESSAGE(STATUS "Without HOGMAN")
ENDIF(WITH_HOGMAN)

OPTION(EXPORT_SHARED_LIBS "Whether to build additional shared libraries for use in other projects. ON/OFF" OFF)

IF(EXPORT_SHARED_LIBS)
  MESSAGE(STATUS "exporting additional libraries")
ELSE(EXPORT_SHARED_LIBS)
  MESSAGE(STATUS "not exporting libraries")
ENDIF(EXPORT_SHARED_LIBS)

IF(WIN32)
  SET(ADDITIONAL_CFLAGS "-O2" CACHE STRING"Additional flags given to the compiler ()" )
  include_directories(${CMAKE_SOURCE_DIR}/windows/)
  link_directories(${CMAKE_SOURCE_DIR}/windows)
  add_library(XGetopt STATIC ${CMAKE_SOURCE_DIR}/windows/XGetopt.cpp)
  SET(CMAKE_STATIC_LIBRARY_SUFFIX "32.lib")
ELSE(WIN32)
  SET(ADDITIONAL_CFLAGS "-O3 -msse3 -Wall -finline-functions -Wno-write-strings -Wno-char-subscripts -Wno-unused-result" CACHE STRING"Additional flags given to the compiler (-O3 -Wall -finline-functions -Wno-write-strings -Wno-unused-result)" )
  # Add include path for OpenGL without GL/-prefix 
  # to avoid the include incompatibility between MACOS 
  # and linux
    FIND_PATH(OPENGL_INC gl.h /usr/include/GL)
    include_directories(${OPENGL_INC})
ENDIF(WIN32)

# Add OpenGL includes for MACOS if needed
IF(APPLE)
  include_directories(/System/Library/Frameworks/GLUT.framework/Headers)
  include_directories(/System/Library/Frameworks/OpenGL.framework/Headers)
ENDIF(APPLE)

SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CFLAGS}")

# Hide CMake variables
SET (CMAKE_INSTALL_PREFIX "/usr/local" CACHE INTERNAL "" FORCE)
SET (CMAKE_BUILD_TYPE "" CACHE INTERNAL "" FORCE)


# Set output directories for libraries and executables
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/obj )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

# Set include and link dirs ... 
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/3rdparty/)
include_directories(${CMAKE_SOURCE_DIR}/3rdparty/glui)
include_directories(${CMAKE_SOURCE_DIR}/3rdparty/wxthings/include/)
include_directories(${CMAKE_SOURCE_DIR}/3rdparty/ann_1.1.1_modified/include)
include_directories(${CMAKE_SOURCE_DIR}/3rdparty/ann_1.1.1_modified/src)
link_directories(${CMAKE_SOURCE_DIR}/obj)
link_directories(${CMAKE_SOURCE_DIR}/lib)

add_subdirectory(3rdparty)
add_subdirectory(src/slam6d)
add_subdirectory(src/show)
#add_subdirectory(src/wxshow)
add_subdirectory(src/grid)
add_subdirectory(src/pmd)
add_subdirectory(src/shapes)
add_subdirectory(src/slam6d/sift)
add_subdirectory(src/scanner)

IF(EXPORT_SHARED_LIBS)
## Compiling a shared library containing all of the project
add_library(slam SHARED src/slam6d/icp6D.cc) 
target_link_libraries(slam scanlib_s ANN_s sparse_s newmat_s) 
ENDIF(EXPORT_SHARED_LIBS)

MESSAGE (STATUS "Build environment is set up!")


# hack to "circumvent" Debug and Release folders that are created under visual studio
# this is why the INSTALL target has to be used in visual studio
IF(MSVC)
  INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/bin/Release/ CONFIGURATIONS Release DESTINATION ${CMAKE_SOURCE_DIR}/windows FILES_MATCHING PATTERN "*.dll" PATTERN "*.exe")

  INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/bin/Debug/ CONFIGURATIONS rebug DESTINATION ${CMAKE_SOURCE_DIR}/windows FILES_MATCHING PATTERN "*.dll" PATTERN "*.exe")
ENDIF(MSVC)
