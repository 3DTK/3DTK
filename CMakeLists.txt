cmake_minimum_required (VERSION 2.8.2)
project (3DTK)
if(POLICY CMP0025)
  #necessary to build with custom clang on macOS
  cmake_policy(SET CMP0025 NEW)
endif()
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/3rdparty/CMakeModules" ${CMAKE_MODULE_PATH})

# cmake no longer defines WIN32 on cygwin
set(CMAKE_LEGACY_CYGWIN_WIN32 0) # remove when cmake >= 2.8.4 is required

# Hide CMake variables
set (CMAKE_INSTALL_PREFIX "/usr/local" CACHE INTERNAL "" FORCE)
set (CMAKE_BUILD_TYPE "" CACHE INTERNAL "" FORCE)

# being able to set the output directory to a different one than the default
# (the source directory) is important for platforms like windows, where the
# produced binaries cannot be run if being placed in certain locations (like on
# a network drive)
set(OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}" CACHE PATH "The directory where the output will be placed into lib, obj and bin subdirectories (default: ${PROJECT_SOURCE_DIR})" )

# Set output directories for libraries and executables
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_DIRECTORY}/lib )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${OUTPUT_DIRECTORY}/obj )
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIRECTORY}/bin )
# Set output directories for multi-config builds (like with MSVC)
foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
  string( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )
  set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${OUTPUT_DIRECTORY}/lib )
  set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${OUTPUT_DIRECTORY}/obj )
  set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${OUTPUT_DIRECTORY}/bin )
endforeach( OUTPUTCONFIG CMAKE_CONFIGURATION_TYPES )

include(CheckSymbolExists)
check_symbol_exists(mmap sys/mman.h HAVE_MMAP)
check_symbol_exists(mkstemp stdlib.h HAVE_MKSTEMP)
set(CMAKE_REQUIRED_DEFINITIONS -D_GNU_SOURCE)
check_symbol_exists(fallocate fcntl.h HAVE_FALLOCATE)
set(CMAKE_REQUIRED_DEFINITIONS)
if(HAVE_MMAP AND HAVE_MKSTEMP AND HAVE_FALLOCATE)
  add_definitions(-DWITH_MMAP_SCAN)
endif()

#include_directories(OPENGL_INCLUDE_DIR)
if(WIN32)
  set(Boost_USE_STATIC_LIBS TRUE)
else()
  set(Boost_USE_STATIC_LIBS FALSE)
endif()

set(Boost_ADDITIONAL_VERSIONS "1.42" "1.42.0" "1.44" "1.44.0" "1.45.0" "1.45" "1.46" "1.46.1" "1.47.0" "1.47" "1.48" "1.50" "1.52" "1.53" "1.55" "1.56")
if(WIN32)
  # for some unknown reason no one variant works on all windows platforms
  set(Boost_DEBUG 1)
  find_package( Boost COMPONENTS serialization graph regex filesystem system thread chrono date_time program_options system REQUIRED)
else()
  find_package( Boost COMPONENTS serialization graph regex filesystem system thread date_time program_options system REQUIRED)
endif()

if(Boost_FOUND)
  link_directories(${BOOST_LIBRARY_DIRS})
  include_directories(${Boost_INCLUDE_DIRS})
  add_definitions(${Boost_LIB_DIAGNOSTIC_DEFINITIONS})
endif()

find_package(CXSparse QUIET)
if (CXSparse_FOUND)
  set(CXSPARSE_LIBRARIES_SHARED ${CXSPARSE_LIBRARIES})
  set(CXSPARSE_LIBRARIES_STATIC ${CXSPARSE_LIBRARIES})
else()
  add_subdirectory("3rdparty/cxsparse")
  set(CXSPARSE_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/3rdparty/cxsparse/cxsparse-4.4.4/suitesparse")
  set(CXSPARSE_LIBRARIES_SHARED cxsparse_shared)
  set(CXSPARSE_LIBRARIES_STATIC cxsparse_static)
endif()

find_package(ANN)
if (ANN_FOUND)
  set(ANN_LIBRARIES_SHARED ${ANN_LIBRARIES})
  set(ANN_LIBRARIES_STATIC ${ANN_LIBRARIES})
else()
  add_subdirectory("3rdparty/ann")
  set(ANN_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/3rdparty/ann/ann_1.1.1_modified/include")
  set(ANN_LIBRARIES_SHARED ann_shared)
  set(ANN_LIBRARIES_STATIC ann_static)
endif()

find_package(Newmat)
if(NEWMAT_FOUND)
  set(NEWMAT_LIBRARIES_SHARED ${NEWMAT_LIBRARIES})
  set(NEWMAT_LIBRARIES_STATIC ${NEWMAT_LIBRARIES})
else()
  add_subdirectory("3rdparty/newmat")
  set(NEWMAT_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/3rdparty/newmat/newmat-10")
  set(NEWMAT_LIBRARIES_SHARED newmat_shared)
  set(NEWMAT_LIBRARIES_STATIC newmat_static)
endif()

if(APPLE)
  find_package(OpenCV REQUIRED HINTS "/usr/local/opt/opencv3/share/OpenCV/")
else()
  if(WIN32)
    set(OpenCV_STATIC ON)
  endif()
endif()

if(EXISTS "${OpenCV_DIR}/OpenCVConfig.cmake")
  include("${OpenCV_DIR}/OpenCVConfig.cmake")
  set(ADDITIONAL_OPENCV_FLAGS
          "-DCV_MINOR_VERSION=${OpenCV_VERSION_MINOR} -DCV_MAJOR_VERSION=${OpenCV_VERSION_MAJOR}"
          CACHE STRING "OpenCV Version Defines)"
          )
  ## Include the standard CMake script
else()
  set(ADDITIONAL_OPENCV_FLAGS
          ""
          CACHE STRING "OpenCV Version Defines (BLUB)"
          )
endif()
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_OPENCV_FLAGS}")

#################################################
# Declare Options and modify build accordingly ##
#################################################

# cvblob is needed for thermo
find_package(OpenCV QUIET)
if (OpenCV_FOUND)
  include_directories(${OpenCV_INCLUDE_DIRS})
  add_subdirectory(3rdparty/cvblob)
  link_directories(${PROJECT_SOURCE_DIR}/3rdparty/cvblob)
else()
  message(STATUS "Not building cvblob because WITH_OPENCV is OFF")
endif()

option(WITH_CGAL "Compile with CGAL support" ON)
option(WITH_LIBZIP "Compile with libzip support" ON)
option(WITH_OPENGL "Compile with OpenGL support" ON)
option(WITH_OPENCV "Compile with OpenCV support" ON)
option(WITH_QT "Compile tools relying on QT (qtshow)" ON)
option(WITH_GLFW "Compile with GLFW support" ON)
option(WITH_FTGL "Compile with FTGL support" ON)
option(WITH_XMLRPC "Compile with XMLRPC support" ON)
option(WITH_EIGEN3 "Compile with eigen3 support" ON)
option(WITH_LIBCONFIG "Compile with libconfig support" ON)
option(WITH_ROS "Compile with ROS support" OFF)
option(WITH_PYTHON "Compile Python bindings" ON)
option(WITH_WXWIDGETS "Compile with wxwidgets support" ON)
option(WITH_OPENCV_NONFREE "Whether to use non-free (patent encumbered) OpenCV functionalities" OFF)
option(WITH_COMPACT_OCTREE "Whether to use the compact octree display ON/OFF" OFF)
option(WITH_GLEE "Whether to use OpenGL extensions, requires glee. ON/OFF" OFF)
option(WITH_LASLIB "Whether to build LASlib based scanio library" ON)

## CUDA accelerated collision detection
option(WITH_CUDA "Whether to build CUDA accelerated collision detection tools" OFF)
if(WITH_CUDA)
  find_package(CUDA REQUIRED)
  include_directories(${FOUND_CUDA_NVCC_INCLUDE})

  #owerrite users input - need to be fixed
  #select all NVIDIA GPU arch which support unified memory (CUDA toolkit >= 6.0) and arch>=30
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_30,code=sm_30;-gencode arch=compute_35,code=sm_35;-gencode arch=compute_50,code=sm_50;-gencode arch=compute_52,code=sm_52)

  message("CUDA_NVCC_FLAGS = ${CUDA_NVCC_FLAGS}")

  message(STATUS "With CUDA accelerated collision detection")
else()
  message(STATUS "Without CUDA accelerated collision detection")
endif()


## RivLib
option(WITH_RIVLIB "Whether the RIEGL rivlib is present ON/OFF" OFF)

if(WITH_RIVLIB)
  message(STATUS "Compiling a scan IO for RXP files")

  set(RIEGL_DIR ${PROJECT_SOURCE_DIR}/3rdparty/riegl/)

  if(UNIX)
    set(RiVLib_USE_STATIC_RUNTIME ON)
  endif()

  find_package(RiVLib QUIET COMPONENTS scanlib HINTS "${RIEGL_DIR}/cmake/")

  if(${RiVLib_FOUND})
    include_directories(${RiVLib_INCLUDE_DIRS})
  else()
    # TODO: Remove this if nobody is using the old RiVLib anymore.
    #       Change QUIET to REQUIRED in the find_package call for RiVLib above.

    message(STATUS "Cannot find current RiVLib. Trying to build scan IO for RXP files with old scanlib.")

    add_definitions(-DWITH_OLD_RIVLIB)

    find_package(LibXml2 REQUIRED)
    include_directories(${LIBXML2_INCLUDE_DIR})

    include_directories(${PROJECT_SOURCE_DIR}/3rdparty)
    if(WIN32)
      set(RiVLib_SCANLIB_LIBRARY ${RIEGL_DIR}libscanlib-mt.lib ${RIEGL_DIR}libctrllib-mt.lib ${RIEGL_DIR}libboost_system-mt-1_43_0-vns.lib ${LIBXML2_LIBRARIES})
    else()
      set(RiVLib_SCANLIB_LIBRARY ${RIEGL_DIR}libscanlib-mt-s.a ${RIEGL_DIR}libctrllib-mt-s.a ${RIEGL_DIR}libboost_system-mt-s-1_43_0-vns.a pthread ${LIBXML2_LIBRARIES})
    endif()
  endif()
else()
  message(STATUS "Do NOT compile a scan IO for RXP")
endif()

option(WITH_OPENMP "Whether to use parallel processing capabilities of OPENMP. ON/OFF" ON)
if (WITH_OPENMP)
  find_package(OpenMP REQUIRED)
endif()

option(WITH_METRICS "Whether to use time metrics. ON/OFF" OFF)

option(WITH_ADDONS "Whether to download and use addons to 3DTK. ON/OFF" OFF)
if(WITH_ADDONS)
  message(STATUS "Compiling addons directory")
  #execute_process(COMMAND svn co https://robotik.informatik.uni-wuerzburg.de/slam6dprivate/trunk/addons addons
  #                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  #                OUTPUT_QUIET
  #                )
  message(STATUS "With addons.")
else()
  message(STATUS "Without addons.")
endif()

#################################################
# OPERATING SYSTEM SPECIFIC BEHAVIOUR          ##
#################################################

## Special treatment for system specifics
if(APPLE)
  add_definitions(-Dfopen64=fopen)
endif()

## Multiple Cores
include(ProcessorCount)
ProcessorCount(PROCESSOR_COUNT)
if(NOT PROCESSOR_COUNT EQUAL 0)
  set(NUMBER_OF_CPUS "${PROCESSOR_COUNT}" CACHE STRING "The number of processors to use (default: ${PROCESSOR_COUNT})" )
else()
  set(NUMBER_OF_CPUS "1" CACHE STRING "The number of processors to use (default: 1)" )
endif()

# OPENMP_NUM_THREADS
if(OPENMP_FOUND AND WITH_OPENMP)
  message(STATUS "With OpenMP ")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMAX_OPENMP_NUM_THREADS=${NUMBER_OF_CPUS} -DOPENMP_NUM_THREADS=${NUMBER_OF_CPUS} ${OpenMP_CXX_FLAGS} -DOPENMP")
else()
  message(STATUS "Without OpenMP")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMAX_OPENMP_NUM_THREADS=1 -DOPENMP_NUM_THREADS=1")
endif()

# 3rdparty
if(WIN32)
  include_directories(${PROJECT_SOURCE_DIR}/3rdparty/windows/)
  include_directories(${PROJECT_SOURCE_DIR}/3rdparty/windows/zlib/)
  link_directories(${PROJECT_SOURCE_DIR}/3rdparty/windows)
  link_directories(${BOOST_LIBRARYDIR})
  if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    link_directories(${PROJECT_SOURCE_DIR}/3rdparty/windows/freeglut/lib/x64/)
  else ()
    link_directories(${PROJECT_SOURCE_DIR}/3rdparty/windows/freeglut/lib/)
  endif ()
  add_library(XGetopt STATIC ${PROJECT_SOURCE_DIR}/3rdparty/windows/XGetopt.cpp)
  set(CMAKE_STATIC_LIBRARY_SUFFIX "32.lib")
  # to be able to use numeric_limits<int>::max() and friends on windows
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNOMINMAX")
else()
  if (WITH_OPENGL)
    # Add include path for OpenGL without GL/-prefix
    # to avoid the include incompatibility between MACOS
    # and linux
    find_path(OPENGL_INC NAMES gl.h GL/gl.h PATHS /usr/include/GL)
    if (${OPENGL_INC})
      include_directories(${OPENGL_INC})
    else()
      message(STATUS "gl.h not found")
    endif()
  endif()
endif()

if (${CMAKE_CXX_COMPILER_ID} STREQUAL Clang)
  # using Clang
  set(ADDITIONAL_CFLAGS "-O3 -std=c++0x -msse3 -Wall -Wno-write-strings -Wno-char-subscripts -Wno-unused-result" CACHE STRING "Additional flags given to the compiler (-O3 -Wall -Wno-write-strings)" )
elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL GNU)
  # using GCC
  set(ADDITIONAL_CFLAGS "-O3 -std=c++0x -msse3 -Wall -finline-functions -Wno-unused-but-set-variable -Wno-write-strings -Wno-char-subscripts -Wno-unused-result" CACHE STRING "Additional flags given to the compiler (-O3 -Wall -finline-functions -Wno-write-strings)" )
elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL Intel)
  # using Intel C++
elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL MSVC)
  # using Visual Studio C++
  set(ADDITIONAL_CFLAGS "-O2" CACHE STRING "Additional flags given to the compiler ( -O2)" )
endif()

# Add OpenGL includes for MACOS if needed
# The OSX OpenGL frameworks natively supports freeglut extensions
if(APPLE)
  include_directories(/System/Library/Frameworks/GLUT.framework/Headers)
  include_directories(/System/Library/Frameworks/OpenGL.framework/Headers)
endif()

# hack to "circumvent" Debug and Release folders that are created under visual studio
# this is why the INSTALL target has to be used in visual studio
if(MSVC)
  install(DIRECTORY ${PROJECT_SOURCE_DIR}/bin/Release/ CONFIGURATIONS Release DESTINATION ${PROJECT_SOURCE_DIR}/windows FILES_MATCHING PATTERN "*.dll" PATTERN "*.exe")
  if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/3rdparty/windows/x64/ CONFIGURATIONS Release DESTINATION ${PROJECT_SOURCE_DIR}/windows FILES_MATCHING PATTERN "*.dll" PATTERN "*.exe")
  else()
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/3rdparty/windows/ CONFIGURATIONS Release DESTINATION ${PROJECT_SOURCE_DIR}/windows FILES_MATCHING PATTERN "*.dll" PATTERN "*.exe")
  endif()

  install(DIRECTORY ${PROJECT_SOURCE_DIR}/bin/Debug/ CONFIGURATIONS Debug DESTINATION ${PROJECT_SOURCE_DIR}/windows FILES_MATCHING PATTERN "*.dll" PATTERN "*.exe")
  if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/3rdparty/windows/x64/ CONFIGURATIONS Debug DESTINATION ${PROJECT_SOURCE_DIR}/windows FILES_MATCHING PATTERN "*.dll" PATTERN "*.exe")
  else()
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/3rdparty/windows/ CONFIGURATIONS Debug DESTINATION ${PROJECT_SOURCE_DIR}/windows FILES_MATCHING PATTERN "*.dll" PATTERN "*.exe")
  endif()

  if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    file(COPY ${PROJECT_SOURCE_DIR}/3rdparty/windows/freeglut/bin/x64/freeglut.dll DESTINATION ${OUTPUT_DIRECTORY}/bin)
  else()
    file(COPY ${PROJECT_SOURCE_DIR}/3rdparty/windows/freeglut/bin/freeglut.dll DESTINATION ${OUTPUT_DIRECTORY}/bin)
  endif()
endif()

#################################################
# Robot Operating System (ROS) Integration     ##
#################################################

if( catkin_FOUND )
  catkin_package(
    INCLUDE_DIRS include addons/include
  )
endif()

#################################################
# GENERAL PROJECT SETTINGS                     ##
#################################################

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CFLAGS}")

if(APPLE)
  set(CMAKE_MACOSX_RPATH 1)
endif()

# Set include and link dirs ...
include_directories(${PROJECT_SOURCE_DIR}/include)
#include_directories(${PROJECT_SOURCE_DIR}/3rdparty/wxthings/include/)
link_directories(${PROJECT_SOURCE_DIR}/obj)
link_directories(${PROJECT_SOURCE_DIR}/lib)

# src/show must be added first because it defines the cache variables
# SHOW_LIBS_* which are used by others
add_subdirectory(src/show)
add_subdirectory(src/slam6d)
add_subdirectory(src/scanio)
add_subdirectory(src/scanserver)
add_subdirectory(src/segmentation)
add_subdirectory(src/normals)
add_subdirectory(src/veloslam)
add_subdirectory(src/qtshow)
add_subdirectory(src/grid)
add_subdirectory(src/pmd)
add_subdirectory(src/shapes)
add_subdirectory(src/floorplan)
add_subdirectory(src/thermo)
add_subdirectory(src/slam6d/fbr)
add_subdirectory(src/scanner)
add_subdirectory(src/model)
add_subdirectory(src/collision)
add_subdirectory(src/peopleremover)
add_subdirectory(src/spherical_quadtree)
add_subdirectory(src/cuda)
add_subdirectory(src/ros)
add_subdirectory(src/tools)
# 3rdparty must come before src/calibration because it sets
# APRILTAG_INCLUDE_DIRS
add_subdirectory(3rdparty)
add_subdirectory(src/calibration)
add_subdirectory(bindings)
if(WITH_ADDONS)
  message(STATUS "With 3dtk addons.")
  add_subdirectory(addons)
endif()

find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
enable_testing()
add_subdirectory(testing)

# Dummy target with all header files
# This is a hint for some IDEs, such as Qt Creator, to show all headers in the project tree
file(GLOB_RECURSE 3DTK_HEADER_FILES "include/*.h")
add_custom_target(headers SOURCES ${3DTK_HEADER_FILES})

message (STATUS "Build environment is set up!")
