cmake_minimum_required (VERSION 2.8.2)
project (3DTK)
if(POLICY CMP0025)
  #necessary to build with custom clang on macOS
  cmake_policy(SET CMP0025 NEW)
endif()
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/3rdparty/CMakeModules" "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

include(OptionalBuild3DTK)

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

if(NOT WIN32)
  find_package(CXSparse REQUIRED)
  link_directories(${CXSPARSE_LIBRARY_DIR})
  include_directories(${CXSPARSE_INCLUDE_DIRS})

  find_package(LibZip REQUIRED)
  if(${LIBZIP_VERSION} VERSION_LESS 0.11.2)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DLIBZIP_OLD")
  endif()
  if (NOT CYGWIN)
    find_package(ANN REQUIRED)
  endif()
  if(APPLE)
    include_directories("${LIBZIP_INCLUDE_DIR_ZIPCONF}")
  endif()
endif()

if(WIN32 OR CYGWIN)
  set(ANN_LIBRARIES ann)
endif()

# The newmat package is optional and will be compiled from 3rdparty if it
# wasn't found. We don't use an operating system specific switch because
# there are some Linux distributions (like Fedora) that do not ship newmat.
find_package(Newmat)
if(NOT ${NEWMAT_FOUND})
  include_directories(${PROJECT_SOURCE_DIR}/3rdparty/newmat-10)
endif()

if(APPLE)
  find_package(OpenCV REQUIRED HINTS "/usr/local/opt/opencv3/share/OpenCV/")
else()
  if(WIN32)
    set(OpenCV_STATIC ON)
  endif()
  find_package(OpenCV REQUIRED)
endif()

include_directories(${OpenCV_INCLUDE_DIRS})
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

## FreeGLUT
OPT_DEP(WITH_FREEGLUT "Whether freeglut is available. This enables iterative drawing in show. ON/OFF" ON "")

## Show
OPT_DEP(WITH_SHOW "Whether to build Show. This is the Visualization program of slam6d. ON/OFF" ON "" )

if(WITH_SHOW)
  message(STATUS "With show")
  find_package(PNG REQUIRED)
else()
  #  set (WITH_OCTREE_DISPLAY "ON" CACHE INTERNAL "" FORCE)
  message(STATUS "Without show")
endif()

## WXShow
OPT_DEP(WITH_WXSHOW "Whether to build WXShow. This is the wxwidgets variant of Show. ON/OFF" OFF "")

if(WITH_WXSHOW)
  find_package(wxWidgets COMPONENTS core base gl REQUIRED)
  find_package(PNG REQUIRED)
  # set wxWidgets_wxrc_EXECUTABLE to be ignored in the configuration
  set (wxWidgets_wxrc_EXECUTABLE " " CACHE INTERNAL "" FORCE)
  # wxWidgets include (this will do all the magic to configure everything)
  include( ${wxWidgets_USE_FILE})
  message(STATUS "With wxshow")
else()
  message(STATUS "Without wxshow")
endif()

## QtShow
OPT_DEP(WITH_QTSHOW "Whether to build QtShow. This is the Qt variant of Show. ON/OFF" OFF "")

## GLUT and OpenGL

if(WITH_FREEGLUT OR WITH_SHOW OR WITH_WXSHOW OR WITH_QTSHOW)
  find_package(OpenGL REQUIRED)
  find_package(GLUT REQUIRED)
  find_package(PNG REQUIRED)
  include_directories(${GLUT_INCLUDE_DIR})
endif()

## Shapes
OPT_DEP(WITH_SHAPE_DETECTION "Whether to build shapes and planes executable for detecting planes. ON/OFF" OFF "")
if(WITH_SHAPE_DETECTION)
  message(STATUS "With shape detection")
else()
  message(STATUS "Without shape detection")
endif()

OPT_DEP(WITH_FLOORPLAN "Whether to build floorplan generation from 3D scans.  ON/OFF" OFF "")
if(WITH_FLOORPLAN)
  message(STATUS "With floorplan generation")
else()
  message(STATUS "Without floorplan generation")
endif()


## Interior reconstruction
OPT_DEP(WITH_MODEL "Whether to build model executable for modelling interior environments. ON/OFF" OFF "")

if(WITH_MODEL)
  message(STATUS "With interior reconstruction")
else()
  message(STATUS "Without interior reconstruction")
endif()

## Thermo
OPT_DEP(WITH_THERMO "Whether to build executables for mutual calibration of laser scanner and camera. ON/OFF" OFF "WITH_SHAPE_DETECTION")
if(WITH_THERMO)
  add_subdirectory(3rdparty/cvblob)
  include_directories(${PROJECT_SOURCE_DIR}/3rdparty/cvblob)
  link_directories(${PROJECT_SOURCE_DIR}/3rdparty/cvblob)
  message(STATUS "With thermo")
else()
  message(STATUS "Without thermo")
endif()

## OpenCV non-free
OPT_DEP(WITH_OPENCV_NONFREE "Whether to use non-free (patent encumbered) OpenCV functionalities ON/OFF" OFF "")

## Octree
OPT_DEP(WITH_COMPACT_OCTREE "Whether to use the compact octree display ON/OFF" OFF "")

## Glee?
OPT_DEP(WITH_GLEE "Whether to use OpenGL extensions, requires glee. ON/OFF" OFF "")

## Gridder
OPT_DEP(WITH_GRIDDER "Whether to build the 2DGridder binary ON/OFF" OFF "")

if(WITH_GRIDDER)
  find_package (Boost COMPONENTS system REQUIRED)
  message(STATUS "With 2DGridder")
else()
  message(STATUS "Without 2DGridder")
endif()

## Dynamic VELOSLAM
OPT_DEP(WITH_VELOSLAM "Whether to build the Velodyne data processing (veloslam/veloshow) ON/OFF" OFF "WITH_SHOW")

if(WITH_VELOSLAM)
  message(STATUS "With VELOSLAM")
else()
  message(STATUS "Without VELOSLAM")
endif()

## Home-made Laserscanner
OPT_DEP(WITH_DAVID_3D_SCANNER "Whether to build the David scanner app for homemade laser scanners binary ON/OFF" OFF "")

if(WITH_DAVID_3D_SCANNER)
  message(STATUS "With David scanner")
else()
  message(STATUS "Without David scanner")
endif()

## Tools

OPT_DEP(WITH_TOOLS "Whether to build additional tools like convergence frame_to_graph etc. ON/OFF" OFF "")

if(WITH_TOOLS)
  message(STATUS "With Tools")
  # seems to override previous find_package command resulting in linker errors, either provide ALL necessary libs again or do not use
  #find_package (Boost COMPONENTS serialization graph regex filesystem system thread chrono date_time program_options system REQUIRED)
  #link_directories()
else()
  message(STATUS "Without Tools")
endif()

## Calibration

OPT_DEP(WITH_CALIBRATION "Whether to build calibration tools. ON/OFF" OFF "")

if(WITH_CALIBRATION)
  message(STATUS "With calibration")
else()
  message(STATUS "Without calibration")
endif()

## Segmentation

OPT_DEP(WITH_SEGMENTATION "Whether to build scan segmantion program ON/OFF" OFF "WITH_FBR")

if(WITH_SEGMENTATION)
  message(STATUS "With segmentation")
  find_package (Boost COMPONENTS program_options REQUIRED)
else()
  message(STATUS "Without segmentation")
endif()

## LASlib

OPT_DEP(WITH_LASLIB "Whether to build LASlib based scanio library" OFF WITH_LASLIB)

if(WITH_LASLIB)
  message(STATUS "With LASlib")
else()
  message(STATUS "Without LASlib")
endif()

## Collision detection

OPT_DEP(WITH_COLLISION "Whether to build collision detection tools" OFF "WITH_COLLISION")

if(WITH_COLLISION)
  message(STATUS "With collision detection")
  # as in WITH_TOOLS, seems to override previous find_package command resulting in linker errors, either provide ALL necessary libs again or do not use
  # find_package (Boost COMPONENTS program_options REQUIRED)
else()
  message(STATUS "Without collision detection")
endif()

## People remover

OPT_DEP(WITH_PEOPLEREMOVER "Whether to build people remover tools" OFF "WITH_PEOPLEREMOVER")

if(WITH_PEOPLEREMOVER)
  message(STATUS "With people remover")
else()
  message(STATUS "Without people remover")
endif()

## CUDA accelerated collision detection
OPT_DEP(WITH_CUDA "Whether to build CUDA accelerated collision detection tools" OFF "WITH_CUDA")

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
OPT_DEP(WITH_RIVLIB "Whether the RIEGL rivlib is present ON/OFF" OFF "")

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

## B3D
OPT_DEP(WITH_B3D "Whether the B3D file format is supported. ON/OFF" OFF "")

if(WITH_B3D)
  add_definitions(-DWITH_B3D)
  include_directories(${PROJECT_SOURCE_DIR}/3rdparty/b3dfile)
  message(STATUS "Compiling a scan IO for B3D files")
else()
  message(STATUS "Do NOT compile a scan IO for B3D files")
endif()

## PMD
OPT_DEP(WITH_PMD "Whether to build the PMD tools like grabVideoAnd3D calibrate etc. ON/OFF" OFF "")

if(WITH_PMD)
  find_package(OpenGL REQUIRED)
  message(STATUS "With PMD")
else()
  message(STATUS "Without PMD")
endif()

## FBR
OPT_DEP(WITH_FBR "Whether to compile feature based registration. ON/OFF" OFF "")

if(WITH_FBR)
  message(STATUS "With FBR ")
else()
  message(STATUS "Without FBR")
endif()

# OPEN
find_package(OpenMP)
if(OPENMP_FOUND)
  message(STATUS "fount OpenMP")
  OPT_DEP(WITH_OPENMP "Whether to use parallel processing capabilities of OPENMP. ON/OFF" ON "")
else()
  message(STATUS "can't found OpenMP")
endif()


OPT_DEP(EXPORT_SHARED_LIBS "Whether to build additional shared libraries for use in other projects. ON/OFF" OFF "WITH_SHOW;WITH_FBR")
#OPT_DEP(EXPORT_SHARED_LIBS "Whether to build additional shared libraries for use in other projects. ON/OFF" OFF "WITH_SHOW")

if(EXISTS "$ENV{ROS_ROOT}")
  set(WITH_FBR ON)
  set(EXPORT_SHARED_LIBS ON)
endif()

if(EXPORT_SHARED_LIBS)
  ## Compile a single shared library containing all of 3DTK
  add_library(slam SHARED src/slam6d/icp6D.cc)
  if (WIN32)
    target_link_libraries(slam scan_s ANN_s cxsparse_s newmat_s show_s fbr_s)
  elseif(CYGWIN)
    target_link_libraries(slam scan_s ANN_s cxsparse newmat_s show_s fbr_s)
  else()
    target_link_libraries(slam scan_s ${ANN_LIBRARIES} cxsparse newmat show_s fbr_s)
  endif()
  message(STATUS "exporting additional libraries")
else()
  message(STATUS "not exporting libraries")
endif()


OPT_DEP(WITH_METRICS "Whether to use time metrics. ON/OFF" OFF "")

OPT_DEP(WITH_ADDONS "Whether to download and use addons to 3DTK. ON/OFF" OFF "")
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

OPT_DEP(WITH_TESTING "Whether to build the test framework. ON/OFF" ON "")
if(WITH_TESTING)
  message(STATUS "With testing.")
else()
  message(STATUS "Without testing.")
endif()

## Language Bindings

OPT_DEP(WITH_PYTHON "Whether to build python bindings" OFF "WITH_PYTHON")

if(WITH_PYTHON)
  message(STATUS "With python bindings")
else()
  message(STATUS "Without python bindings")
endif()

## ROS

OPT_DEP(WITH_ROS "Whether or not to build ROS related components" OFF "WITH_ROS")

if(WITH_ROS)
  message(STATUS "With ROS support")
else()
  message(STATUS "Without ROS support")
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
if(APPLE)
  include_directories(${PROJECT_SOURCE_DIR}/3rdparty/cxsparse-4.4.4)
endif()

if(WIN32)
  include_directories(${PROJECT_SOURCE_DIR}/3rdparty/ann_1.1.1_modified/include)
  include_directories(${PROJECT_SOURCE_DIR}/3rdparty/cxsparse-4.4.4)
  include_directories(${PROJECT_SOURCE_DIR}/3rdparty/windows/)
  include_directories(${PROJECT_SOURCE_DIR}/3rdparty/windows/zlib/)
  include_directories(${PROJECT_SOURCE_DIR}/3rdparty/libzip-1.0.1/lib/)
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
elseif(CYGWIN)
  include_directories(${PROJECT_SOURCE_DIR}/3rdparty/ann_1.1.1_modified/include)
else()
  # Add include path for OpenGL without GL/-prefix
  # to avoid the include incompatibility between MACOS
  # and linux
  find_path(OPENGL_INC NAMES gl.h GL/gl.h PATHS /usr/include/GL)
  include_directories(${OPENGL_INC})
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
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/glui-2.36/include/)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/wxthings/include/)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/rply-1.1.4)
link_directories(${PROJECT_SOURCE_DIR}/obj)
link_directories(${PROJECT_SOURCE_DIR}/lib)

add_subdirectory(src/slam6d)
add_subdirectory(src/scanio)
add_subdirectory(src/scanserver)
add_subdirectory(src/segmentation)
add_subdirectory(src/normals)
add_subdirectory(src/veloslam)
add_subdirectory(src/show)
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
add_subdirectory(src/calibration)
add_subdirectory(src/ros)
add_subdirectory(src/tools)
add_subdirectory(3rdparty)
add_subdirectory(bindings)
if(WITH_ADDONS)
  message(STATUS "With 3dtk addons.")
  add_subdirectory(addons)
endif()
if(WITH_TESTING)
  message(STATUS "With 3dtk testing.")
  find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
  enable_testing()
  add_subdirectory(testing)
endif()

# Dummy target with all header files
# This is a hint for some IDEs, such as Qt Creator, to show all headers in the project tree
file(GLOB_RECURSE 3DTK_HEADER_FILES "include/*.h")
add_custom_target(headers SOURCES ${3DTK_HEADER_FILES})

message (STATUS "Build environment is set up!")
