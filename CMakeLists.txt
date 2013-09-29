cmake_minimum_required (VERSION 2.8.2)
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/3rdparty/CMakeModules" ${CMAKE_MODULE_PATH})
project (3DTK)

# Hide CMake variables
SET (CMAKE_INSTALL_PREFIX "/usr/local" CACHE INTERNAL "" FORCE)
SET (CMAKE_BUILD_TYPE "" CACHE INTERNAL "" FORCE)


# Set output directories for libraries and executables
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/obj )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin )

#include_directories(OPENGL_INCLUDE_DIR)
IF(WIN32)
  set(Boost_USE_STATIC_LIBS TRUE)
ELSE(WIN32)
  set(Boost_USE_STATIC_LIBS FALSE)
ENDIF(WIN32)

SET(Boost_ADDITIONAL_VERSIONS "1.42" "1.42.0" "1.44" "1.44.0" "1.45.0" "1.45" "1.46" "1.46.1" "1.47.0" "1.47" "1.48" "1.49" "1.50" "1.52" "1.53")
IF(WIN32)
  # for some unknown reason no one variant works on all windows platforms
  set(Boost_DEBUG 1)  
  find_package( Boost COMPONENTS serialization graph regex filesystem system thread chrono date_time program_options REQUIRED)
ELSE(WIN32)
  find_package( Boost COMPONENTS serialization graph regex filesystem system thread date_time program_options REQUIRED)
ENDIF(WIN32)

if(Boost_FOUND)
  link_directories(${BOOST_LIBRARY_DIRS})
  include_directories(${Boost_INCLUDE_DIRS})
  add_definitions(${Boost_LIB_DIAGNOSTIC_DEFINITIONS})
endif()


FIND_PACKAGE(OpenCV REQUIRED)
include("3rdparty/CMakeModules/OpenCV.cmake")
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_OPENCV_FLAGS}")

#################################################
# Declare Options and modify build accordingly ##
#################################################


FUNCTION(ENFORCE_OPTION_DEP_3DTK option VALUE)
  SET (${option} "${VALUE}" CACHE BOOL "${${option}_DESCRIPTION}" FORCE) # this option set to VALUE as advised
  
  #now make sure other dependencies are also true
  FOREACH(d ${${option}_DEPENDENCIES}) # look through all my dependencies
    STRING(REGEX REPLACE " +" ";" CMAKE_DEPENDENT_OPTION_DEP "${d}")

    # check for a not in front
    STRING(STRIP "${CMAKE_DEPENDENT_OPTION_DEP}" CMAKE_DEPENDENT_OPTION_DEP)
    STRING(SUBSTRING "${CMAKE_DEPENDENT_OPTION_DEP}" 0 3 CMAKE_DEPENDENT_OPTION_DEP_3)
    STRING(TOUPPER "${CMAKE_DEPENDENT_OPTION_DEP_3}" CMAKE_DEPENDENT_OPTION_DEP_3)
    STRING(COMPARE EQUAL "${CMAKE_DEPENDENT_OPTION_DEP_3}" "NOT" CMAKE_DEPENDENT_OPTION_DEP_NOT)
    #STRING(REPLACE "NOT " "" CMAKE_DEPENDENT_OPTION_DEP "${d}")
    IF(CMAKE_DEPENDENT_OPTION_DEP_NOT) # we found a NOT
      STRING(REPLACE "NOT;" "" CMAKE_DEPENDENT_OPTION_DEP "${CMAKE_DEPENDENT_OPTION_DEP}")
      IF(${CMAKE_DEPENDENT_OPTION_DEP})  # not met, make sure it is
        ENFORCE_OPTION_DEP_3DTK(${CMAKE_DEPENDENT_OPTION_DEP} OFF)
      ELSE(${CMAKE_DEPENDENT_OPTION_DEP}) # dependency is met
      ENDIF(${CMAKE_DEPENDENT_OPTION_DEP})
    ELSE(CMAKE_DEPENDENT_OPTION_DEP_NOT)
      IF(${CMAKE_DEPENDENT_OPTION_DEP})  # dependency is met
      ELSE(${CMAKE_DEPENDENT_OPTION_DEP}) # not met, make sure it is
        ENFORCE_OPTION_DEP_3DTK(${CMAKE_DEPENDENT_OPTION_DEP} ON)
      ENDIF(${CMAKE_DEPENDENT_OPTION_DEP})
    ENDIF(CMAKE_DEPENDENT_OPTION_DEP_NOT)
  ENDFOREACH(d)
  
ENDFUNCTION(ENFORCE_OPTION_DEP_3DTK)

MACRO(OPT_DEP option doc default depends)
  OPTION(${option} "${doc}" "${default}")
  SET(${option} "${${option}}" CACHE BOOL "${doc}" FORCE)
  SET(${option}_DEPENDENCIES "${depends}" CACHE INTERNAL "" FORCE)
  SET(${option}_DESCRIPTION "${doc}" CACHE INTERNAL "" FORCE)

  IF (${option})
    #MESSAGE(STATUS "Yes ${option} is true")
    #  MESSAGE("FOREACH d in ${depends}")

    FOREACH(d ${depends})
      STRING(REGEX REPLACE " +" ";" CMAKE_DEPENDENT_OPTION_DEP "${d}")

      # check for a not in front
      STRING(STRIP "${CMAKE_DEPENDENT_OPTION_DEP}" CMAKE_DEPENDENT_OPTION_DEP)
      STRING(SUBSTRING "${CMAKE_DEPENDENT_OPTION_DEP}" 0 3 CMAKE_DEPENDENT_OPTION_DEP_3)
      STRING(TOUPPER "${CMAKE_DEPENDENT_OPTION_DEP_3}" CMAKE_DEPENDENT_OPTION_DEP_3)
      STRING(COMPARE EQUAL "${CMAKE_DEPENDENT_OPTION_DEP_3}" "NOT" CMAKE_DEPENDENT_OPTION_DEP_NOT)
      IF(CMAKE_DEPENDENT_OPTION_DEP_NOT) # we found a NOT
        STRING(REPLACE "NOT;" "" CMAKE_DEPENDENT_OPTION_DEP "${CMAKE_DEPENDENT_OPTION_DEP}")
        IF(${CMAKE_DEPENDENT_OPTION_DEP})  # not met, make sure it is
          ENFORCE_OPTION_DEP_3DTK(${CMAKE_DEPENDENT_OPTION_DEP} OFF)
        ELSE(${CMAKE_DEPENDENT_OPTION_DEP}) # dependency is met
        ENDIF(${CMAKE_DEPENDENT_OPTION_DEP})
      ELSE(CMAKE_DEPENDENT_OPTION_DEP_NOT)
        IF(${CMAKE_DEPENDENT_OPTION_DEP})  # dependency is met
        ELSE(${CMAKE_DEPENDENT_OPTION_DEP}) # not met, make sure it is
          ENFORCE_OPTION_DEP_3DTK(${CMAKE_DEPENDENT_OPTION_DEP} ON)
        ENDIF(${CMAKE_DEPENDENT_OPTION_DEP})
      ENDIF(CMAKE_DEPENDENT_OPTION_DEP_NOT)
    ENDFOREACH(d)

  ENDIF(${option})
ENDMACRO(OPT_DEP)



## FreeGLUT
OPT_DEP(WITH_FREEGLUT "Whether freeglut is available. This enables iterative drawing in show. ON/OFF" ON "")

IF(WITH_FREEGLUT)
  MESSAGE(STATUS "With freeglut")
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_FREEGLUT")
ELSE(WITH_FREEGLUT)
  MESSAGE(STATUS "Without freeglut")
ENDIF(WITH_FREEGLUT)

## Show
OPT_DEP(WITH_SHOW "Whether to build Show. This is the Visualization program of slam6d. ON/OFF" ON "" )

IF(WITH_SHOW)
  FIND_PACKAGE(OpenGL REQUIRED)
  FIND_PACKAGE(GLUT REQUIRED)
  MESSAGE(STATUS "With show")
ELSE(WITH_SHOW)
  #  SET (WITH_OCTREE_DISPLAY "ON" CACHE INTERNAL "" FORCE)
  MESSAGE(STATUS "Without show")
ENDIF(WITH_SHOW)

## WXShow
OPT_DEP(WITH_WXSHOW "Whether to build WXShow. This is the wxwidgets variant of Show. ON/OFF" OFF "")

IF(WITH_WXSHOW)
  FIND_PACKAGE(OpenGL REQUIRED)
  FIND_PACKAGE(GLUT REQUIRED)
  find_package(wxWidgets COMPONENTS core base gl REQUIRED)
  # set wxWidgets_wxrc_EXECUTABLE to be ignored in the configuration
  SET (wxWidgets_wxrc_EXECUTABLE " " CACHE INTERNAL "" FORCE)
  # wxWidgets include (this will do all the magic to configure everything)
  include( ${wxWidgets_USE_FILE})
  MESSAGE(STATUS "With wxshow")
ELSE(WITH_WXSHOW)
  MESSAGE(STATUS "Without wxshow")
ENDIF(WITH_WXSHOW)

## Shapes
OPT_DEP(WITH_SHAPE_DETECTION "Whether to build shapes and planes executable for detecting planes. ON/OFF" OFF "")
IF(WITH_SHAPE_DETECTION)
  MESSAGE(STATUS "With shape detection")
ELSE(WITH_SHAPE_DETECTION)
  MESSAGE(STATUS "Without shape detection")
ENDIF(WITH_SHAPE_DETECTION)

## Interior reconstruction
OPT_DEP(WITH_MODEL "Whether to build model executable for modelling interior environments. ON/OFF" OFF "")

if(WITH_MODEL)
  message(STATUS "With interior reconstruction")
else(WITH_MODEL)
  message(STATUS "Without interior reconstruction")
endif(WITH_MODEL)

## Thermo
OPT_DEP(WITH_THERMO "Whether to build executables for mutual calibration of laser scanner and camera. ON/OFF" OFF "WITH_SHAPE_DETECTION")
IF(WITH_THERMO)
  add_subdirectory(3rdparty/cvblob)
  include_directories(${CMAKE_SOURCE_DIR}/3rdparty/cvblob)
  link_directories(${CMAKE_SOURCE_DIR}/3rdparty/cvblob)
  MESSAGE(STATUS "With thermo")
ELSE(WITH_THERMO)
  MESSAGE(STATUS "Without thermo")
ENDIF(WITH_THERMO)


## Octree
OPT_DEP(WITH_COMPACT_OCTREE "Whether to use the compact octree display ON/OFF" OFF "")

IF(WITH_COMPACT_OCTREE)
  MESSAGE(STATUS "Using compact octrees")
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_COMPACT_TREE")
ELSE(WITH_COMPACT_OCTREE)
  MESSAGE(STATUS "Not using compact octreees: Warning uses more memory")
ENDIF(WITH_COMPACT_OCTREE)

## Glee?
OPT_DEP(WITH_GLEE "Whether to use OpenGL extensions, requires glee. ON/OFF" OFF "")

IF(WITH_GLEE)
  MESSAGE(STATUS "Using opengl extensions")
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_GLEE")
ELSE(WITH_GLEE)
  MESSAGE(STATUS "Not using opengl extensions")
ENDIF(WITH_GLEE)

## Gridder
OPT_DEP(WITH_GRIDDER "Whether to build the 2DGridder binary ON/OFF" OFF "")

IF(WITH_GRIDDER)
  MESSAGE(STATUS "With 2DGridder")
ELSE(WITH_GRIDDER)
  MESSAGE(STATUS "Without 2DGridder")
ENDIF(WITH_GRIDDER)

## Dynamic VELOSLAM
OPT_DEP(WITH_VELOSLAM "Whether to build the Velodyne data processing (veloslam/veloshow) ON/OFF" OFF "WITH_SHOW")

IF(WITH_VELOSLAM)
  MESSAGE(STATUS "With VELOSLAM")
ELSE(WITH_VELOSLAM)
  MESSAGE(STATUS "Without VELOSLAM")
ENDIF(WITH_VELOSLAM)

## Home-made Laserscanner
OPT_DEP(WITH_DAVID_3D_SCANNER "Whether to build the David scanner app for homemade laser scanners binary ON/OFF" OFF "")

IF(WITH_DAVID_3D_SCANNER)
  MESSAGE(STATUS "With David scanner")
ELSE(WITH_DAVID_3D_SCANNER)
  MESSAGE(STATUS "Without David scanner")
ENDIF(WITH_DAVID_3D_SCANNER)

## Tools

OPT_DEP(WITH_TOOLS "Whether to build additional tools like convergence frame_to_graph etc. ON/OFF" OFF "WITH_FBR")

IF(WITH_TOOLS)
  MESSAGE(STATUS "With Tools")
  find_package (Boost COMPONENTS program_options REQUIRED)
ELSE(WITH_TOOLS)
  MESSAGE(STATUS "Without Tools")
ENDIF(WITH_TOOLS)

## Segmentation

OPT_DEP(WITH_SEGMENTATION "Whether to build scan segmantion program ON/OFF" OFF "WITH_FBR")

IF(WITH_SEGMENTATION)
  MESSAGE(STATUS "With segmentation")
  find_package (Boost COMPONENTS program_options REQUIRED)
ELSE(WITH_SEGMENTATION)
  MESSAGE(STATUS "Without segmentation")
ENDIF(WITH_SEGMENTATION)

## Collision detection

OPT_DEP(WITH_COLLISION "Whether to build collision detection tools" OFF "WITH_COLLISION")

IF(WITH_COLLISION)
  MESSAGE(STATUS "With collision detection")
  find_package (Boost COMPONENTS program_options REQUIRED)
ELSE(WITH_COLLISION)
  MESSAGE(STATUS "Without collision detection")
ENDIF(WITH_COLLISION)

## RivLib
OPT_DEP(WITH_RIVLIB "Whether the RIEGL rivlib is present ON/OFF" OFF "")

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

## PMD
OPT_DEP(WITH_PMD "Whether to build the PMD tools like grabVideoAnd3D calibrate etc. ON/OFF" OFF "")

IF(WITH_PMD)
  FIND_PACKAGE(OpenGL REQUIRED)
  MESSAGE(STATUS "With PMD")
ELSE(WITH_PMD)
  MESSAGE(STATUS "Without PMD")
ENDIF(WITH_PMD)

## FBR
OPT_DEP(WITH_FBR "Whether to compile feature based registration. ON/OFF" OFF "")

IF(WITH_FBR)
  MESSAGE(STATUS "With FBR ")
ELSE(WITH_FBR)
  MESSAGE(STATUS "Without FBR")
ENDIF(WITH_FBR)

# OPEN
FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
  OPT_DEP(WITH_OPENMP "Whether to use parallel processing capabilities of OPENMP. ON/OFF" ON "")
ENDIF(OPENMP_FOUND)

IF(OPENMP_FOUND AND WITH_OPENMP)
  MESSAGE(STATUS "With OpenMP ")
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMAX_OPENMP_NUM_THREADS=${NUMBER_OF_CPUS} -DOPENMP_NUM_THREADS=${NUMBER_OF_CPUS} ${OpenMP_CXX_FLAGS} -DOPENMP")
ELSE(OPENMP_FOUND AND WITH_OPENMP)
  MESSAGE(STATUS "Without OpenMP")
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMAX_OPENMP_NUM_THREADS=1 -DOPENMP_NUM_THREADS=1")
ENDIF(OPENMP_FOUND AND WITH_OPENMP)


OPT_DEP(EXPORT_SHARED_LIBS "Whether to build additional shared libraries for use in other projects. ON/OFF" OFF "WITH_SHOW;WITH_FBR")

IF(EXPORT_SHARED_LIBS)
  ## Compile a single shared library containing all of 3DTK
  add_library(slam SHARED src/slam6d/icp6D.cc)
  target_link_libraries(slam scan_s ANN_s sparse_s newmat_s show_s fbr_s)
  MESSAGE(STATUS "exporting additional libraries")
ELSE(EXPORT_SHARED_LIBS)
  MESSAGE(STATUS "not exporting libraries")
ENDIF(EXPORT_SHARED_LIBS)


OPT_DEP(WITH_METRICS "Whether to use metrics in slam6d. ON/OFF" OFF "")

IF(WITH_METRICS)
  MESSAGE(STATUS "With metrics in slam6d.")
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_METRICS")
ELSE(WITH_METRICS)
  MESSAGE(STATUS "Without metrics in slam6d.")
ENDIF(WITH_METRICS)

OPT_DEP(WITH_ADDONS "Whether to download and use addons to 3DTK. ON/OFF" OFF "")
IF(WITH_ADDONS)
  execute_process(COMMAND svn co https://robotik.informatik.uni-wuerzburg.de/slam6dprivate/trunk/addons addons 
                  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                  OUTPUT_QUIET
                  ) 
  MESSAGE(STATUS "With addons.")
ELSE(WITH_ADDONS)
  MESSAGE(STATUS "Without addons.")
ENDIF(WITH_ADDONS)

#################################################
# OPERATING SYSTEM SPECIFIC BEHAVIOUR          ##
#################################################

## Special treatment for system specifics
IF(APPLE)
add_definitions(-Dfopen64=fopen)
ENDIF(APPLE)

## Multiple Cores
IF(APPLE)
  SET(PROCESSOR_COUNT 2)
ELSE(APPLE)
  INCLUDE(CountProcessors)
  SET(NUMBER_OF_CPUS "${PROCESSOR_COUNT}" CACHE STRING "The number of processors to use (default: ${PROCESSOR_COUNT})" )
ENDIF(APPLE)

IF(WIN32)
  SET(ADDITIONAL_CFLAGS "-O2" CACHE STRING "Additional flags given to the compiler ( -O2)" )
  include_directories(${CMAKE_SOURCE_DIR}/3rdparty/windows/)
   include_directories(${CMAKE_SOURCE_DIR}/3rdparty/windows/GL)
  link_directories(${CMAKE_SOURCE_DIR}/3rdparty/windows)
  link_directories(${CMAKE_SOURCE_DIR}/3rdparty/windows/x64)
  add_library(XGetopt STATIC ${CMAKE_SOURCE_DIR}/3rdparty/windows/XGetopt.cpp)
  SET(CMAKE_STATIC_LIBRARY_SUFFIX "32.lib")
ELSE(WIN32)
  SET(ADDITIONAL_CFLAGS "-O3 -std=c++0x -msse3 -Wall -finline-functions -Wno-unused-but-set-variable -Wno-write-strings -Wno-char-subscripts -Wno-unused-result" CACHE STRING"Additional flags given to the compiler (-O3 -Wall -finline-functions -Wno-write-strings)" )
  # Add include path for OpenGL without GL/-prefix
  # to avoid the include incompatibility between MACOS
  # and linux
    FIND_PATH(OPENGL_INC gl.h /usr/include/GL)
    include_directories(${OPENGL_INC})
ENDIF(WIN32)

# Add OpenGL includes for MACOS if needed
# The OSX OpenGL frameworks natively supports freeglut extensions
IF(APPLE)
  include_directories(/System/Library/Frameworks/GLUT.framework/Headers)
  include_directories(/System/Library/Frameworks/OpenGL.framework/Headers)
ENDIF(APPLE)

# hack to "circumvent" Debug and Release folders that are created under visual studio
# this is why the INSTALL target has to be used in visual studio
IF(MSVC)
  INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/bin/Release/ CONFIGURATIONS Release DESTINATION ${CMAKE_SOURCE_DIR}/windows FILES_MATCHING PATTERN "*.dll" PATTERN "*.exe")
  IF( CMAKE_SIZEOF_VOID_P EQUAL 8 )
	  INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/3rdparty/windows/x64/ CONFIGURATIONS Release DESTINATION ${CMAKE_SOURCE_DIR}/windows FILES_MATCHING PATTERN "*.dll" PATTERN "*.exe")
  ELSE( CMAKE_SIZEOF_VOID_P EQUAL 8 )
	  INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/3rdparty/windows/ CONFIGURATIONS Release DESTINATION ${CMAKE_SOURCE_DIR}/windows FILES_MATCHING PATTERN "*.dll" PATTERN "*.exe")
  ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 8 )

  INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/bin/Debug/ CONFIGURATIONS Debug DESTINATION ${CMAKE_SOURCE_DIR}/windows FILES_MATCHING PATTERN "*.dll" PATTERN "*.exe")
  IF( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/3rdparty/windows/x64/ CONFIGURATIONS Debug DESTINATION ${CMAKE_SOURCE_DIR}/windows FILES_MATCHING PATTERN "*.dll" PATTERN "*.exe")
  ELSE( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/3rdparty/windows/ CONFIGURATIONS Debug DESTINATION ${CMAKE_SOURCE_DIR}/windows FILES_MATCHING PATTERN "*.dll" PATTERN "*.exe")
  ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 8 )
ENDIF(MSVC)



#################################################
# GENERAL PROJECT SETTINGS                     ##
#################################################

SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CFLAGS}")


# Set include and link dirs ...
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/3rdparty/)
include_directories(${CMAKE_SOURCE_DIR}/3rdparty/glui)
include_directories(${CMAKE_SOURCE_DIR}/3rdparty/wxthings/include/)
include_directories(${CMAKE_SOURCE_DIR}/3rdparty/ann_1.1.1_modified/include)
include_directories(${CMAKE_SOURCE_DIR}/3rdparty/ann_1.1.1_modified/src)
include_directories(${CMAKE_SOURCE_DIR}/3rdparty/lastools/laslib/src)
include_directories(${CMAKE_SOURCE_DIR}/3rdparty/lastools/laslib/inc)
link_directories(${CMAKE_SOURCE_DIR}/obj)
link_directories(${CMAKE_SOURCE_DIR}/lib)

add_subdirectory(src/slam6d)
add_subdirectory(src/scanio)
add_subdirectory(src/scanserver)
add_subdirectory(src/segmentation)
add_subdirectory(src/normals)
add_subdirectory(src/veloslam)
add_subdirectory(src/show)
add_subdirectory(src/grid)
add_subdirectory(src/pmd)
add_subdirectory(src/shapes)
add_subdirectory(src/thermo)
add_subdirectory(src/slam6d/fbr)
add_subdirectory(src/scanner)
add_subdirectory(src/model)
add_subdirectory(src/collision)
add_subdirectory(3rdparty)
IF(WITH_ADDONS)
  MESSAGE(STATUS "With 3dtkaddons.")
  add_subdirectory(addons)
ENDIF(WITH_ADDONS)

MESSAGE (STATUS "Build environment is set up!")
