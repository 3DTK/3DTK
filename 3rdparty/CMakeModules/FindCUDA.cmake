###############################################################################
#  For more information, please see: http://software.sci.utah.edu
#
#  The MIT License
#
#  Copyright (c) 2007-2008
#  Scientific Computing and Imaging Institute, University of Utah
#
#  License for the specific language governing rights and limitations under
#  Permission is hereby granted, free of charge, to any person obtaining a
#  copy of this software and associated documentation files (the "Software"),
#  to deal in the Software without restriction, including without limitation
#  the rights to use, copy, modify, merge, publish, distribute, sublicense,
#  and/or sell copies of the Software, and to permit persons to whom the
#  Software is furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included
#  in all copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
#  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
#  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
#  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
#  DEALINGS IN THE SOFTWARE.
#
# This script locates the Nvidia Compute Unified Driver Architecture (CUDA)
# tools. It should work on linux, windows, and mac and should be reasonably
# up to date with cuda releases.
#
# The script will prompt the user to specify CUDA_INSTALL_PREFIX if the
# prefix cannot be determined by the location of nvcc in the system path. To
# use a different installed version of the toolkit set the environment variable
# CUDA_BIN_PATH before running cmake (e.g. CUDA_BIN_PATH=/usr/local/cuda1.0
# instead of the default /usr/local/cuda).
#
# Set CUDA_BUILD_EMULATION to ON for Emulation mode.  Defaults to OFF (device
# mode).
# _DEVICEEMU is defined when CUDA_BUILD_EMULATION is TRUE.
#
# Set CUDA_HOST_COMPILATION_CPP to OFF for C compilation of host code.
# Default TRUE.
#
# Set CUDA_BUILD_CUBIN to "ON" or "OFF" to enable and extra compilation pass
# with the -cubin option in Device mode. The output is parsed and register,
# shared memory usage is printed during build. Default ON.
#
# Set CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE to ON if you want the custom build
# rule to be attached to the source file in Visual Studio.  Defaults to OFF.
#
# This allows the user to build the target from the CUDA file, however bad
# things can happen if the CUDA source file is added to multiple targets.  When
# performing parallel builds it is possible for the custom build command to be
# run more than once and in parallel causing cryptic build errors.  This is
# because VS runs the rules for every source file in the target, and a source
# can have only one rule no matter how many projects it is added to.  Therefore,
# the rule assigned to the source file really only applies to one target you get
# clashes when it is run from multiple targets.  Eventually everything will get
# built, but if the user is unaware of this behavior, there may be confusion.
# It would be nice if we could detect the reuse of source files across multiple
# targets and turn the option off for the user, but no good solution could be
# found.
#
# Set CUDA_64_BIT_DEVICE_CODE to ON to compile for 64 bit devices.  Defaults to
# match host bit size.  Note that making this different than the host code when
# generating C files from CUDA code just won't work, because size_t gets defined
# by nvcc in the generated source.  If you compile to PTX and then load the file
# yourself, you can mix bit sizes between device and host.
#
# Set CUDA_VERBOSE_BUILD to ON to see all the commands used when building the
# CUDA file.  When using a Makefile generator the value defaults to VERBOSE (run
# make VERBOSE=1 to see output).  You can override this by setting
# CUDA_VERBOSE_BUILD to ON.
#
# The script creates the following macros:
# CUDA_INCLUDE_DIRECTORIES( path0 path1 ... )
# -- Sets the directories that should be passed to nvcc
#    (e.g. nvcc -Ipath0 -Ipath1 ... ). These paths usually contain other .cu
#    files.
#
# CUDA_ADD_LIBRARY( cuda_target file0 file1 ... )
# -- Creates a shared library "cuda_target" which contains all of the source
#    (*.c, *.cc, etc.) specified and all of the nvcc'ed .cu files specified.
#    All of the specified source files and generated .cpp files are compiled
#    using the standard CMake compiler, so the normal INCLUDE_DIRECTORIES,
#    LINK_DIRECTORIES, and TARGET_LINK_LIBRARIES can be used to affect their
#    build and link.
#
# CUDA_ADD_CUFFT_TO_TARGET( cuda_target )
# -- Adds the cufft library to the target.  Handles whether you are in emulation
#    mode or not.
#
# CUDA_ADD_CUBLAS_TO_TARGET( cuda_target )
# -- Adds the cublas library to the target.  Handles whether you are in emulation
#    mode or not.
#
# CUDA_ADD_EXECUTABLE( cuda_target file0 file1 ... )
# -- Same as CUDA_ADD_LIBRARY except that an exectuable is created.
#
# CUDA_COMPILE( cuda_files file0 file1 ... )
# -- Returns a list of build commands in the first argument to be used with
#    ADD_LIBRARY or ADD_EXECUTABLE.
#
# CUDA_BUILD_CLEAN_TARGET()
# -- Creates a convience target that deletes all the dependency files generated.
#    You should make clean after running this target to ensure the dependency
#    files get regenerated.
#
# The script defines the following variables:
#
# ( Note CUDA_ADD_* macros setup cuda/cut library dependencies automatically.
# These variables are only needed if a cuda API call must be made from code in
# a outside library or executable. )
#
# CUDA_INCLUDE_DIRS     -- Include directory for cuda headers.
# CUDA_LIBRARIES        -- Cuda RT library.
# CUDA_CUT_INCLUDE_DIRS -- Include directory for cuda SDK headers (cutil.h).
# CUDA_CUT_LIBRARIES    -- SDK libraries.
# CUDA_NVCC_FLAGS       -- Additional NVCC command line arguments. NOTE:
#                          multiple arguments must be semi-colon delimited
#                          e.g. --compiler-options;-Wall
# CUFFT_LIBRARIES       -- Device or emulation library for the Cuda FFT 
#                          implementation (alternative to: 
#                          CUDA_ADD_CUFFT_TO_TARGET macro)
# CUBLAS_LIBRARIES      -- Device or emulation library for the Cuda BLAS 
#                          implementation (alterative to: 
#                          CUDA_ADD_CUBLAS_TO_TARGET macro). 
#
#
# It might be necessary to set CUDA_INSTALL_PATH manually on certain platforms,
# or to use a cuda runtime not installed in the default location. In newer
# versions of the toolkit the cuda library is included with the graphics
# driver- be sure that the driver version matches what is needed by the cuda
# runtime version.
#
# -- Abe Stephens SCI Institute -- http://www.sci.utah.edu/~abe/FindCuda.html
# -- James Bigler NVIDIA Corp
###############################################################################

# FindCUDA.cmake

# This macro helps us find the location of helper files we will need the full path to
MACRO(CUDA_FIND_HELPER_FILE _name _extension)
  SET(_full_name "${_name}.${_extension}")
  # CMAKE_CURRENT_LIST_FILE contains the full path to the file currently being
  # processed.  Using this variable, we can pull out the current path, and
  # provide a way to get access to the other files we need local to here.
  GET_FILENAME_COMPONENT(CMAKE_CURRENT_LIST_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
  FIND_FILE(CUDA_${_name} ${_full_name} PATHS ${CMAKE_CURRENT_LIST_DIR} NO_DEFAULT_PATH)
  IF(NOT CUDA_${_name})
    SET(error_message "${_full_name} not found in CMAKE_MODULE_PATH")
    IF(CUDA_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "${error_message}")
    ELSE(CUDA_FIND_REQUIRED)
      IF(NOT CUDA_FIND_QUIETLY)
        MESSAGE(STATUS "${error_message}")
      ENDIF(NOT CUDA_FIND_QUIETLY)
    ENDIF(NOT CUDA_FIND_REQUIRED)
  ENDIF(NOT CUDA_${_name})
  # Set this variable as internal, so the user isn't bugged with it.
  SET(CUDA_${_name} ${CUDA_${_name}} CACHE INTERNAL "Location of ${_full_name}" FORCE)
ENDMACRO(CUDA_FIND_HELPER_FILE)

#####################################################################
## CUDA_INCLUDE_NVCC_DEPENDENCIES
##

# So we want to try and include the dependency file if it exists.  If
# it doesn't exist then we need to create an empty one, so we can
# include it.

# If it does exist, then we need to check to see if all the files it
# depends on exist.  If they don't then we should clear the dependency
# file and regenerate it later.  This covers the case where a header
# file has disappeared or moved.

# Need to locate the empty.depend.in file, because CONFIGURE_FILE requires
# full paths.
CUDA_FIND_HELPER_FILE(empty "depend.in")

MACRO(CUDA_INCLUDE_NVCC_DEPENDENCIES dependency_file)
  SET(CUDA_NVCC_DEPEND)
  SET(CUDA_NVCC_DEPEND_REGENERATE FALSE)


  # Include the dependency file.  Create it first if it doesn't exist
  # for make files except for IDEs (see below).  The INCLUDE puts a
  # dependency that will force CMake to rerun and bring in the new info
  # when it changes.  DO NOT REMOVE THIS (as I did and spent a few hours
  # figuring out why it didn't work.
#  IF(${CMAKE_MAKE_PROGRAM} MATCHES "make")
    IF(NOT EXISTS ${dependency_file})
      #message("configuring dependency_file = ${dependency_file}")
      CONFIGURE_FILE(
        ${CUDA_empty}
        ${dependency_file} IMMEDIATE)
    ENDIF(NOT EXISTS ${dependency_file})
    # Always include this file to force CMake to run again next
    # invocation and rebuild the dependencies.
    #message("including dependency_file = ${dependency_file}")
    INCLUDE(${dependency_file})
#   ELSE(${CMAKE_MAKE_PROGRAM} MATCHES "make")
#     # for IDE generators like MS dev only include the depend files
#     # if they exist.   This is to prevent ecessive reloading of
#     # workspaces after each build.   This also means
#     # that the depends will not be correct until cmake
#     # is run once after the build has completed once.
#     # the depend files are created in the wrap tcl/python sections
#     # when the .xml file is parsed.
#     INCLUDE(${dependency_file} OPTIONAL RESULT_VARIABLE found)
#     if (found)
#       message("included dependency_file (${dependency_file}) found")
#     else()
#       message("included dependency_file (${dependency_file}) NOT found")
#     endif()
#   ENDIF(${CMAKE_MAKE_PROGRAM} MATCHES "make")

  # Now we need to verify the existence of all the included files
  # here.  If they aren't there we need to just blank this variable and
  # make the file regenerate again.
#   if(DEFINED CUDA_NVCC_DEPEND)
#     message("CUDA_NVCC_DEPEND set")
#   else()
#     message("CUDA_NVCC_DEPEND NOT set")
#   endif()
  IF(CUDA_NVCC_DEPEND)
    #message("CUDA_NVCC_DEPEND true")
    FOREACH(f ${CUDA_NVCC_DEPEND})
      #message("searching for ${f}")
      IF(NOT EXISTS ${f})
        #message("file ${f} not found")
        SET(CUDA_NVCC_DEPEND_REGENERATE TRUE)
      ENDIF()
    ENDFOREACH(f)
  ELSE(CUDA_NVCC_DEPEND)
    #message("CUDA_NVCC_DEPEND false")
    # No dependencies, so regenerate the file.
    SET(CUDA_NVCC_DEPEND_REGENERATE TRUE)
  ENDIF(CUDA_NVCC_DEPEND)

  #message("CUDA_NVCC_DEPEND_REGENERATE = ${CUDA_NVCC_DEPEND_REGENERATE}")
  # No incoming dependencies, so we need to generate them.  Make the
  # output depend on the dependency file itself, which should cause the
  # rule to re-run.
  IF(CUDA_NVCC_DEPEND_REGENERATE)
    SET(CUDA_NVCC_DEPEND ${dependency_file})
    # Force CMake to run again next build
    CONFIGURE_FILE(
      ${CUDA_empty}
      ${dependency_file} IMMEDIATE)
  ENDIF(CUDA_NVCC_DEPEND_REGENERATE)

ENDMACRO(CUDA_INCLUDE_NVCC_DEPENDENCIES)

###############################################################################
###############################################################################
# Locate CUDA, Set Build Type, etc.
###############################################################################
###############################################################################

# Set whether we are using emulation or device mode.
OPTION(CUDA_BUILD_EMULATION "Build in Emulation mode" OFF)
# Parse HOST_COMPILATION mode.
OPTION(CUDA_HOST_COMPILATION_CPP "Generated file extension" ON)
# Allow the user to specify if the device code is supposed to be 32 or 64 bit.
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(CUDA_64_BIT_DEVICE_CODE_DEFAULT ON)
else()
  set(CUDA_64_BIT_DEVICE_CODE_DEFAULT OFF)
endif()
OPTION(CUDA_64_BIT_DEVICE_CODE "Compile device code in 64 bit mode" ${CUDA_64_BIT_DEVICE_CODE_DEFAULT})
# Prints out extra information about the cuda file during compilation
OPTION(CUDA_BUILD_CUBIN "Generate and parse .cubin files in Device mode." ON)
# Extra user settable flags
SET(CUDA_NVCC_FLAGS "" CACHE STRING "Semi-colon delimit multiple arguments.")
# Attach the build rule to the source file in VS.  This option
OPTION(CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE "Attach the build rule to the CUDA source file.  Enable only when the CUDA source file is added to at most one target." OFF)
# Specifies whether the commands used when compiling the .cu file will be printed out.
OPTION(CUDA_VERBOSE_BUILD "Print out the commands run while compiling the CUDA source file.  With the Makefile generator this defaults to VERBOSE variable specified on the command line, but can be forced on with this option." OFF)

# Search for the cuda distribution.
IF(NOT CUDA_INSTALL_PREFIX)

  # Search in the CUDA_BIN_PATH first.
  FIND_PATH(CUDA_INSTALL_PREFIX
    NAMES nvcc nvcc.exe
    PATHS ENV CUDA_BIN_PATH
    DOC "Toolkit location."
    NO_DEFAULT_PATH
    )
  # Now search default paths
  FIND_PATH(CUDA_INSTALL_PREFIX
    NAMES nvcc nvcc.exe
    PATHS /usr/local/bin
          /usr/local/cuda/bin
    DOC "Toolkit location."
    )

  IF (CUDA_INSTALL_PREFIX)
    STRING(REGEX REPLACE "[/\\\\]?bin[/\\\\]?$" "" CUDA_INSTALL_PREFIX ${CUDA_INSTALL_PREFIX})
    # We need to force this back into the cache.
    SET(CUDA_INSTALL_PREFIX ${CUDA_INSTALL_PREFIX} CACHE PATH "Toolkit location." FORCE)
  ENDIF(CUDA_INSTALL_PREFIX)
  IF (NOT EXISTS ${CUDA_INSTALL_PREFIX})
    MESSAGE(FATAL_ERROR "Specify CUDA_INSTALL_PREFIX")
  ENDIF (NOT EXISTS ${CUDA_INSTALL_PREFIX})
ENDIF (NOT CUDA_INSTALL_PREFIX)

# CUDA_NVCC
IF (NOT CUDA_NVCC)
  FIND_PROGRAM(CUDA_NVCC
    NAMES nvcc
    PATHS "${CUDA_INSTALL_PREFIX}/bin"
    ENV CUDA_BIN_PATH
    NO_DEFAULT_PATH
    )
  # Search default search paths, after we search our own set of paths.
  FIND_PROGRAM(CUDA_NVCC nvcc)
  IF(NOT CUDA_NVCC)
    MESSAGE(FATAL_ERROR "Could not find nvcc")
  ELSE(NOT CUDA_NVCC)
    MARK_AS_ADVANCED(CUDA_NVCC)
  ENDIF(NOT CUDA_NVCC)
ENDIF(NOT CUDA_NVCC)

# IF (NOT FOUND_CUDA_NVCC_INCLUDE)
FIND_PATH(FOUND_CUDA_NVCC_INCLUDE
  device_functions.h # Header included in toolkit
  PATHS "${CUDA_INSTALL_PREFIX}/include"
  ENV CUDA_INC_PATH
  NO_DEFAULT_PATH
  )
# Search default search paths, after we search our own set of paths.
FIND_PATH(FOUND_CUDA_NVCC_INCLUDE device_functions.h)

IF(NOT FOUND_CUDA_NVCC_INCLUDE)
  MESSAGE(FATAL_ERROR "Could not find CUDA headers")
ELSE(NOT FOUND_CUDA_NVCC_INCLUDE)
  # Set the user list of include dir to nothing to initialize it.
  SET (CUDA_NVCC_INCLUDE_ARGS_USER "")
  SET (CUDA_INCLUDE_DIRS ${FOUND_CUDA_NVCC_INCLUDE})

  MARK_AS_ADVANCED(
    FOUND_CUDA_NVCC_INCLUDE
    )
ENDIF(NOT FOUND_CUDA_NVCC_INCLUDE)

# ENDIF(NOT FOUND_CUDA_NVCC_INCLUDE)


MACRO(FIND_LIBRARY_LOCAL_FIRST _var _names _doc)
  FIND_LIBRARY(${_var}
    NAMES ${_names}
    PATHS "${CUDA_INSTALL_PREFIX}/lib"
    ENV CUDA_LIB_PATH
    DOC ${_doc}
    NO_DEFAULT_PATH
    )
  # Search default search paths, after we search our own set of paths.
  FIND_LIBRARY(${_var} NAMES ${_names} DOC ${_doc})
ENDMACRO()

# CUDA_LIBRARIES
IF (NOT CUDA_LIBRARIES)

  FIND_LIBRARY_LOCAL_FIRST(FOUND_CUDART cudart "\"cudart\" library")

  # Check to see if cudart library was found.
  IF(NOT FOUND_CUDART)
    MESSAGE(FATAL_ERROR "Could not find cudart library (cudart)")
  ENDIF(NOT FOUND_CUDART)

  # 1.1 toolkit on linux doesn't appear to have a separate library on
  # some platforms.
  FIND_LIBRARY_LOCAL_FIRST(FOUND_CUDA cuda "\"cuda\" library (older versions only).")

  # Add cuda library to the link line only if it is found.
  IF (FOUND_CUDA)
    SET(CUDA_LIBRARIES ${FOUND_CUDA})
  ENDIF(FOUND_CUDA)

  # Always add cudart to the link line.
  IF(FOUND_CUDART)
    SET(CUDA_LIBRARIES
      ${CUDA_LIBRARIES} ${FOUND_CUDART}
      )
    MARK_AS_ADVANCED(
      CUDA_LIBRARIES
      CUDA_LIB
      FOUND_CUDA
      FOUND_CUDART
      )
  ELSE(FOUND_CUDART)
    MESSAGE(FATAL_ERROR "Could not find cuda libraries.")
  ENDIF(FOUND_CUDART)

ENDIF(NOT CUDA_LIBRARIES)

FIND_PATH(CUDA_SDK_INSTALL_PREFIX common/inc/cutil.h
  "/usr/local/src/NVIDIA_GPU_Computing_SDK/C/"
  "[HKEY_LOCAL_MACHINE\\SOFTWARE\\NVIDIA Corporation\\Installed Products\\NVIDIA SDK 10\\Compute;InstallDir]"
  "$ENV{NVSDKCUDA_ROOT}"
  )

# Keep the CUDA_INSTALL_PREFIX first in order to be able to override the
# environment variables.
SET(CUDA_SDK_SEARCH_PATH
  "${CUDA_SDK_INSTALL_PREFIX}"
  "${CUDA_INSTALL_PREFIX}/local/NVSDK0.2"
  "${CUDA_INSTALL_PREFIX}/NVSDK0.2"
  "${CUDA_INSTALL_PREFIX}/NV_CUDA_SDK"
  "$ENV{HOME}/NVIDIA_CUDA_SDK"
  "$ENV{HOME}/NVIDIA_CUDA_SDK_MACOSX"
  "/Developer/CUDA"
  )
# CUDA_CUT_INCLUDE_DIRS
IF(NOT CUDA_CUT_INCLUDE_DIRS)
  FIND_PATH(FOUND_CUT_INCLUDE
    cutil.h
    PATHS ${CUDA_SDK_SEARCH_PATH}
    PATH_SUFFIXES "common/inc"
    DOC "Location of cutil.h"
    NO_DEFAULT_PATH
    )
  # Now search system paths
  FIND_PATH(FOUND_CUT_INCLUDE cutil.h DOC "Location of cutil.h")

  IF(FOUND_CUT_INCLUDE)
    SET(CUDA_CUT_INCLUDE_DIRS ${FOUND_CUT_INCLUDE})
    MARK_AS_ADVANCED(
      FOUND_CUT_INCLUDE
      )
  ENDIF(FOUND_CUT_INCLUDE)
ENDIF(NOT CUDA_CUT_INCLUDE_DIRS)


# CUDA_CUT_LIBRARIES
IF(NOT CUDA_CUT_LIBRARIES)
  # cutil library is called cutil64 for 64 bit builds on windows.  We don't want
  # to get these confused, so we are setting the name based on the word size of
  # the build.
  IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
    SET(cuda_cutil_name cutil64)
  ELSE(CMAKE_SIZEOF_VOID_P EQUAL 8)
    SET(cuda_cutil_name cutil32)
  ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 8)

  FIND_LIBRARY(FOUND_CUT
    NAMES cutil ${cuda_cutil_name}
    PATHS ${CUDA_SDK_SEARCH_PATH}
    # The new version of the sdk shows up in common/lib, but the old one is in lib
    PATH_SUFFIXES "common/lib" "lib"
    DOC "Location of cutil library"
    NO_DEFAULT_PATH
    )
  # Now search system paths
  FIND_LIBRARY(FOUND_CUT NAMES cutil ${cuda_cutil_name} DOC "Location of cutil library")

  IF(FOUND_CUT)
    SET(CUDA_CUT_LIBRARIES ${FOUND_CUT})
    MARK_AS_ADVANCED(FOUND_CUT)
  ENDIF(FOUND_CUT)

  MACRO(FIND_CUDA_HELPER_LIBS _name)
    FIND_LIBRARY_LOCAL_FIRST(FOUND_${_name} ${_name} "\"${_name}\" library")
    MARK_AS_ADVANCED(FOUND_${_name})
  ENDMACRO(FIND_CUDA_HELPER_LIBS)

  # Search for cufft and cublas libraries.
  FIND_CUDA_HELPER_LIBS(cufftemu)
  FIND_CUDA_HELPER_LIBS(cublasemu)
  FIND_CUDA_HELPER_LIBS(cufft)
  FIND_CUDA_HELPER_LIBS(cublas)

  if (CUDA_BUILD_EMULATION)
    SET(CUFFT_LIBRARIES ${FOUND_cufftemu})
    SET(CUBLAS_LIBRARIES ${FOUND_cublasemu})
  else()
    SET(CUFFT_LIBRARIES ${FOUND_cufft})
    SET(CUBLAS_LIBRARIES ${FOUND_cublas})
  endif()  

ENDIF(NOT CUDA_CUT_LIBRARIES)


###############################################################################
# Add include directories to pass to the nvcc command.
MACRO(CUDA_INCLUDE_DIRECTORIES)
  FOREACH(dir ${ARGN})
    list(APPEND CUDA_NVCC_INCLUDE_ARGS_USER "-I${dir}")
  ENDFOREACH(dir ${ARGN})
ENDMACRO(CUDA_INCLUDE_DIRECTORIES)

CUDA_FIND_HELPER_FILE(parse_cubin cmake)
CUDA_FIND_HELPER_FILE(make2cmake cmake)
CUDA_FIND_HELPER_FILE(run_nvcc cmake)

##############################################################################
##############################################################################
# This helper macro populates the following variables and setups up custom 
# commands and targets to invoke the nvcc compiler to generate C or PTX source
# dependant upon the format parameter.  The compiler is invoked once with -M 
# to generate a dependency file and a second time with -cuda or -ptx to generate
# a .cpp or .ptx file. 
# INPUT:
#   cuda_target         - Target name
#   format              - PTX or C
# OUTPUT:
#   ${target_srcs}      - List of the generated C or PTX source files
#   ${cuda_cu_sources}  - List of the original CU files
##############################################################################
##############################################################################

MACRO(CUDA_add_custom_commands cuda_target format)

  IF( ${format} MATCHES "PTX" )
    SET( compile_to_ptx ON )
  ELSEIF( ${format} MATCHES "C")
    SET( compile_to_ptx OFF )
  ELSE()
    MESSAGE( "Invalid format flag passed to CUDA_add_custom_commands: '${format}'.  Defaulting to C")
    SET( compile_to_ptx OFF )
  ENDIF()

  # Set up all the command line flags here, so that they can be overriden on a per target basis.

  set(nvcc_flags "")

  # Emulation if the card isn't present.
  IF (CUDA_BUILD_EMULATION)
    # Emulation.
    SET(nvcc_flags ${nvcc_flags} --device-emulation -D_DEVICEEMU -g)
  ELSE(CUDA_BUILD_EMULATION)
    # Device mode.  No flags necessary.
  ENDIF(CUDA_BUILD_EMULATION)

  IF(CUDA_HOST_COMPILATION_CPP)
    SET(generated_extension cpp)
  ELSE(CUDA_HOST_COMPILATION_CPP)
    SET(generated_extension c)
    SET(nvcc_flags ${nvcc_flags} --host-compilation C)
  ENDIF(CUDA_HOST_COMPILATION_CPP)

  IF(CUDA_64_BIT_DEVICE_CODE)
    SET(nvcc_flags ${nvcc_flags} -m64)
  ELSE()
    SET(nvcc_flags ${nvcc_flags} -m32)
  ENDIF()

  # This needs to be passed in at this stage, because VS needs to fill out the
  # value of VCInstallDir from withing VS.
  IF(CMAKE_GENERATOR MATCHES "Visual Studio")
    IF( CMAKE_SIZEOF_VOID_P EQUAL 8 )
      # Add nvcc flag for 64b Windows
      SET(ccbin_flags -D "\"CCBIN:PATH=$(VCInstallDir)bin\"" )
    ENDIF( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  ENDIF(CMAKE_GENERATOR MATCHES "Visual Studio")

  # Initialize our list of includes with the user ones followed by the CUDA system ones.
  set(CUDA_NVCC_INCLUDE_ARGS ${CUDA_NVCC_INCLUDE_ARGS_USER} -I${CUDA_INCLUDE_DIRS} -I${CUDA_CUT_INCLUDE_DIRS})
  # Get the include directories for this directory and use them for our nvcc command.
  get_directory_property(CUDA_NVCC_INCLUDE_DIRECTORIES INCLUDE_DIRECTORIES)
  if(CUDA_NVCC_INCLUDE_DIRECTORIES)
    foreach(dir ${CUDA_NVCC_INCLUDE_DIRECTORIES})
      list(APPEND CUDA_NVCC_INCLUDE_ARGS "-I${dir}")
    endforeach()
  endif()

  SET(target_srcs "")
  SET(cuda_cu_sources "")

  # Iterate over the macro arguments and create custom
  # commands for all the .cu files.
  FOREACH(file ${ARGN})
    IF(${file} MATCHES ".*\\.cu$")

      # Add a custom target to generate a c or ptx file. ######################
      GET_FILENAME_COMPONENT( basename ${file} NAME )
      IF( compile_to_ptx )
        SET(generated_file "${CMAKE_BINARY_DIR}/lib/ptx/${cuda_target}_generated_${basename}.ptx")
        SET(format_flag "-ptx")
        FILE(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/lib/ptx)
      ELSE( compile_to_ptx )
        SET(generated_file "${CMAKE_BINARY_DIR}/src/cuda/${cuda_target}_generated_${basename}.${generated_extension}")
        SET(format_flag "-cuda")
        FILE(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/src/cuda)
      ENDIF( compile_to_ptx )

      SET(generated_target "${file}_target")


      SET(source_file ${CMAKE_CURRENT_SOURCE_DIR}/${file})

      # Bring in the dependencies.  Creates a variable CUDA_NVCC_DEPEND #######
      SET(cmake_dependency_file "${generated_file}.depend")
      CUDA_INCLUDE_NVCC_DEPENDENCIES(${cmake_dependency_file})
      SET(NVCC_generated_dependency_file "${generated_file}.NVCC-depend")

      # Convience string for output ###########################################
      IF(CUDA_BUILD_EMULATION)
        SET(cuda_build_type "Emulation")
      ELSE(CUDA_BUILD_EMULATION)
        SET(cuda_build_type "Device")
      ENDIF(CUDA_BUILD_EMULATION)

      # Build the NVCC made dependency file ###################################
      SET(build_cubin OFF)
      IF ( NOT CUDA_BUILD_EMULATION AND CUDA_BUILD_CUBIN )
         IF ( NOT compile_to_ptx )
           SET ( build_cubin ON )

           # Initialize a string containing commands to produce the cubin report.
           SET(generated_cubin_file ${generated_file}.cubin.txt)

         ENDIF( NOT compile_to_ptx )
      ENDIF( NOT CUDA_BUILD_EMULATION AND CUDA_BUILD_CUBIN )

      # Configure the build script
      set(custom_target_script "${generated_file}.cmake")
      configure_file("${CUDA_run_nvcc}" "${custom_target_script}" @ONLY)

      # So if a user specifies the same cuda file as input more than once, you
      # can have bad things happen with dependencies.  Here we check an option
      # to see if this is the behavior they want.
      if(CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE)
        set(main_dep MAIN_DEPENDENCY ${source_file})
      else()
        set(main_dep DEPENDS ${source_file})
      endif()

      if(CUDA_VERBOSE_BUILD)
        set(verbose_output ON)
      elseif(CMAKE_GENERATOR MATCHES "Makefiles")
        set(verbose_output "$(VERBOSE)")
      else()
        set(verbose_output OFF)
      endif()

      # Build the generated file and dependency file ##########################
      add_custom_command(
        OUTPUT ${generated_file}
        # These output files depend on the source_file and the contents of cmake_dependency_file
        ${main_dep}
        DEPENDS ${CUDA_NVCC_DEPEND}
        DEPENDS ${custom_target_script}
        COMMAND ${CMAKE_COMMAND} ARGS -D verbose:BOOL=${verbose_output} ${ccbin_flags} -P "${custom_target_script}"
        COMMENT "Building (${cuda_build_type}) NVCC ${source_file}: ${generated_file}"
        )

      # Make sure the build system knows the file is generated.
      set_source_files_properties(${generated_file} PROPERTIES GENERATED TRUE)

      # Add the generated file name to the source list.  ######################
      SET(cuda_cu_sources ${cuda_cu_sources} ${source_file})
      SET(target_srcs ${target_srcs} ${generated_file})

      # Add the other files that we want cmake to clean on a cleanup ##########
      list(APPEND CUDA_ADDITIONAL_CLEAN_FILES "${cmake_dependency_file}")
      list(REMOVE_DUPLICATES CUDA_ADDITIONAL_CLEAN_FILES)
      set(CUDA_ADDITIONAL_CLEAN_FILES ${CUDA_ADDITIONAL_CLEAN_FILES} CACHE INTERNAL "List of intermediate files that are part of the cuda dependency scanning.")

    ELSE(${file} MATCHES ".*\\.cu$")

      # Otherwise add the file name to the source list.
      SET(target_srcs ${target_srcs} ${file})

    ENDIF(${file} MATCHES ".*\\.cu$")
  ENDFOREACH(file)

ENDMACRO(CUDA_add_custom_commands)


###############################################################################
###############################################################################
# ADD LIBRARY
###############################################################################
###############################################################################
MACRO(CUDA_ADD_LIBRARY cuda_target)

  # Create custom commands and targets for each file.
  CUDA_add_custom_commands( ${cuda_target} C ${ARGN} )

  # Add the library.
  ADD_LIBRARY(${cuda_target}
    ${target_srcs}
    ${cuda_cu_sources}
    )

  TARGET_LINK_LIBRARIES(${cuda_target}
    ${CUDA_LIBRARIES}
    )

ENDMACRO(CUDA_ADD_LIBRARY cuda_target)


###############################################################################
###############################################################################
# ADD EXECUTABLE
###############################################################################
###############################################################################
MACRO(CUDA_ADD_EXECUTABLE cuda_target)

  # Create custom commands and targets for each file.
  CUDA_add_custom_commands( ${cuda_target} C ${ARGN} )

  # Add the library.
  ADD_EXECUTABLE(${cuda_target}
    ${target_srcs}
    ${cuda_cu_sources}
    )

  TARGET_LINK_LIBRARIES(${cuda_target}
    ${CUDA_LIBRARIES}
    )


ENDMACRO(CUDA_ADD_EXECUTABLE cuda_target)


###############################################################################
###############################################################################
# CUDA COMPILE
###############################################################################
###############################################################################
MACRO(CUDA_COMPILE file_variable)

  # Create custom commands and targets for each file.
  CUDA_add_custom_commands( cuda_compile C ${ARGN} )

  SET(${file_variable} ${target_srcs} ${cuda_cu_sources})

ENDMACRO(CUDA_COMPILE)


###############################################################################
###############################################################################
# CUDA COMPILE PTX
###############################################################################
###############################################################################
MACRO(CUDA_COMPILE_PTX ptx_files cuda_files )

  # Create custom commands and targets for each file.
  CUDA_add_custom_commands( cuda_compile_ptx PTX ${ARGN} )

  SET( ${ptx_files} ${target_srcs} )
  SET( ${cuda_files} ${cuda_cu_sources} )

ENDMACRO(CUDA_COMPILE_PTX)

###############################################################################
###############################################################################
# CUDA ADD CUTIL TO TARGET
###############################################################################
###############################################################################
macro(CUDA_ADD_CUTIL_TO_TARGET target)
  target_link_libraries(${target} ${CUDA_CUT_LIBRARIES})
endmacro()

###############################################################################
###############################################################################
# CUDA ADD CUFFT TO TARGET
###############################################################################
###############################################################################
macro(CUDA_ADD_CUFFT_TO_TARGET target)
  if (CUDA_BUILD_EMULATION)
    target_link_libraries(${target} ${FOUND_cufftemu})
  else()
    target_link_libraries(${target} ${FOUND_cufft})
  endif()
endmacro()

###############################################################################
###############################################################################
# CUDA ADD CUBLAS TO TARGET
###############################################################################
###############################################################################
macro(CUDA_ADD_CUBLAS_TO_TARGET target)
  if (CUDA_BUILD_EMULATION)
    target_link_libraries(${target} ${FOUND_cublasemu})
  else()
    target_link_libraries(${target} ${FOUND_cublas})
  endif()
endmacro()

###############################################################################
###############################################################################
# CUDA BUILD CLEAN TARGET
###############################################################################
###############################################################################
macro(CUDA_BUILD_CLEAN_TARGET)
  # Call this after you add all your CUDA targets, and you will get a convience
  # target.  You should also make clean after running this target to get the
  # build system to generate all the code again.
  add_custom_target(CleanCudaDepends
    COMMAND ${CMAKE_COMMAND} -E remove ${CUDA_ADDITIONAL_CLEAN_FILES})

  # Clear out the variable, so the next time we configure it will be empty.
  # This is useful so that the files won't persist in the list after targets
  # have been removed.
  set(CUDA_ADDITIONAL_CLEAN_FILES "" CACHE INTERNAL "List of intermediate files that are part of the cuda dependency scanning.")
endmacro(CUDA_BUILD_CLEAN_TARGET)
