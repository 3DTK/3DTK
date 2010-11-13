#cmakelists.txt for subdirectory APSCpp 07 Mar 2008 TKSharpless
#
# Builds autopano-sift-c aka APSC++
#
# Uses the ANN kd-tree implementation.  Builds a static ANN library from 
# source in subdirectory ANN.  Note ANN.h has been customized for APSCpp.
#
# ANN needs this to satisfy #include "ANN/xxx"
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR} )
            
add_subdirectory( ANN )

# source files that must be compiled as C 
SET( C_Sources CamLens.c HermiteSpline.c saInterp.c saRemap.c )

#source files that must be compiled as C++ on Windows

SET( Cpp_Sources APSCpp_main.c APSCpp.c )

if( MSVC )
  SET_SOURCE_FILES_PROPERTIES( ${Cpp_Sources}
                    PROPERTIES  LANGUAGE CXX 
                             )
endif( MSVC )

# the build target
add_executable( autopano-sift-c ANNkd_wrap.cpp ${Cpp_Sources} ${C_Sources})

TARGET_LINK_LIBRARIES( autopano-sift-c ANN ${all_libs} )

install(TARGETS autopano-sift-c DESTINATION bin)
