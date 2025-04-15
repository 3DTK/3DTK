include_directories(${PROJECT_SOURCE_DIR}/include/srr)
include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(${SUITESPARSE_INCLUDE_DIRS})

## e57 to scan tool
if(WITH_E57)
  message(STATUS "With e57 Support.")
  add_definitions(-DWITH_E57)
  if(APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -framework CoreServices -framework CoreFoundation")
  endif()
  include_directories(${LIBE57_INCLUDE_DIRS})
  add_executable(e572scan e572scan.cpp)
  target_link_libraries(e572scan E57RefImpl ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
else()
  message(STATUS "Whitout e57 Support.")
endif()

include_directories(${SUITESPARSE_INCLUDE_DIRS})
include_directories(${ANN_INCLUDE_DIRS})
include_directories(${NEWMAT_INCLUDE_DIRS})

add_definitions(-DWITH_LIBZIP)

set(SRR_SRCS
  correction.cc
  linescan.cc
  continuousreg.cc
  lum6Deuler.cc
  ghelix6DQ2.cc
  simplereg.cc
  gapx6D.cc
  srr_program_options.cc
  lsegment.cc
  )

add_executable(correction ${SRR_SRCS})

if(WIN32)
  target_link_libraries(correction scan scanio ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
else()
  #  target_link_libraries(correction scan scanio ${LIBZIP_LIBRARY} ${NEWMAT_LIBRARIES_SHARED} ${CXSPARSE_LIBRARIES} ${ANN_LIBRARIES_SHARED} dl ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
  target_link_libraries(correction scan ${NEWMAT_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${ANN_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
endif()

