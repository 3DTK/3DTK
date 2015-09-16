if(WITH_FLOORPLAN)

    # Module name
    set(MODULE "Floorplan")
    set(UNIX_LIBS scan newmat dl ANN)
    set(WIN32_LIBS scan newmat XGetopt)

    # Set CXX flags for CGAL
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frounding-math")
    
    # Sources
    file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.cc")
    
    # Binaries
    add_executable(floorplan ${SOURCES})

    # Find libraries
    find_library(LIB_CGAL CGAL)
    find_library(LIB_GMP gmp)
    find_package(Boost REQUIRED)
    find_package(OpenCV REQUIRED)
    
    if(UNIX)
        target_link_libraries(floorplan ${UNIX_LIBS} ${LIB_CGAL} ${Boost_LIBS} ${LIB_GMP} ${OpenCV_LIBS})
    endif(UNIX)

    if (WIN32)
        target_link_libraries(floorplan ${UNIX_LIBS} ${LIB_CGAL} ${Boost_LIBS} ${LIB_GMP} ${OpenCV_LIBS})
    endif(WIN32)

endif(WITH_FLOORPLAN)
