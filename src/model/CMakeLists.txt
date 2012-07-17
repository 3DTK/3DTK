if(WITH_MODEL)

    # Module name
    set(MODULE "Model")
    set(UNIX_LIBS shape scan newmat dl ANN)
    set(WIN32_LIBS shape scan newmat XGetopt)

    # Set CXX flags for CGAL
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frounding-math")
    
    # Sources
    file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.cc")
    
    # Binaries
    add_executable(model ${SOURCES})

    # Find libraries
    find_library(LIB_CGAL CGAL)
    find_library(LIB_GMP gmp)
    find_package(Boost REQUIRED)
    find_package(OpenCV REQUIRED)
    
    if(UNIX)
        target_link_libraries(model ${UNIX_LIBS} ${LIB_CGAL} ${Boost_LIBS} ${LIB_GMP} ${OpenCV_LIBS})
    endif(UNIX)

    if (WIN32)
        target_link_libraries(model ${UNIX_LIBS} ${LIB_CGAL} ${Boost_LIBS} ${LIB_GMP} ${OpenCV_LIBS})
    endif(WIN32)

endif(WITH_MODEL)
