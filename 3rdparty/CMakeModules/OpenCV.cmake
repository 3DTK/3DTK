  find_package(OpenCV REQUIRED)
  if(EXISTS "${OpenCV_DIR}/OpenCVConfig.cmake")
    include("${OpenCV_DIR}/OpenCVConfig.cmake")
    set(ADDITIONAL_OPENCV_FLAGS
                            "-DCV_MINOR_VERSION=${OpenCV_VERSION_MINOR} -DCV_MAJOR_VERSION=${OpenCV_VERSION_MAJOR}" 
                            CACHE STRING"OpenCV Version Defines)" 
                            )
    ## Include the standard CMake script
  ELSE(EXISTS "${OpenCV_DIR}/OpenCVConfig.cmake")
    set(ADDITIONAL_OPENCV_FLAGS
                            ""
                            CACHE STRING"OpenCV Version Defines (BLUB)" 
                            )
  endif(EXISTS "${OpenCV_DIR}/OpenCVConfig.cmake")

