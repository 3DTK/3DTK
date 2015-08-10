### CUDALIB
IF(WITH_CUDA)

SET(CUDALIB_SRCS
  grid.cc tools.cc grid_kernel.cu
  )

cuda_add_library(cudaslam STATIC ${CUDALIB_SRCS})

FIND_PACKAGE(CUDA REQUIRED)

target_link_libraries(cudaslam)


ENDIF(WITH_CUDA)
