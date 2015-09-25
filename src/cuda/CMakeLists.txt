### CUDALIB
if(WITH_CUDA)

set(CUDALIB_SRCS
  grid.cc tools.cc grid_kernel.cu
  )

cuda_add_library(cudaslam STATIC ${CUDALIB_SRCS})

find_package(CUDA REQUIRED)

target_link_libraries(cudaslam)


endif()
