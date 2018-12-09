if(APPLE)
    set(CMAKE_MACOSX_RPATH 1)
endif()
SET(NEWMAT_DIR ${PROJECT_SOURCE_DIR}/3rdparty/newmat/newmat-10/newmat/)
add_library(newmat
  ${NEWMAT_DIR}newmat1.cpp
  ${NEWMAT_DIR}newmat2.cpp
  ${NEWMAT_DIR}newmat3.cpp
  ${NEWMAT_DIR}newmat4.cpp
  ${NEWMAT_DIR}newmat5.cpp
  ${NEWMAT_DIR}newmat6.cpp
  ${NEWMAT_DIR}newmat7.cpp
  ${NEWMAT_DIR}newmat8.cpp
  ${NEWMAT_DIR}newmatex.cpp
  ${NEWMAT_DIR}bandmat.cpp
  ${NEWMAT_DIR}submat.cpp
  ${NEWMAT_DIR}myexcept.cpp
  ${NEWMAT_DIR}cholesky.cpp
  ${NEWMAT_DIR}evalue.cpp
  ${NEWMAT_DIR}fft.cpp
  ${NEWMAT_DIR}hholder.cpp
  ${NEWMAT_DIR}jacobi.cpp
  ${NEWMAT_DIR}newfft.cpp
  ${NEWMAT_DIR}sort.cpp
  ${NEWMAT_DIR}svd.cpp
  ${NEWMAT_DIR}newmatrm.cpp
  ${NEWMAT_DIR}newmat9.cpp)
