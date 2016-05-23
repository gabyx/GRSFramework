# Find STLSOFT Template Library
#
# INTEL_BLAS_FOUND
# INTEL_BLAS_INCLUDE_DIR      where to find the include files
# INTEL_BLAS_LIB_DIR
# INTEl_LIB_DIR
# INTEL_BLAS_LIBRARIES


SET(INTEL_BLAS_SEARCH "E:/Program/Intel/Compiler/11.1/060/mkl/em64t/" "E:/Program/Intel/Compiler/11.1/060/mkl/" "E:/Program/Intel/Compiler/11.1/060/lib/intel64")


FIND_PATH(INTEL_BLAS_INCLUDE_DIR "mkl_blas.h" PATHS ${INTEL_BLAS_SEARCH} PATH_SUFFIXES "include" ) 

FIND_PATH(INTEL_BLAS_LIB_DIR "mkl_core.lib" PATHS ${INTEL_BLAS_SEARCH} PATH_SUFFIXES "lib")
FIND_PATH(INTEL_LIB_DIR "libm.lib" PATHS ${INTEL_BLAS_SEARCH} PATH_SUFFIXES "lib") 


message(STATUS "BLAS Intel MKL Include Dir: " ${INTEL_BLAS_INCLUDE_DIR})
STRING(COMPARE NOTEQUAL ${INTEL_BLAS_INCLUDE_DIR}  "INTEL_BLAS_INCLUDE_DIR-NOTFOUND" INTEL_BLAS_FOUND)

IF(NOT INTEL_BLAS_FOUND)
	IF(INTEL_BLAS_FIND_REQUIRED)
		message( FATAL_ERROR "BLAS Intel MKL Include directory was not found!")
	ELSE(INTEL_BLAS_FIND_REQUIRED)
		message( STATUS "BLAS Intel MKL Include directory was not found!")
	ENDIF(INTEL_BLAS_FIND_REQUIRED)
ELSE(NOT INTEL_BLAS_FOUND)
	message(STATUS "BLAS Intel MKL Include directory found!")
ENDIF(NOT INTEL_BLAS_FOUND)

MARK_AS_ADVANCED( INTEL_BLAS_INCLUDE_DIR )
