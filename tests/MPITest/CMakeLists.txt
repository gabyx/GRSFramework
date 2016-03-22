#Detect Build Type if Debug appears in CMAKE_BINARY_DIR path
MARK_AS_ADVANCED( CMAKE_BUILD_TYPE )
STRING(REGEX MATCH ".*(Debug|debug).*" DEBUG_MATCH ${CMAKE_BINARY_DIR} )
if(DEBUG_MATCH)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "The build type for makefile based generators")
ELSE()
    set(CMAKE_BUILD_TYPE Release CACHE STRING "The build type for makefile based generators")
ENDIF()


SET(CMAKE_VERBOSE_MAKEFILE ON)


ENABLE_TESTING()

if(${CMAKE_GENERATOR} MATCHES "Unix Makefiles")
        set(MPIProjectName "MPIProject${CMAKE_BUILD_TYPE}")
else()
        set(MPIProjectName "MPIProject")
endif()

PROJECT(${MPIProjectName})



#The CMake Minimum version that is required. The FindCUDA script
#is distributed since version 2.8
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

# If GNU C Compiler set warning lenght to one line for eclipse
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fmessage-length=0")
ENDIF(CMAKE_COMPILER_IS_GNUCC)
IF(CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)



set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../CMakeModules/;${CMAKE_MODULE_PATH}")
message(STATUS "Module path is now set to: " ${CMAKE_MODULE_PATH} )

set(COMMON_SOURCE_DIR "${PROJECT_SOURCE_DIR}/CommonSource")
message(STATUS "CommonSource Dir is set to: ${COMMON_SOURCE_DIR} ")

include(PrintListMacro)


#Add some multithreaded build support
MARK_AS_ADVANCED(MULTITHREADED_BUILD)
set(MULTITHREADED_BUILD 12 CACHE STRING "How many threads are used to build the project")
if(MULTITHREADED_BUILD)
    if(${CMAKE_GENERATOR} MATCHES "Unix Makefiles")
            message(STATUS ${CMAKE_MAKE_PROGRAM})
            #set(CMAKE_MAKE_PROGRAM "${CMAKE_MAKE_PROGRAM} -j${MULTITHREADED_BUILD}")
            message(STATUS "Added arguments to CMAKE_BUILD_TOOL: ${CMAKE_MAKE_PROGRAM}")
    elseif(MSVC)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
      message(STATUS "Added parallel build arguments to CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
    endif()
endif()

##Searching CUDA
## Set the flags
#IF(WIN32)
#        SET(MYPROJECT_CUDA_PROPAGATE_HOST_FLAGS ON CACHE BOOL "Propagate C/CXX Flags and friends to the host compiler in NVCC via -Xompile")
#ENDIF()

#IF(UNIX)
#        SET(MYPROJECT_CUDA_PROPAGATE_HOST_FLAGS OFF CACHE BOOL "Propagate C/CXX Flags and friends to the host compiler in NVCC via -Xompile")
#ENDIF()

## Set the architecture for the grafics card!
#SET(MYPROJECT_CUDA_NVCC_FLAGS
#"-gencode=arch=compute_20,code=\\\"sm_20,compute_20\\\""
#"--ptxas-options=-v"
#CACHE VALUE "Semicolon delimited several arguments for the NVCC compiler")
#SET(MYPROJECT_CUDA_NVCC_FLAGS_DEBUG
#"-g"
#"-G"
#CACHE VALUE "Semicolon delimited several arguments for the NVCC compiler")
#SET(MYPROJECT_CUDA_NVCC_FLAGS_RELEASE "-O2"
#CACHE VALUE "Semicolon delimited several arguments for the NVCC compiler")

#FIND_PACKAGE(CUDA REQUIRED)
#SET(CUDA_VERBOSE_BUILD ON)
#SET(CUDA_NVCC_FLAGS           "${MYPROJECT_CUDA_NVCC_FLAGS}"            CACHE INTERNAL "Internal flags do not edit" FORCE)
#SET(CUDA_NVCC_FLAGS_DEBUG     "${MYPROJECT_CUDA_NVCC_FLAGS_DEBUG}"      CACHE INTERNAL "Internal flags do not edit" FORCE)
#SET(CUDA_NVCC_FLAGS_RELEASE   "${MYPROJECT_CUDA_NVCC_FLAGS_RELEASE}"    CACHE INTERNAL "Internal flags do not edit" FORCE)
#SET(CUDA_PROPAGATE_HOST_FLAGS "${MYPROJECT_CUDA_PROPAGATE_HOST_FLAGS}"  CACHE INTERNAL "Internal flags do not edit" FORCE)

## Configure File for the BUILD!
#string(REPLACE "." "" CUDA_VERSION_INTEGER ${CUDA_VERSION})


#Searching Eigen
FIND_PACKAGE(Eigen3 REQUIRED)
set(EIGEN_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR})

#Searching STLSoft
#FIND_PACKAGE(STLSoft REQUIRED)

#Searching Boost
set(Boost_ADDITIONAL_VERSIONS "1.46" "1.46.0" "1.46.1" "1.45" "1.45.0" "1.44" "1.44.0" "1.42" "1.42.0" "1.41.0" "1.41" "1.40.0" "1.40" "1.39.0" "1.39" "1.38.0" "1.38" "1.37.0" "1.37" )
set(BOOST_COMPONENTS thread date_time system filesystem timer chrono serialization mpi)
find_package(Boost COMPONENTS ${BOOST_COMPONENTS} REQUIRED)


#SET(MYPROJECT_USE_BLAS_VARIANT "OPENBLAS" CACHE STRING "Blas Variant either 'INTEL', 'GOTO','OPENBLAS', 'NONE' ")
#MARK_AS_ADVANCED(MYPROJECT_USE_BLAS_VARIANT)
#STRING(COMPARE EQUAL ${MYPROJECT_USE_BLAS_VARIANT} "GOTO" USE_GOTO_BLAS_FLAG)
#STRING(COMPARE EQUAL ${MYPROJECT_USE_BLAS_VARIANT} "OPENBLAS" USE_OPEN_BLAS_FLAG)
#STRING(COMPARE EQUAL ${MYPROJECT_USE_BLAS_VARIANT} "INTEL" USE_INTEL_BLAS_FLAG)

#if(USE_INTEL_BLAS_FLAG)
#	FIND_PACKAGE(IntelBlas)
#	if(INTEL_BLAS_FOUND)
#		MARK_AS_ADVANCED(MYPROJECT_MKL_LIBS)
#		SET(MYPROJECT_BLAS_LIBS "mkl_intel_ilp64_dll.lib"
#		"mkl_intel_thread_dll.lib"
#		"mkl_core_dll.lib"
#		"mkl_blacs_ilp64_dll.lib"
#		"libiomp5md.lib" CACHE STRING "The Intel's MKL Blas libs to use  with this project!"
#		)
#		SET(BLAS_INCLUDE_DIR ${INTEL_BLAS_INCLUDE_DIR})
#		SET(BLAS_LIBS "optimized" ${MYPROJECT_BLAS_LIBS} "debug" ${MYPROJECT_BLAS_LIBS})
#		SET(BLAS_LIB_DIR ${INTEL_LIB_DIR} ${INTEL_BLAS_LIB_DIR})
#		SET(MYPROJECT_CXX_FLAGS "/DMKL_ILP64" CACHE STRING "The special flags which are used for the Intel MKL Libs!")
#		SET(BLAS_NUM_THREADS 6 CACHE VALUE "The number of threads which should be used!")
#	endif(INTEL_BLAS_FOUND)
#endif(USE_INTEL_BLAS_FLAG)

#if(USE_GOTO_BLAS_FLAG)
#	FIND_PACKAGE(GotoBlas)
#	if(GotoBLAS_FOUND)
#		SET(MYPROJECT_BLAS_LIBS "libgoto2.lib" CACHE STRING "The Goto's Blas libs to use with this project!")
#		SET(BLAS_INCLUDE_DIR ${GotoBLAS_INCLUDE_DIR})
#		SET(BLAS_LIBS "optimized" ${MYPROJECT_BLAS_LIBS} "debug" ${MYPROJECT_BLAS_LIBS})
#		SET(BLAS_LIB_DIR ${GotoBLAS_LIB_DIR} )
#		SET(MYPROJECT_CXX_FLAGS "/DMKL_ILP64" CACHE STRING "The special flags which are used for the GOTO Headers as well!")
#		SET(BLAS_NUM_THREADS 6 CACHE VALUE "The number of threads which should be used!")
#	endif(GotoBLAS_FOUND)
#endif(USE_GOTO_BLAS_FLAG)


#if(USE_OPEN_BLAS_FLAG)
#	FIND_PACKAGE(OpenBlas)
#	if(OPENBLAS_FOUND)
#		SET(MYPROJECT_BLAS_LIBS ${OPENBLAS_LIB} CACHE STRING "The Goto's Blas libs to use with this project!")
#		SET(BLAS_INCLUDE_DIR ${OPENBLAS_INC})
#		SET(BLAS_LIBS "optimized" ${MYPROJECT_BLAS_LIBS} "debug" ${MYPROJECT_BLAS_LIBS})
#		SET(BLAS_LIB_DIR ${OPENBLAS_LIB_DIR} )
#		SET(MYPROJECT_CXX_FLAGS "/DMKL_ILP64" CACHE STRING "The special flags which are used for the GOTO Headers as well!")
#		SET(BLAS_NUM_THREADS 6 CACHE VALUE "The number of threads which should be used!")
#	endif()
#endif()


#Find MPI
FIND_PACKAGE(MPI REQUIRED)
set(CMAKE_CXX_COMPILE_FLAGS ${CMAKE_CXX_COMPILE_FLAGS} ${MPI_COMPILE_FLAGS})
set(CMAKE_CXX_LINK_FLAGS ${CMAKE_CXX_LINK_FLAGS} ${MPI_LINK_FLAGS})




SET(MYPROJECT_CXX_FLAGS "-std=c++0x" CACHE STRING "The project's own compiler flags")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MYPROJECT_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MYPROJECT_CXX_FLAGS}")
MESSAGE(STATUS "Added ${MYPROJECT_CXX_FLAGS} to CMAKE_CXX and CMAKE_C_FLAGS: ${CMAKE_CXX_FLAGS} and ${CMAKE_C_FLAGS}")

#configure_file (
#  "${COMMON_SOURCE_DIR}/inc/General/ConfigureFile.hpp.in"
#  "${PROJECT_BINARY_DIR}/ConfigureFile.hpp"
#)


#set common includes for all projects which need them

#INCLUDE_GAUSS_SEIDEL_CUDA(GaussSeidelGPU_SRC GaussSeidelGPU_INC COMMON_INCLUDE_DIR ${COMMON_SOURCE_DIR})
#INCLUDE_PROX_CUDA(ProxGPU_SRC ProxGPU_INC COMMON_INCLUDE_DIR ${COMMON_SOURCE_DIR})
#INCLUDE_VECTOR_ADD_CUDA(VectorAddGPU_SRC VectorAddGPU_INC COMMON_INCLUDE_DIR ${COMMON_SOURCE_DIR})
#INCLUDE_MATRIX_MULT_CUDA(MatrixMultGPU_SRC MatrixMultGPU_INC COMMON_INCLUDE_DIR ${COMMON_SOURCE_DIR})
#INCLUDE_MATRIX_VECTOR_MULT_CUDA(MatrixVectorMultGPU_SRC MatrixVectorMultGPU_INC COMMON_INCLUDE_DIR ${COMMON_SOURCE_DIR})
#INCLUDE_TESTS_CUDA(TestsGPU_SRC TestsGPU_INC COMMON_INCLUDE_DIR ${COMMON_SOURCE_DIR})
#INCLUDE_GENERAL_CUDA(GeneralCommonFiles_SRC GeneralCommonFiles_INC COMMON_INCLUDE_DIR ${COMMON_SOURCE_DIR})
#PRINTLIST("GENERAL: " "${GeneralCommonFiles_INC}")



set(COMMON_FILES_INC

)
set(COMMON_FILES_SRC

)

set(COMMON_INCLUDE_DIR
	${COMMON_INCLUDE_DIR}
   ${COMMON_SOURCE_DIR}/inc/
)



add_subdirectory(Projects/MpiTests)

add_subdirectory(Projects/PNetCDFTests)
