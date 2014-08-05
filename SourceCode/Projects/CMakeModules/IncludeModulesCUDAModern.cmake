
MACRO(INCLUDE_GAUSS_SEIDEL_CUDA SRC INC INCLUDE_DIRS COMMON_SOURCE_DIR)
set(${INC}
	${COMMON_SOURCE_DIR}/inc/GaussSeidelGPU/GaussSeidelGPU.hpp
    ${COMMON_SOURCE_DIR}/inc/GaussSeidelGPU/GaussSeidelTestVariant.hpp
	${COMMON_SOURCE_DIR}/inc/GaussSeidelGPU/KernelsGaussSeidel.cuh
)
set(${SRC}
	${COMMON_SOURCE_DIR}/src/GaussSeidelGPU/GaussSeidelGPU.cu
	${COMMON_SOURCE_DIR}/src/GaussSeidelGPU/GaussSeidelGPU.cpp
)
set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}} ${COMMON_SOURCE_DIR}/inc/GaussSeidelGPU/)
endmacro(INCLUDE_GAUSS_SEIDEL_CUDA)


MACRO(INCLUDE_PROX_CUDA SRC INC INCLUDE_DIRS COMMON_SOURCE_DIR)
set(${INC}
	${COMMON_SOURCE_DIR}/inc/ProxGPU/ProxTestVariant.hpp
	${COMMON_SOURCE_DIR}/inc/ProxGPU/ProxGPU.hpp
    ${COMMON_SOURCE_DIR}/inc/ProxGPU/ProxKernelSettings.hpp
    ${COMMON_SOURCE_DIR}/inc/ProxGPU/ProxSettings.hpp
	${COMMON_SOURCE_DIR}/inc/ProxGPU/KernelsProx.cuh
    ${COMMON_SOURCE_DIR}/inc/ProxGPU/SorProxGPUVariant.hpp
    ${COMMON_SOURCE_DIR}/inc/ProxGPU/JorProxGPUVariant.hpp
)
set(${SRC}
	${COMMON_SOURCE_DIR}/src/ProxGPU/ProxGPU.cu
	${COMMON_SOURCE_DIR}/src/ProxGPU/ProxGPU.cpp
)
set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}} ${COMMON_SOURCE_DIR}/inc/ProxGPU/)
endmacro(INCLUDE_PROX_CUDA)

MACRO(INCLUDE_VECTOR_ADD_CUDA SRC INC INCLUDE_DIRS COMMON_SOURCE_DIR)
set(${INC}
	${COMMON_SOURCE_DIR}/inc/VectorAddGPU/VectorAddGPU.hpp
	${COMMON_SOURCE_DIR}/inc/VectorAddGPU/KernelsVectorAdd.cuh
)
set(${SRC}
	${COMMON_SOURCE_DIR}/src/VectorAddGPU/VectorAddGPU.cu
	${COMMON_SOURCE_DIR}/src/VectorAddGPU/VectorAddGPU.cpp
)
set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}} ${COMMON_SOURCE_DIR}/inc/VectorAddGPU/)
endmacro(INCLUDE_VECTOR_ADD_CUDA)

MACRO(INCLUDE_MATRIX_MULT_CUDA SRC INC INCLUDE_DIRS COMMON_SOURCE_DIR)
set(${INC}
	${COMMON_SOURCE_DIR}/inc/MatrixMultGPU/MatrixMultGPU.hpp
	${COMMON_SOURCE_DIR}/inc/MatrixMultGPU/KernelsMatrixMult.cuh
)
set(${SRC}
	${COMMON_SOURCE_DIR}/src/MatrixMultGPU/MatrixMultGPU.cu
	${COMMON_SOURCE_DIR}/src/MatrixMultGPU/MatrixMultGPU.cpp
)
set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}} ${COMMON_SOURCE_DIR}/inc/MatrixMultGPU/)
endmacro(INCLUDE_MATRIX_MULT_CUDA)

MACRO(INCLUDE_MATRIX_VECTOR_MULT_CUDA SRC INC INCLUDE_DIRS COMMON_SOURCE_DIR)
set(${INC}
   ${COMMON_SOURCE_DIR}/inc/MatrixVectorMultGPU/MatrixVectorMultGPU.hpp
   ${COMMON_SOURCE_DIR}/inc/MatrixVectorMultGPU/MatrixVectorMultTestVariant.hpp
   ${COMMON_SOURCE_DIR}/inc/MatrixVectorMultGPU/KernelsMatrixVectorMult.cuh
)
set(${SRC}
	${COMMON_SOURCE_DIR}/src/MatrixVectorMultGPU/MatrixVectorMultGPU.cu
	${COMMON_SOURCE_DIR}/src/MatrixVectorMultGPU/MatrixVectorMultGPU.cpp
)
set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}} ${COMMON_SOURCE_DIR}/inc/MatrixVectorMultGPU/)
endmacro(INCLUDE_MATRIX_VECTOR_MULT_CUDA)

MACRO(INCLUDE_TESTS_CUDA SRC INC INCLUDE_DIRS COMMON_SOURCE_DIR)
set(${INC}
   	${COMMON_SOURCE_DIR}/inc/TestsGPU/TestsGPU.hpp
	${COMMON_SOURCE_DIR}/inc/TestsGPU/KernelsTests.cuh
)
set(${SRC}
	${COMMON_SOURCE_DIR}/src/TestsGPU/TestsGPU.cu
	${COMMON_SOURCE_DIR}/src/TestsGPU/TestsGPU.cpp
)
set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}} ${COMMON_SOURCE_DIR}/inc/TestsGPU/)
endmacro(INCLUDE_TESTS_CUDA)

MACRO(INCLUDE_GENERAL_CUDA SRC INC INCLUDE_DIRS COMMON_SOURCE_DIR )
set(${INC}
	${PROJECT_BINARY_DIR}/ConfigureFile.hpp
	${COMMON_SOURCE_DIR}/inc/General/Utilities.hpp
	${COMMON_SOURCE_DIR}/inc/General/FloatingPointType.hpp
	${COMMON_SOURCE_DIR}/inc/General/GPUMutex.hpp
	${COMMON_SOURCE_DIR}/inc/General/TypeTraitsHelper.hpp
	${COMMON_SOURCE_DIR}/inc/General/StaticAssert.hpp
	${COMMON_SOURCE_DIR}/inc/General/FlopsCounting.hpp
    ${COMMON_SOURCE_DIR}/inc/General/AssertionDebug.hpp
    ${COMMON_SOURCE_DIR}/inc/General/AssertionDebugC.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaEvent.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaTimer.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaAlloc.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaRefcounting.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaDeviceMemory.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaTypeDefs.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaDeviceGroup.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaContextGroup.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaContext.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaDevice.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaException.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaMemSupport.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaMatrixSupport.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaUtilities.hpp
    ${COMMON_SOURCE_DIR}/inc/General/Exception.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaError.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaMatrix.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaPrint.hpp
    
    ${COMMON_SOURCE_DIR}/inc/PerformanceTest/PerformanceTest.hpp
    ${COMMON_SOURCE_DIR}/inc/PerformanceTest/KernelTestMethod.hpp
    
    ${COMMON_SOURCE_DIR}/external/pugixml/src/pugixml.hpp
)
set(${SRC}
    ${COMMON_SOURCE_DIR}/src/General/CudaModern/CudaCompilerVersion.cu
    ${COMMON_SOURCE_DIR}/src/General/CudaModern/CudaMemSupport.cpp
    ${COMMON_SOURCE_DIR}/src/General/CudaModern/CudaDevice.cpp
    ${COMMON_SOURCE_DIR}/src/General/CudaModern/CudaDeviceGroup.cpp
    ${COMMON_SOURCE_DIR}/src/General/CudaModern/CudaContextGroup.cpp
    ${COMMON_SOURCE_DIR}/src/General/CudaModern/CudaContext.cpp
    ${COMMON_SOURCE_DIR}/src/General/CudaModern/CudaAlloc.cpp
    ${COMMON_SOURCE_DIR}/src/General/CudaModern/CudaUtilities.cpp
    ${COMMON_SOURCE_DIR}/src/General/CudaModern/CudaEvent.cpp
    ${COMMON_SOURCE_DIR}/src/General/CudaModern/CudaTimer.cpp
    ${COMMON_SOURCE_DIR}/src/General/CudaModern/CudaPrint.cpp
	${COMMON_SOURCE_DIR}/src/General/Utilities.cpp
    
    ${COMMON_SOURCE_DIR}/external/pugixml/src/pugixml.cpp
)
set(${INCLUDE_DIRS} 
    ${${INCLUDE_DIRS}} 
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern 
    ${COMMON_SOURCE_DIR}/inc/General/
    ${COMMON_SOURCE_DIR}/inc/PerformanceTest/
    ${COMMON_SOURCE_DIR}/external/pugixml/src/
)
endmacro(INCLUDE_GENERAL_CUDA)

MACRO(INCLUDE_GENERAL_EXTERN_CUDA SRC INC INCLUDE_DIRS COMMON_SOURCE_DIR )
set(${INC}
	${COMMON_SOURCE_DIR}/inc/General/Utilities.hpp
	${COMMON_SOURCE_DIR}/inc/General/FloatingPointType.hpp
	${COMMON_SOURCE_DIR}/inc/General/TypeTraitsHelper.hpp
	${COMMON_SOURCE_DIR}/inc/General/GPUMutex.hpp
	${COMMON_SOURCE_DIR}/inc/General/StaticAssert.hpp
	${COMMON_SOURCE_DIR}/inc/General/FlopsCounting.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaEvent.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaTimer.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaAlloc.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaRefcounting.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaDeviceMemory.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaDeviceMatrix.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaTypeDefs.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaDeviceGroup.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaContextGroup.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaContext.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaDevice.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaException.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaMemSupport.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaMatrixSupport.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaUtilities.hpp
    ${COMMON_SOURCE_DIR}/inc/General/Exception.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaError.hpp
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/CudaMatrix.hpp
    
    ${COMMON_SOURCE_DIR}/inc/PerformanceTest/PerformanceTest.hpp
    ${COMMON_SOURCE_DIR}/inc/PerformanceTest/KernelTestMethod.hpp
    
    #${COMMON_SOURCE_DIR}/external/pugixml/src/pugixml.hpp
)
set(${SRC}
    ${COMMON_SOURCE_DIR}/src/General/CudaModern/CudaCompilerVersion.cu
    ${COMMON_SOURCE_DIR}/src/General/CudaModern/CudaMemSupport.cpp
    ${COMMON_SOURCE_DIR}/src/General/CudaModern/CudaDevice.cpp
    ${COMMON_SOURCE_DIR}/src/General/CudaModern/CudaDeviceGroup.cpp
    ${COMMON_SOURCE_DIR}/src/General/CudaModern/CudaContextGroup.cpp
    ${COMMON_SOURCE_DIR}/src/General/CudaModern/CudaContext.cpp
    ${COMMON_SOURCE_DIR}/src/General/CudaModern/CudaAlloc.cpp
    ${COMMON_SOURCE_DIR}/src/General/CudaModern/CudaUtilities.cpp
    ${COMMON_SOURCE_DIR}/src/General/CudaModern/CudaEvent.cpp
    ${COMMON_SOURCE_DIR}/src/General/CudaModern/CudaTimer.cpp
	${COMMON_SOURCE_DIR}/src/General/Utilities.cpp
    
    #${COMMON_SOURCE_DIR}/external/pugixml/src/pugixml.cpp
)
set(${INCLUDE_DIRS} 
    ${${INCLUDE_DIRS}} 
    ${COMMON_SOURCE_DIR}/inc/General/CudaModern/
    ${COMMON_SOURCE_DIR}/inc/General/
    ${COMMON_SOURCE_DIR}/inc/PerformanceTest/
)
endmacro(INCLUDE_GENERAL_EXTERN_CUDA)



# Macros for the JOR Prox Velocity Module
MACRO(INCLUDE_JORPROX_VELOCITY_MODULE_EXTERN_CUDA SRC INC INCLUDE_DIRS COMMON_SOURCE_DIR )
set(${SRC}
	${COMMON_SOURCE_DIR}/src/JORProxVel/JORProxVelocityGPUModule.cpp
    
)
set(${INC}
    ${COMMON_SOURCE_DIR}/inc/JORProxVel/JORProxVelocityGPUModule.hpp
    ${COMMON_SOURCE_DIR}/inc/JORProxVel/GPUBufferOffsets.hpp
)
set(${INCLUDE_DIRS}
    ${${INCLUDE_DIRS}}
    ${COMMON_SOURCE_DIR}/inc/JORProxVel
)
ENDMACRO(INCLUDE_JORPROX_VELOCITY_MODULE_EXTERN_CUDA)
