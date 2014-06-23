
MACRO(INCLUDE_GAUSS_SEIDEL_CUDA SRC INC INCLUDE_DIRS PATH_TO_MODULES)
set(${INC}
	${PATH_TO_MODULES}/inc/GaussSeidelGPU/GaussSeidelGPU.hpp
    ${PATH_TO_MODULES}/inc/GaussSeidelGPU/GaussSeidelTestVariant.hpp
	${PATH_TO_MODULES}/inc/GaussSeidelGPU/KernelsGaussSeidel.cuh
)
set(${SRC}
	${PATH_TO_MODULES}/src/GaussSeidelGPU/GaussSeidelGPU.cu
	${PATH_TO_MODULES}/src/GaussSeidelGPU/GaussSeidelGPU.cpp
)
set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}} ${PATH_TO_MODULES}/inc/GaussSeidelGPU/)
endmacro(INCLUDE_GAUSS_SEIDEL_CUDA)


MACRO(INCLUDE_PROX_CUDA SRC INC INCLUDE_DIRS PATH_TO_MODULES)
set(${INC}
	${PATH_TO_MODULES}/inc/ProxGPU/ProxTestVariant.hpp
	${PATH_TO_MODULES}/inc/ProxGPU/ProxGPU.hpp
    ${PATH_TO_MODULES}/inc/ProxGPU/ProxKernelSettings.hpp
    ${PATH_TO_MODULES}/inc/ProxGPU/ProxSettings.hpp
	${PATH_TO_MODULES}/inc/ProxGPU/KernelsProx.cuh
    ${PATH_TO_MODULES}/inc/ProxGPU/SorProxGPUVariant.hpp
    ${PATH_TO_MODULES}/inc/ProxGPU/JorProxGPUVariant.hpp
)
set(${SRC}
	${PATH_TO_MODULES}/src/ProxGPU/ProxGPU.cu
	${PATH_TO_MODULES}/src/ProxGPU/ProxGPU.cpp
)
set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}} ${PATH_TO_MODULES}/inc/ProxGPU/)
endmacro(INCLUDE_PROX_CUDA)

MACRO(INCLUDE_VECTOR_ADD_CUDA SRC INC INCLUDE_DIRS PATH_TO_MODULES)
set(${INC}
	${PATH_TO_MODULES}/inc/VectorAddGPU/VectorAddGPU.hpp
	${PATH_TO_MODULES}/inc/VectorAddGPU/KernelsVectorAdd.cuh
)
set(${SRC}
	${PATH_TO_MODULES}/src/VectorAddGPU/VectorAddGPU.cu
	${PATH_TO_MODULES}/src/VectorAddGPU/VectorAddGPU.cpp
)
set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}} ${PATH_TO_MODULES}/inc/VectorAddGPU/)
endmacro(INCLUDE_VECTOR_ADD_CUDA)

MACRO(INCLUDE_MATRIX_MULT_CUDA SRC INC INCLUDE_DIRS PATH_TO_MODULES)
set(${INC}
	${PATH_TO_MODULES}/inc/MatrixMultGPU/MatrixMultGPU.hpp
	${PATH_TO_MODULES}/inc/MatrixMultGPU/KernelsMatrixMult.cuh
)
set(${SRC}
	${PATH_TO_MODULES}/src/MatrixMultGPU/MatrixMultGPU.cu
	${PATH_TO_MODULES}/src/MatrixMultGPU/MatrixMultGPU.cpp
)
set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}} ${PATH_TO_MODULES}/inc/MatrixMultGPU/)
endmacro(INCLUDE_MATRIX_MULT_CUDA)

MACRO(INCLUDE_MATRIX_VECTOR_MULT_CUDA SRC INC INCLUDE_DIRS PATH_TO_MODULES)
set(${INC}
   ${PATH_TO_MODULES}/inc/MatrixVectorMultGPU/MatrixVectorMultGPU.hpp
   ${PATH_TO_MODULES}/inc/MatrixVectorMultGPU/MatrixVectorMultTestVariant.hpp
   ${PATH_TO_MODULES}/inc/MatrixVectorMultGPU/KernelsMatrixVectorMult.cuh
)
set(${SRC}
	${PATH_TO_MODULES}/src/MatrixVectorMultGPU/MatrixVectorMultGPU.cu
	${PATH_TO_MODULES}/src/MatrixVectorMultGPU/MatrixVectorMultGPU.cpp
)
set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}} ${PATH_TO_MODULES}/inc/MatrixVectorMultGPU/)
endmacro(INCLUDE_MATRIX_VECTOR_MULT_CUDA)

MACRO(INCLUDE_TESTS_CUDA SRC INC INCLUDE_DIRS PATH_TO_MODULES)
set(${INC}
   	${PATH_TO_MODULES}/inc/TestsGPU/TestsGPU.hpp
	${PATH_TO_MODULES}/inc/TestsGPU/KernelsTests.cuh
)
set(${SRC}
	${PATH_TO_MODULES}/src/TestsGPU/TestsGPU.cu
	${PATH_TO_MODULES}/src/TestsGPU/TestsGPU.cpp
)
set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}} ${PATH_TO_MODULES}/inc/TestsGPU/)
endmacro(INCLUDE_TESTS_CUDA)

MACRO(INCLUDE_GENERAL_CUDA SRC INC INCLUDE_DIRS PATH_TO_MODULES )
set(${INC}
	${PROJECT_BINARY_DIR}/ConfigureFile.hpp
	${PATH_TO_MODULES}/inc/General/Utilities.hpp
	${PATH_TO_MODULES}/inc/General/FloatingPointType.hpp
	${PATH_TO_MODULES}/inc/General/GPUMutex.hpp
	${PATH_TO_MODULES}/inc/General/TypeTraitsHelper.hpp
	${PATH_TO_MODULES}/inc/General/StaticAssert.hpp
	${PATH_TO_MODULES}/inc/General/FlopsCounting.hpp
    ${PATH_TO_MODULES}/inc/General/AssertionDebug.hpp
    ${PATH_TO_MODULES}/inc/General/AssertionDebugC.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaEvent.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaTimer.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaAlloc.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaRefcounting.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaDeviceMemory.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaTypeDefs.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaDeviceGroup.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaContextGroup.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaContext.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaDevice.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaException.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaMemSupport.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaMatrixSupport.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaUtilities.hpp
    ${PATH_TO_MODULES}/inc/General/Exception.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaError.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaMatrix.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaPrint.hpp
)
set(${SRC}
    ${PATH_TO_MODULES}/src/General/CudaModern/CudaCompilerVersion.cu
    ${PATH_TO_MODULES}/src/General/CudaModern/CudaMemSupport.cpp
    ${PATH_TO_MODULES}/src/General/CudaModern/CudaDevice.cpp
    ${PATH_TO_MODULES}/src/General/CudaModern/CudaDeviceGroup.cpp
    ${PATH_TO_MODULES}/src/General/CudaModern/CudaContextGroup.cpp
    ${PATH_TO_MODULES}/src/General/CudaModern/CudaContext.cpp
    ${PATH_TO_MODULES}/src/General/CudaModern/CudaAlloc.cpp
    ${PATH_TO_MODULES}/src/General/CudaModern/CudaUtilities.cpp
    ${PATH_TO_MODULES}/src/General/CudaModern/CudaEvent.cpp
    ${PATH_TO_MODULES}/src/General/CudaModern/CudaTimer.cpp
    ${PATH_TO_MODULES}/src/General/CudaModern/CudaPrint.cpp
	${PATH_TO_MODULES}/src/General/Utilities.cpp
)
set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}} ${PATH_TO_MODULES}/inc/General/CudaModern ${PATH_TO_MODULES}/inc/General/)
endmacro(INCLUDE_GENERAL_CUDA)

MACRO(INCLUDE_GENERAL_EXTERN_CUDA SRC INC INCLUDE_DIRS PATH_TO_MODULES )
set(${INC}
	${PATH_TO_MODULES}/inc/General/Utilities.hpp
	${PATH_TO_MODULES}/inc/General/FloatingPointType.hpp
	${PATH_TO_MODULES}/inc/General/TypeTraitsHelper.hpp
	${PATH_TO_MODULES}/inc/General/GPUMutex.hpp
	${PATH_TO_MODULES}/inc/General/StaticAssert.hpp
	${PATH_TO_MODULES}/inc/General/FlopsCounting.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaEvent.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaTimer.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaAlloc.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaRefcounting.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaDeviceMemory.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaDeviceMatrix.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaTypeDefs.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaDeviceGroup.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaContextGroup.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaContext.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaDevice.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaException.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaMemSupport.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaMatrixSupport.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaUtilities.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/Exception.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaError.hpp
    ${PATH_TO_MODULES}/inc/General/CudaModern/CudaMatrix.hpp
)
set(${SRC}
    ${PATH_TO_MODULES}/src/General/CudaModern/CudaCompilerVersion.cu
    ${PATH_TO_MODULES}/src/General/CudaModern/CudaMemSupport.cpp
    ${PATH_TO_MODULES}/src/General/CudaModern/CudaDevice.cpp
    ${PATH_TO_MODULES}/src/General/CudaModern/CudaDeviceGroup.cpp
    ${PATH_TO_MODULES}/src/General/CudaModern/CudaContextGroup.cpp
    ${PATH_TO_MODULES}/src/General/CudaModern/CudaContext.cpp
    ${PATH_TO_MODULES}/src/General/CudaModern/CudaAlloc.cpp
    ${PATH_TO_MODULES}/src/General/CudaModern/CudaUtilities.cpp
    ${PATH_TO_MODULES}/src/General/CudaModern/CudaEvent.cpp
    ${PATH_TO_MODULES}/src/General/CudaModern/CudaTimer.cpp
	${PATH_TO_MODULES}/src/General/Utilities.cpp
)
set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}} ${PATH_TO_MODULES}/inc/General/)
endmacro(INCLUDE_GENERAL_EXTERN_CUDA)

