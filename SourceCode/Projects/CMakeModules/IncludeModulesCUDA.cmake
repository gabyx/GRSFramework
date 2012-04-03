
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
	${PATH_TO_MODULES}/inc/General/UtilitiesCuda.hpp
	${PATH_TO_MODULES}/inc/General/MatrixCuda.hpp
	${PATH_TO_MODULES}/inc/General/FloatingPointType.hpp
	${PATH_TO_MODULES}/inc/General/GPUMutex.hpp
	${PATH_TO_MODULES}/inc/General/TemplateHelper.hpp
	${PATH_TO_MODULES}/inc/General/UtilitiesMatrixCuda.hpp
	${PATH_TO_MODULES}/inc/General/HandleError.hpp
	${PATH_TO_MODULES}/inc/General/AssertionDebug.hpp
	${PATH_TO_MODULES}/inc/General/AssertionDebugC.hpp
	${PATH_TO_MODULES}/inc/General/StaticAssert.hpp
	${PATH_TO_MODULES}/inc/General/TypenameComparision.hpp
	${PATH_TO_MODULES}/inc/General/FlopsCounting.hpp
	${PATH_TO_MODULES}/inc/General/GPUDefines.hpp
)
set(${SRC}
	${PATH_TO_MODULES}/src/General/Utilities.cpp
	${PATH_TO_MODULES}/src/General/UtilitiesCuda.cpp
   ${PATH_TO_MODULES}/src/General/UtilitiesMatrixCuda.cpp
)
set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}} ${PATH_TO_MODULES}/inc/General/)
endmacro(INCLUDE_GENERAL_CUDA)

MACRO(INCLUDE_GENERAL_EXTERN_CUDA SRC INC INCLUDE_DIRS PATH_TO_MODULES )
set(${INC}
	${PATH_TO_MODULES}/inc/General/Utilities.hpp
	${PATH_TO_MODULES}/inc/General/UtilitiesCuda.hpp
	${PATH_TO_MODULES}/inc/General/MatrixCuda.hpp
	${PATH_TO_MODULES}/inc/General/FloatingPointType.hpp
	${PATH_TO_MODULES}/inc/General/TemplateHelper.hpp
	${PATH_TO_MODULES}/inc/General/GPUMutex.hpp
	${PATH_TO_MODULES}/inc/General/UtilitiesMatrixCuda.hpp
	${PATH_TO_MODULES}/inc/General/HandleError.hpp
	${PATH_TO_MODULES}/inc/General/AssertionDebugC.hpp
	${PATH_TO_MODULES}/inc/General/StaticAssert.hpp
	${PATH_TO_MODULES}/inc/General/FlopsCounting.hpp
	${PATH_TO_MODULES}/inc/General/GPUDefines.hpp
)
set(${SRC}
	${PATH_TO_MODULES}/src/General/Utilities.cpp
	${PATH_TO_MODULES}/src/General/UtilitiesCuda.cpp
   ${PATH_TO_MODULES}/src/General/UtilitiesMatrixCuda.cpp
)
set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}} ${PATH_TO_MODULES}/inc/General/)
endmacro(INCLUDE_GENERAL_EXTERN_CUDA)