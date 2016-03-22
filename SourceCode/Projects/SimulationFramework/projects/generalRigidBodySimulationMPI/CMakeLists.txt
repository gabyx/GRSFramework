

PROJECT(GeneralRigidBodySimulationMPI)

message(STATUS "==============================Project: ${PROJECT_NAME} ==================================")

include(PrintListMacro)

## EXTREMLY IMPORTANT ISSUES ############################################################################################################################################
# In MSVC 64bit mode, there is no problem with vectorization at all, because everything is aligned nicely!,
# If we move this code to UNIX 64bit it is not anymore like that, the Code of me, is not thoroughly supported for Vectorization in EIGEN
# TODO: To make it fully work with EIGEN_VECTORIZE as PREPROCESSOR MACRO (standart, set see App Log, have made an output!
# in 32bit on Windows and Unix, we need the EIGEN_MAKE_ALLOCATOR_NEW always defined, (almost done) and all std::vector and stuff we need special allocators! not done!
# TO make it still work on UNIX 64bit, turn EIGEN_DONT_ALIGN, for disabling completetly, and  EIGEN_DONT_ALIGN_STATICALLY for partly which still works... (or should)
# http://eigen.tuxfamily.org/dox-devel/TopicPreprocessorDirectives.html
#########################################################################################################################################################################


#Find MPI
FIND_PACKAGE(MPI REQUIRED)
set(CMAKE_CXX_COMPILE_FLAGS ${CMAKE_CXX_COMPILE_FLAGS} ${MPI_COMPILE_FLAGS})
set(CMAKE_CXX_LINK_FLAGS ${CMAKE_CXX_LINK_FLAGS} ${MPI_LINK_FLAGS})


SET( APP_INC
)
SET( APP_SRC
    ${PROJECT_SOURCE_DIR}/src/GRSF/App/main.cpp
)

SET(SINGELTON_INC
    ${GRSF_COMMON_SOURCE_DIR}/include/GRSF/Singeltons/FileManager.hpp
)

SET( SINGELTON_SRC
    ${GRSF_COMMON_SOURCE_DIR}/src/GRSF/Singeltons/FileManager.cpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Singeltons/MPIGlobalCommunicators.hpp
)

SET( STATES_INC
    ${PROJECT_SOURCE_DIR}/include/GRSF/States/SimulationManager/SimulationManagerMPI.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/States/SimulationManager/SimulationManagerMPI.icc
)
SET(STATES_SRC
)

SET(COMMON_INC
    ${PROJECT_SOURCE_DIR}/include/GRSF/Common/RedirectOutput.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Common/AssertionDebug.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Common/TypeDefs.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Common/LogDefines.hpp
)
SET(COMMON_SRC
    ${PROJECT_SOURCE_DIR}/src/GRSF/Common/TypeDefs.cpp
)

SET(DYNAMICS_INC
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/General/MPICommunicatorId.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/General/MPIMessageTag.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/General/MPIMessages.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/General/MPIDataTypes.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/General/MPIInformation.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/General/MPICommunication.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/General/MPITopology.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/General/MPITopologyGrid.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/General/MPITopologyKdTree.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/General/MPITopologyVisitors.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/General/MPITopologyBuilder.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/General/MPITopologyBuilderSettings.hpp

    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/General/MultiBodySimFileMPI.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/General/MultiBodySimFilePart.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/General/DynamicsSystemMPI.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/General/NeighbourMap.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/General/NeighbourDataBodyCommunication.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/General/BodyInfoMap.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/General/BodyCommunicator.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/General/RigidBodyGarbageCollector.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/General/RigidBodySolverDataMPI.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/General/RigidBodyMPI.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/General/MoreauTimeStepperMPI.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/Collision/CollisionSolverMPI.hpp

    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactGraphMPI.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactGraphMPI.icc
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactGraphNodeDataMPI.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactGraphVisitorsMPI.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/Inclusion/InclusionSolverCONoGMPI.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/Inclusion/InclusionCommunicator.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/Inclusion/InclusionCommunicator.icc
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/Inclusion/NeighbourDataInclusionCommunication.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/Inclusion/InclusionSolverSettingsMPI.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Dynamics/Buffers/StateRecorderMPI.hpp
)
SET(DYNAMICS_SRC
    ${PROJECT_SOURCE_DIR}/src/GRSF/Dynamics/General/MultiBodySimFileMPI.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/Dynamics/General/MultiBodySimFilePart.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/Dynamics/General/DynamicsSystemMPI.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/Dynamics/General/BodyCommunicator.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/Dynamics/Collision/CollisionSolverMPI.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/Dynamics/Inclusion/InclusionSolverCONoGMPI.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/Dynamics/Inclusion/ContactGraphNodeDataMPI.cpp

    ${PROJECT_SOURCE_DIR}/src/GRSF/Dynamics/General/MPIDataTypes.cpp
)

SET(SYSTEMS_INC
    ${PROJECT_SOURCE_DIR}/include/GRSF/Systems/SceneParserMPI.hpp
)

SET(SYSTEMS_SRC
)


# COMMON FILES INCLUDE ====================================================================
INCLUDE(IncludeModulesSimulationFramework)

INCLUDE_SIMULATION_FRAMEWORK_MPI( GLOBAL_COMMON_SRC  GLOBAL_COMMON_INC GLOBAL_COMMON_INC_DIR ${GRSF_COMMON_SOURCE_DIR})

# ===========================================================================================


# CUDA SOURCE INCLUDES ====================================================================
if(SIMULATIONFRAMEWORK_USE_CUDA)

MESSAGE(ERROR "CUDE does not work, write library to include!")
INCLUDE(IncludeModulesCUDA)

# Configure File for the BUILD!
string(REPLACE "." "" CUDA_VERSION_INTEGER ${CUDA_VERSION})

INCLUDE_PROX_CUDA(ProxGPU_SRC ProxGPU_INC CUDA_COMMON_INCLUDE_DIR ${CUDA_COMMON_SOURCE_DIR})
INCLUDE_VECTOR_ADD_CUDA(VectorAddGPU_SRC VectorAddGPU_INC CUDA_COMMON_INCLUDE_DIR ${CUDA_COMMON_SOURCE_DIR})
INCLUDE_MATRIX_VECTOR_MULT_CUDA(MatrixVectorMultGPU_SRC MatrixVectorMultGPU_INC CUDA_COMMON_INCLUDE_DIR ${CUDA_COMMON_SOURCE_DIR})
INCLUDE_GENERAL_EXTERN_CUDA(GeneralCommonFiles_SRC GeneralCommonFiles_INC CUDA_COMMON_INCLUDE_DIR ${CUDA_COMMON_SOURCE_DIR})


SET(CUDA_SRC
   ${ProxGPU_SRC} ${VectorAddGPU_SRC} ${MatrixVectorMultGPU_SRC} ${GeneralCommonFiles_SRC}
)
SET(CUDA_INC
   ${ProxGPU_INC} ${VectorAddGPU_INC} ${MatrixVectorMultGPU_INC} ${GeneralCommonFiles_INC}
)
else()
# Include nothing
endif()
# ===========================================================================================

#Configure ApproxMVBB =======================================================================
include(${ApproxMVBB_CMAKE_DIR}/DefineApproxMVBBSources.cmake)
#write config file for this source code
#set(ApproxMVBB_USE_DIFFERENT_HEADERS TRUE)
#set(ApproxMVBB_AssertionDebug_INCLUDE_FILE     "ApproxMVBB/Common//AssertionDebug.hpp")
#set(ApproxMVBB_Exception_INCLUDE_FILE          "ApproxMVBB/Common//Exception.hpp")
#set(ApproxMVBB_Platform_INCLUDE_FILE           "ApproxMVBB/Common/Platform.hpp")
#set(ApproxMVBB_StaticAssert_INCLUDE_FILE       "ApproxMVBB/Common/StaticAssert.hpp")
#set(ApproxMVBB_TypeDefs_INCLUDE_FILE           "ApproxMVBB/Common/TypeDefs.hpp" )
#set(ApproxMVBB_MyMatrixTypeDefs_INCLUDE_FILE   "ApproxMVBBWrapper/Common/MyMatrixTypeDefs.hpp")
#set(ApproxMVBB_AABB_INCLUDE_FILE               "GRSF/Dynamics/Collision/Geometry/AABB.hpp")
#set(ApproxMVBB_OOBB_INCLUDE_FILE               "GRSF/Dynamics/Collision/Geometry/OOBB.hpp" )

# Define all MVBB Source files   
INCLUDE_ALL_ApproxMVBB_SOURCE(  ApproxMVBB_SRC 
                                ApproxMVBB_INC 
                                ApproxMVBB_INC_DIRS 
                                ApproxMVBB_DEPENDING_TARGETS
                                ${ApproxMVBB_ROOT_DIR} ${PROJECT_BINARY_DIR})

# ===========================================================================================


# WRITE CONFIGURATION FILE
SET(CONFIG_INC ${PROJECT_BINARY_DIR}/include/GRSF/ConfigFiles/ConfigureFile.hpp)
configure_file (
  ${PROJECT_SOURCE_DIR}/include/GRSF/ConfigFiles/ConfigureFile.hpp.in
  ${CONFIG_INC}
)
#=========================


SET(SOURCE_FILES
    ${APP_SRC}
    ${SINGELTON_SRC}
    ${STATES_SRC}
    ${COMMON_SRC}
    ${DYNAMICS_SRC}
    ${SYSTEMS_SRC}
    ${CUDA_SRC}
    
    ${GLOBAL_COMMON_SRC}
    ${ApproxMVBB_SRC}
)

SET(INCLUDE_FILES
    ${CONFIG_INC}
    ${APP_INC}
    ${SINGELTON_INC}
    ${STATES_INC}
    ${COMMON_INC}
    ${DYNAMICS_INC}
    ${SYSTEMS_INC}
    ${CUDA_INC}
    
    ${GLOBAL_COMMON_INC}
    ${ApproxMVBB_INC}
)


SET(CMAKE_DEBUG_POSTFIX "_d")

#include directories
set(INCLUDE_DIRS
   
    ${GRSF_DEP_INCLUDE_DIRS}
    
    ${MPI_CXX_INCLUDE_PATH}

    ${PROJECT_BINARY_DIR}/include/
    ${PROJECT_SOURCE_DIR}/include/

    ${GLOBAL_COMMON_INC_DIR}
    ${CUDA_COMMON_INCLUDE_DIR}
    ${ApproxMVBB_INC_DIRS}
)

include_directories(${INCLUDE_DIRS})
#PRINTLIST("Include directories are:" "${INCLUDE_DIRS}")



#link directories
set(LINK_DIRS
    ${GRSF_DEP_LIBRARY_DIRS}
)
link_directories(${LINK_DIRS})
#PRINTLIST( "Link directories are:" "${LINK_DIRS}")



set(LINK_LIBS   ${GRSF_DEP_LIBRARIES}
                ${CUDA_cublas_LIBRARY}
                ${MPI_CXX_LIBRARIES}
                )
#PRINTLIST( "Linked libraries are:" "${LINK_LIBS}")



message(STATUS "Add executable:...")
if(SIMULATIONFRAMEWORK_USE_CUDA)
CUDA_ADD_EXECUTABLE(${PROJECT_NAME}  ${SOURCE_FILES} ${INCLUDE_FILES})
else()
ADD_EXECUTABLE(${PROJECT_NAME}  ${SOURCE_FILES} ${INCLUDE_FILES})
endif()

target_link_libraries(${PROJECT_NAME} ${LINK_LIBS} )
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin/
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/bin/Debug
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/bin/Release
)

ADD_DEPENDENCIES(${PROJECT_NAME} ${ApproxMVBB_DEPENDING_TARGETS})

# Add target for copying Resources Release and Debug
SET(ResourceDir ${CMAKE_CURRENT_SOURCE_DIR}/resources)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND 
          ${CMAKE_COMMAND}
          -DFolder=$<$<CONFIG:debug>:${ResourceDir}/debug>$<$<CONFIG:release>:${ResourceDir}/release>
          -DDestination=$<$<CONFIG:debug>:${PROJECT_BINARY_DIR}/bin/Debug>$<$<CONFIG:release>:${PROJECT_BINARY_DIR}/bin/Release>
          -P ${MODULE_FOLDER}/CopyFiles.cmake
)

message(STATUS "=========================================================================")
