

PROJECT(GRSFSimMPI)

message(STATUS "==============================Project: ${PROJECT_NAME} ==================================")

include(PrintListMacro)


#Find MPI
FIND_PACKAGE(MPI REQUIRED)
set(CMAKE_CXX_COMPILE_FLAGS ${CMAKE_CXX_COMPILE_FLAGS} ${MPI_COMPILE_FLAGS})
set(CMAKE_CXX_LINK_FLAGS ${CMAKE_CXX_LINK_FLAGS} ${MPI_LINK_FLAGS})


SET( APP_INC
)
SET( APP_SRC
    ${PROJECT_SOURCE_DIR}/src/GRSF/app/main.cpp
)

SET(SINGELTON_INC
    ${GRSF_COMMON_SOURCE_DIR}/include/GRSF/singeltons/FileManager.hpp
)

SET( SINGELTON_SRC
    ${GRSF_COMMON_SOURCE_DIR}/src/GRSF/singeltons/FileManager.cpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/singeltons/MPIGlobalCommunicators.hpp
)

SET( STATES_INC
    ${PROJECT_SOURCE_DIR}/include/GRSF/states/simulationManager/SimulationManagerMPI.hpp
    
)
SET(STATES_SRC
    ${PROJECT_SOURCE_DIR}/src/GRSF/states/simulationManager/SimulationManagerMPI.cpp
)

SET(COMMON_INC
    ${PROJECT_SOURCE_DIR}/include/GRSF/common/RedirectOutput.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/common/Asserts.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/common/TypeDefs.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/common/LogDefines.hpp
)
SET(COMMON_SRC
    ${PROJECT_SOURCE_DIR}/src/GRSF/common/TypeDefs.cpp
)

SET(DYNAMICS_INC
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/general/MPICommunicatorId.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/general/MPIMessageTag.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/general/MPIMessages.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/general/MPIDataTypes.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/general/MPIInformation.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/general/MPICommunication.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/general/MPITopology.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/general/MPITopologyGrid.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/general/MPITopologyKdTree.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/general/MPITopologyVisitors.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/general/MPITopologyBuilder.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/general/MPITopologyBuilderSettings.hpp

    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/general/MultiBodySimFileMPI.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/general/MultiBodySimFilePart.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/general/DynamicsSystemMPI.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/general/NeighbourMap.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/general/NeighbourDataBodyCommunication.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/general/BodyInfoMap.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/general/BodyCommunicator.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/general/RigidBodyGarbageCollector.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/general/RigidBodySolverDataMPI.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/general/RigidBodyMPI.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/general/MoreauTimeStepperMPI.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/collision/CollisionSolverMPI.hpp

    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactGraphMPI.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactGraphMPI.icc
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactGraphNodeDataMPI.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactGraphVisitorsMPI.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/inclusion/InclusionSolverCONoGMPI.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/inclusion/InclusionCommunicator.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/inclusion/InclusionCommunicator.icc
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/inclusion/NeighbourDataInclusionCommunication.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/inclusion/InclusionSolverSettingsMPI.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/buffers/StateRecorderMPI.hpp
)
SET(DYNAMICS_SRC
    ${PROJECT_SOURCE_DIR}/src/GRSF/dynamics/general/MultiBodySimFileMPI.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/dynamics/general/MultiBodySimFilePart.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/dynamics/general/DynamicsSystemMPI.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/dynamics/general/BodyCommunicator.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/dynamics/collision/CollisionSolverMPI.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/dynamics/inclusion/InclusionSolverCONoGMPI.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/dynamics/inclusion/ContactGraphNodeDataMPI.cpp

    ${PROJECT_SOURCE_DIR}/src/GRSF/dynamics/general/MPIDataTypes.cpp
)

SET(SYSTEMS_INC
    ${PROJECT_SOURCE_DIR}/include/GRSF/systems/SceneParserMPI.hpp
)

SET(SYSTEMS_SRC
)


# COMMON FILES INCLUDE ====================================================================
INCLUDE(IncludeModulesGRSFramework)

INCLUDE_SIMULATION_FRAMEWORK_MPI( GLOBAL_COMMON_SRC  GLOBAL_COMMON_INC GLOBAL_COMMON_INC_DIR ${GRSF_COMMON_SOURCE_DIR})

# ===========================================================================================


# CUDA SOURCE INCLUDES ====================================================================
if(GRSF_USE_CUDA)

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
#set(ApproxMVBB_AssertionDebug_INCLUDE_FILE     "ApproxMVBB/Common//Asserts.hpp")
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
SET(CONFIG_INC ${PROJECT_BINARY_DIR}/include/GRSF/configFiles/ConfigureFile.hpp)
configure_file (
  ${PROJECT_SOURCE_DIR}/include/GRSF/configFiles/ConfigureFile.hpp.in
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
PRINTLIST("Include directories are:" "${INCLUDE_DIRS}")



#link directories
set(LINK_DIRS
    ${GRSF_DEP_LIBRARY_DIRS}
)
link_directories(${LINK_DIRS})
PRINTLIST( "Link directories are:" "${LINK_DIRS}")



set(LINK_LIBS   ${GRSF_DEP_LIBRARIES}
                ${CUDA_cublas_LIBRARY}
                ${MPI_CXX_LIBRARIES}
                )
PRINTLIST( "Linked libraries are:" "${LINK_LIBS}")



message(STATUS "Add executable:  ${PPROJECT_NAME}")

if(GRSF_USE_CUDA)
  CUDA_ADD_EXECUTABLE(${PROJECT_NAME}  ${SOURCE_FILES} ${INCLUDE_FILES})
else()
  ADD_EXECUTABLE(${PROJECT_NAME}  ${SOURCE_FILES} ${INCLUDE_FILES})
endif()

target_link_libraries(${PROJECT_NAME} ${LINK_LIBS} )
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin/
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/bin/debug
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/bin/release
)

ADD_DEPENDENCIES(${PROJECT_NAME} ${ApproxMVBB_DEPENDING_TARGETS})

# Add target for copying Resources Release and Debug
SET(ResourceDir ${CMAKE_CURRENT_SOURCE_DIR}/resources)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND 
          ${CMAKE_COMMAND}
          -DFolder=$<$<CONFIG:debug>:${ResourceDir}/debug>$<$<CONFIG:release>:${ResourceDir}/release>
          -DDestination=$<$<CONFIG:debug>:${PROJECT_BINARY_DIR}/bin/Debug>$<$<CONFIG:release>:${PROJECT_BINARY_DIR}/bin/release>
          -P ${MODULE_FOLDER}/CopyFiles.cmake
)

message(STATUS "=========================================================================")
