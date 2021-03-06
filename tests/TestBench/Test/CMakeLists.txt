
PROJECT(GeneralRigidBodySimulationNoGUI_TESTS)

message(STATUS "==============================Project: ${PROJECT_NAME} ==================================")

include(PrintListMacro)




#Set Source Code (its in the GeneralRigidBodySimulation Folder)
SET(SOURCE_DIR       "${PROJECT_SOURCE_DIR}/../../../projects/granularRigidBodySimulationGUI/")
set(COMMONSOURCE_DIR "${PROJECT_SOURCE_DIR}/../../../common")


SET(COMMON_IN
    ${PROJECT_SOURCE_DIR}/include/TestFunctions.hpp
)

SET(COMMON_SRC
    ${PROJECT_SOURCE_DIR}/src/main.cpp
)


# COMMON FILES INCLUDE ====================================================================
INCLUDE(IncludeModulesGRSFramework)

INCLUDE_SIMULATION_FRAMEWORK_NOGUI_TEST( GLOBAL_COMMON_SRC  GLOBAL_COMMON_INC GLOBAL_COMMON_INC_DIR ${COMMONSOURCE_DIR})
PRINTLIST("Global Common src files:" "${GLOBAL_COMMON_SRC}")
PRINTLIST("Global Common include files:" "${GLOBAL_COMMON_INC}")
# ===========================================================================================

# CUDA SOURCE INCLUDES ====================================================================
if(SIMULATIONFRAMEWORK_USE_CUDA)

INCLUDE(IncludeModulesCUDAModern)

# Configure File for the BUILD!
string(REPLACE "." "" CUDA_VERSION_INTEGER ${CUDA_VERSION})

# GPU Modules (JOR/SOR Lambda Dense Modules)
INCLUDE_PROX_CUDA(ProxGPU_SRC ProxGPU_INC CUDA_COMMON_INCLUDE_DIR ${CUDA_COMMON_SOURCE_DIR})
INCLUDE_VECTOR_ADD_CUDA(VectorAddGPU_SRC VectorAddGPU_INC CUDA_COMMON_INCLUDE_DIR ${CUDA_COMMON_SOURCE_DIR})
INCLUDE_MATRIX_VECTOR_MULT_CUDA(MatrixVectorMultGPU_SRC MatrixVectorMultGPU_INC CUDA_COMMON_INCLUDE_DIR ${CUDA_COMMON_SOURCE_DIR})
INCLUDE_GENERAL_EXTERN_CUDA(GeneralCommonFiles_SRC GeneralCommonFiles_INC CUDA_COMMON_INCLUDE_DIR ${CUDA_COMMON_SOURCE_DIR})

# GPU Modules (JOR Velocity Modules)
INCLUDE_JORPROX_VELOCITY_MODULE_EXTERN_CUDA( JorProxVelocity_SRC  JorProxVelocity_INC CUDA_COMMON_INCLUDE_DIR ${CUDA_COMMON_SOURCE_DIR})

SET(CUDA_SRC
   ${ProxGPU_SRC} ${VectorAddGPU_SRC} ${MatrixVectorMultGPU_SRC} ${GeneralCommonFiles_SRC} ${JorProxVelocity_SRC}
)
SET(CUDA_INC
   ${ProxGPU_INC} ${VectorAddGPU_INC} ${MatrixVectorMultGPU_INC} ${GeneralCommonFiles_INC} ${JorProxVelocity_INC}
)
else()

# Include nothing

endif()
# ===========================================================================================


#Configure ApproxMVBB =======================================================================
include(${ApproxMVBB_CMAKE_DIR}/DefineApproxMVBBSources.cmake)

# Define all MVBB Source files
INCLUDE_ALL_ApproxMVBB_SOURCE(  ApproxMVBB_SRC 
                                ApproxMVBB_INC 
                                ApproxMVBB_INC_DIRS 
                                ApproxMVBB_DEPENDING_TARGETS 
                                ${ApproxMVBB_ROOT_DIR} ${PROJECT_BINARY_DIR})

# ===========================================================================================


# FIND PCL Library
#find_package(PCL 1.8 REQUIRED COMPONENTS common io filters kdtree)
 #add_definitions(${PCL_DEFINITIONS})


include(GetGRSFVersionNumber)
GetGRSFVersionNumber()


# WRITE CONFIGURATION FILE
SET(CONFIG_INC ${PROJECT_BINARY_DIR}/include/GRSF/configFiles/ConfigureFile.hpp)
configure_file (
  ${SOURCE_DIR}/include/GRSF/configFiles/ConfigureFile.hpp.in
  ${CONFIG_INC}
)
# =======================

SET(SOURCE_FILES
    ${COMMON_SRC}
    ${CUDA_SRC}
    ${GLOBAL_COMMON_SRC}
    
    ${ApproxMVBB_SRC}
)

SET(INCLUDE_FILES
    ${CONFIG_INC}
    ${COMMON_INC}
    ${CUDA_INC}
    ${GLOBAL_COMMON_INC}
    
    ${ApproxMVBB_INC}
)

SET(CMAKE_DEBUG_POSTFIX "_d")

#include directories
set(INCLUDE_DIRS
    
    ${Boost_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIR}
    ${EIGEN_INCLUDE_DIR}/unsupported

    ${COLLISIONLIB_INCLUDE_DIRS}
    
    ${ASSIMP_INCLUDE_DIRS}
    
    ${PCL_INCLUDE_DIRS}
    ${PUGIXML_INCLUDE_DIR}
    ${Meta_INCLUDE_DIR}
    
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_BINARY_DIR}/include/
    
    #Linked Source
    ${COMMONSOURCE_DIR}/include/
        
    ${ApproxMVBB_INC_DIRS}
    
    
    ${GLOBAL_COMMON_INC_DIR}
    
    ${CUDA_COMMON_INCLUDE_DIR}
    
    
)

include_directories(${INCLUDE_DIRS})
PRINTLIST("Include directories are:" "${INCLUDE_DIRS}")


#link directories
set(LINK_DIRS
    ${Boost_LIBRARY_DIRS}
)
link_directories(${LINK_DIRS})
PRINTLIST( "Link directories are:" "${LINK_DIRS}")



set(LINK_LIBS ${Boost_LIBRARIES} ${CUDA_cublas_LIBRARY} ${COLLISIONLIB_LIBRARIES} ${ASSIMP_LIBRARIES} ${PUGIXML_LIBRARIES})
PRINTLIST( "Linked libraries are:" "${LINK_LIBS}")


message(STATUS "Add executable:...")
if(SIMULATIONFRAMEWORK_USE_CUDA)
CUDA_ADD_EXECUTABLE(${PROJECT_NAME}  ${SOURCE_FILES} ${INCLUDE_FILES})
else()
ADD_EXECUTABLE(${PROJECT_NAME}  ${SOURCE_FILES} ${INCLUDE_FILES} )
endif()
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS} )

ADD_DEPENDENCIES(${PROJECT_NAME} ${ApproxMVBB_DEPENDING_TARGETS})





# Add google tests

ADD_EXECUTABLE(GRSF_TEST_StringParse  ${PROJECT_SOURCE_DIR}/src/StringParseTest.cpp ${INCLUDE_FILES}   )
TARGET_LINK_LIBRARIES(GRSF_TEST_StringParse ${Boost_LIBRARIES} ${CUDA_cublas_LIBRARY} ${COLLISIONLIB_LIBRARIES} ${ASSIMP_LIBRARIES} ${PUGIXML_LIBRARIES} gtest gmock_main)






message(STATUS "=========================================================================")
