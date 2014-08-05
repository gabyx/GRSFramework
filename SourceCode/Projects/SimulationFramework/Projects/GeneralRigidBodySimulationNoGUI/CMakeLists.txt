

PROJECT(GeneralRigidBodySimulationNoGUI)

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


#Set Source Code (its in the GeneralRigidBodySimulation Folder)
SET(SOURCE_DIR "${PROJECT_SOURCE_DIR}/../GeneralRigidBodySimulation/")



SET(CONFIG_INC ${PROJECT_BINARY_DIR}/ConfigFiles/ConfigureFile.hpp)





SET( APP_INC
)
SET( APP_SRC
    ${PROJECT_SOURCE_DIR}/src/App/main.cpp

)

SET(SINGELTON_INC
)

SET( SINGELTON_SRC
)

SET( STATES_INC
    ${PROJECT_SOURCE_DIR}/include/States/SimulationManager/SimulationManager.hpp
)
SET(STATES_SRC
    ${PROJECT_SOURCE_DIR}/src/States/SimulationManager/SimulationManager.cpp
)

SET(COMMON_INC
    ${PROJECT_SOURCE_DIR}/include/Common/TypeDefs.hpp
    ${PROJECT_SOURCE_DIR}/include/Common/LogDefines.hpp
    ${SOURCE_DIR}/include/Common/AssertionDebug.hpp

)
SET(COMMON_SRC
    ${PROJECT_SOURCE_DIR}/src/Common/TypeDefs.cpp
)

SET(DYNAMICS_INC
)
SET(DYNAMICS_SRC
)

SET(SYSTEMS_INC
)

SET(SYSTEMS_SRC
)

# COMMON FILES INCLUDE ====================================================================
INCLUDE(IncludeModulesSimulationFramework)

INCLUDE_SIMULATION_FRAMEWORK_NOGUI( GLOBAL_COMMON_SRC  GLOBAL_COMMON_INC GLOBAL_COMMON_INC_DIR ${COMMONSOURCE_DIR})
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


# WRITE CONFIGURATION FILE
configure_file (
  ${SOURCE_DIR}/include/ConfigFiles/ConfigureFile.hpp.in
  ${PROJECT_BINARY_DIR}/ConfigFiles/ConfigureFile.hpp
)
# =======================

SET(SOURCE_FILES
    ${APP_SRC}
    ${SINGELTON_SRC}
    ${STATES_SRC}
    ${COMMON_SRC}
    ${DYNAMICS_SRC}
    ${SYSTEMS_SRC}
    ${CUDA_SRC}
    ${GLOBAL_COMMON_SRC}
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
)

SET(CMAKE_DEBUG_POSTFIX "_d")

#include directories
set(INCLUDE_DIRS

    ${Boost_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIR}
    ${TICPP_INCLUDE_DIRS}
    ${COLLISIONLIB_INCLUDE_DIRS}
    ${ASSIMP_INCLUDE_DIRS}

    ${PROJECT_BINARY_DIR}/ConfigFiles/

    ${PROJECT_SOURCE_DIR}/include/App

    #Only for LogDefines.hpp
    ${PROJECT_SOURCE_DIR}/include/Common # Needs to be previous to {SOURCE_DIR}/include/Common (because there is another LogDefines.hpp
    
    ${PROJECT_SOURCE_DIR}/include/States/SimulationManager/
    
    #Linked Source
    ${SOURCE_DIR}/include/Common
    
    ${SOURCE_DIR}/external/pugixml
    ${SOURCE_DIR}/external/pugixml

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



set(LINK_LIBS ${Boost_LIBRARIES} ${CUDA_cublas_LIBRARY} ${TICPP_LIBRARIES} ${COLLISIONLIB_LIBRARIES} ${ASSIMP_LIBRARIES})
PRINTLIST( "Linked libraries are:" "${LINK_LIBS}")


#source_group("Source Files\\App" FILES ${APP_SRC})
source_group("Source Files\\Singeltons" FILES ${SINGELTON_SRC})
source_group("Source Files\\States" FILES ${STATES_SRC})
source_group("Source Files\\Common" FILES ${COMMON_SRC})
source_group("Source Files\\Dynamics" FILES ${DYNAMICS_SRC})
source_group("Source Files\\Dynamics\\Cuda\\Prox" FILES ${ProxGPU_SRC})
source_group("Source Files\\Dynamics\\Cuda\\VectorAdd" FILES ${VectorAddGPU_SRC})
source_group("Source Files\\Dynamics\\Cuda\\MatrixVectorMult" FILES ${MatrixVectorMultGPU_SRC})
source_group("Source Files\\Dynamics\\Cuda\\General" FILES ${GeneralCommonFiles_SRC})
source_group("Source Files\\Systems" FILES ${SYSTEMS_SRC})

source_group("Header Files\\ConfigFiles" FILES ${CONFIG_INC})
#source_group("Header Files\\App" FILES ${APP_INC})
source_group("Header Files\\Singeltons" FILES ${SINGELTON_INC})
source_group("Header Files\\States" FILES ${STATES_INC})
source_group("Header Files\\Common" FILES ${COMMON_INC})
source_group("Header Files\\Dynamics" FILES ${DYNAMICS_INC})
source_group("Header Files\\Dynamics\\Cuda\\Prox" FILES ${ProxGPU_INC})
source_group("Header Files\\Dynamics\\Cuda\\VectorAdd" FILES ${VectorAddGPU_INC})
source_group("Header Files\\Dynamics\\Cuda\\MatrixVectorMult" FILES ${MatrixVectorMultGPU_INC})
source_group("Header Files\\Dynamics\\Cuda\\General" FILES ${GeneralCommonFiles_INC})
source_group("Header Files\\Systems" FILES ${SYSTEMS_INC})



message(STATUS "Add executable:...")
if(SIMULATIONFRAMEWORK_USE_CUDA)
CUDA_ADD_EXECUTABLE(${PROJECT_NAME}  ${SOURCE_FILES} ${INCLUDE_FILES})
else()
ADD_EXECUTABLE(${PROJECT_NAME}  ${SOURCE_FILES} ${INCLUDE_FILES} )
endif()
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS} )


SET(INSTALL_DIR_PROJECT ${PROJECT_BINARY_DIR}/bin/)
set_target_properties(${PROJECT_NAME} PROPERTIES
RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/bin/Debug
RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/bin/Release
)


# Install all files
if(WIN32)

#   install(TARGETS ${PROJECT_NAME}
#       RUNTIME DESTINATION ${INSTALL_DIR_PROJECT}/Debug
#       CONFIGURATIONS Debug)
#   install(TARGETS ${PROJECT_NAME}
#       RUNTIME DESTINATION ${INSTALL_DIR_PROJECT}/Release
#       CONFIGURATIONS Release)


    #install(FILES
                #${CMAKE_CURRENT_SOURCE_DIR}/resources/install/SceneFile.xml
        #DESTINATION ${INSTALL_DIR_PROJECT}/Release
        #CONFIGURATIONS Release
    #)
    #install(FILES
                #${CMAKE_CURRENT_SOURCE_DIR}/resources/install/SceneFile.xml
        #DESTINATION ${INSTALL_DIR_PROJECT}/Debug
        #CONFIGURATIONS Debug
    #)

    install(FILES
        ${COLLISIONLIB_REL}
        ${ASSIMP_REL}
        DESTINATION ${INSTALL_DIR_PROJECT}/Release
        CONFIGURATIONS Release
    )

    install(FILES
        ${COLLISIONLIB_DBG}
        ${ASSIMP_DBG}
        DESTINATION ${INSTALL_DIR_PROJECT}/Debug
        CONFIGURATIONS Debug
    )

   # as of sdk 1.7.2 we need to copy the boost dll's as well
   # because they're not linked statically (it worked with 1.7.1 though)
   install(FILES ${Boost_DATE_TIME_LIBRARY_RELEASE}
      ${Boost_THREAD_LIBRARY_RELEASE}
      DESTINATION ${INSTALL_DIR_PROJECT}/Release
      CONFIGURATIONS Release
   )
   install(FILES ${Boost_DATE_TIME_LIBRARY_DEBUG}
      ${Boost_THREAD_LIBRARY_DEBUG}
      DESTINATION ${INSTALL_DIR_PROJECT}/Debug
      CONFIGURATIONS Debug
   )

endif(WIN32)

if(UNIX)

#   install(TARGETS ${PROJECT_NAME}
#       RUNTIME DESTINATION ${INSTALL_DIR_PROJECT}/Debug
#       CONFIGURATIONS Debug)
#   install(TARGETS ${PROJECT_NAME}
#       RUNTIME DESTINATION ${INSTALL_DIR_PROJECT}/Release
#       CONFIGURATIONS Release)


    #install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/resources/install/SceneFile.xml
        #DESTINATION ${INSTALL_DIR_PROJECT}/Release
        #CONFIGURATIONS Release
    #)
    #install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/resources/install/SceneFile.xml
        #DESTINATION ${INSTALL_DIR_PROJECT}/Debug
        #CONFIGURATIONS Debug
    #)

endif(UNIX)

message(STATUS "=========================================================================")
