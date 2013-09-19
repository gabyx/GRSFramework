

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
#    ${SOURCE_DIR}/include/App/App.hpp
#    ${SOURCE_DIR}/include/App/MenuMouse.hpp
#    ${SOURCE_DIR}/include/App/OrbitCamera.hpp
)
SET( APP_SRC
#    ${SOURCE_DIR}/src/App/App.cpp
#    ${SOURCE_DIR}/src/App/MenuMouse.cpp
#    ${SOURCE_DIR}/src/App/OrbitCamera.cpp
    ${PROJECT_SOURCE_DIR}/src/App/main.cpp

)

SET(SINGELTON_INC
#    ${SOURCE_DIR}/include/Singeltons/Contexts/GuiContext.hpp
#    ${SOURCE_DIR}/include/Singeltons/Contexts/InputContext.hpp
#    ${SOURCE_DIR}/include/Singeltons/Contexts/RenderContext.hpp
    ${COMMONSOURCE_DIR}/include/Singeltons/FileManager.hpp
)

SET( SINGELTON_SRC
#    ${PROJECT_SOURCE_DIR}/src/Singeltons/Contexts/GuiContext.cpp
#    ${PROJECT_SOURCE_DIR}/src/Singeltons/Contexts/InputContext.cpp
#    ${PROJECT_SOURCE_DIR}/src/Singeltons/Contexts/RenderContext.cpp
    ${COMMONSOURCE_DIR}/src/Singeltons/FileManager.cpp
)

SET( STATES_INC
#    ${SOURCE_DIR}/include/States/AppState.hpp
#    ${SOURCE_DIR}/include/States/AppStateManager.hpp
#    ${SOURCE_DIR}/include/States/SimulationState.hpp
#    ${SOURCE_DIR}/include/States/PlaybackState.hpp
#    ${SOURCE_DIR}/include/States/VideoDropper.hpp
#    ${SOURCE_DIR}/include/States/SimulationManager/SimulationManagerBase.hpp
    ${COMMONSOURCE_DIR}/include/States/SimulationManager/SimulationManager.hpp
    ${COMMONSOURCE_DIR}/include/States/SimulationManager/SimulationManager.icc
#    ${SOURCE_DIR}/include/States/SimulationManager/PlaybackManagerBase.hpp
#    ${SOURCE_DIR}/include/States/SimulationManager/PlaybackManager.hpp
#    ${SOURCE_DIR}/include/States/SimulationManager/PlaybackManager.icc
#    ${SOURCE_DIR}/include/States/SimulationManager/PlaybackLoader.hpp
)
SET(STATES_SRC
#    ${SOURCE_DIR}/src/States/AppStateManager.cpp
#    ${SOURCE_DIR}/src/States/SimulationState.cpp
#    ${SOURCE_DIR}/src/States/PlaybackState.cpp
#    ${SOURCE_DIR}/src/States/SimulationManager/SimulationManagerBase.cpp
#    ${SOURCE_DIR}/src/States/SimulationManager/PlaybackManagerBase.cpp
)

SET(COMMON_INC
    ${SOURCE_DIR}/include/Common/TypeDefs.hpp
#    ${SOURCE_DIR}/include/Common/OgreMeshExtraction.hpp
#    ${SOURCE_DIR}/include/Common/CommonFunctionsOgre.hpp
    ${SOURCE_DIR}/include/Common/LogDefines.hpp
    ${COMMONSOURCE_DIR}/include/Common/foreach_macro.hpp
    ${SOURCE_DIR}/include/Common/AssertionDebug.hpp
    ${COMMONSOURCE_DIR}/include/Common/PlatformDefines.hpp
    ${COMMONSOURCE_DIR}/include/Common/CommonFunctions.hpp
    ${COMMONSOURCE_DIR}/include/Common/MeshData.hpp
    ${COMMONSOURCE_DIR}/include/Common/Singleton.hpp
    ${COMMONSOURCE_DIR}/include/Common/SimpleLogger.hpp
    ${COMMONSOURCE_DIR}/include/Common/MakeCoordinateSystem.hpp
    ${COMMONSOURCE_DIR}/include/Common/VectorToSkewMatrix.hpp
    ${COMMONSOURCE_DIR}/include/Common/MatrixHelpers.hpp
    ${COMMONSOURCE_DIR}/include/Common/QuaternionHelpers.hpp
    ${COMMONSOURCE_DIR}/include/Common/TypenameComparision.hpp
    ${COMMONSOURCE_DIR}/include/Common/MultiBodySimFile.hpp
    ${COMMONSOURCE_DIR}/include/Common/BinaryFile.hpp
    ${COMMONSOURCE_DIR}/include/Common/ApplicationCLOptions.hpp
)
SET(COMMON_SRC
#    ${SOURCE_DIR}/src/Common/OgreMeshExtraction.cpp
    ${COMMONSOURCE_DIR}/src/Common/CommonFunctions.cpp
#    ${SOURCE_DIR}/src/Common/CommonFunctionsOgre.cpp
    ${COMMONSOURCE_DIR}/src/Common/SimpleLogger.cpp
    ${SOURCE_DIR}/src/Common/TypeDefs.cpp
    ${COMMONSOURCE_DIR}/src/Common/MultiBodySimFile.cpp
)

SET(DYNAMICS_INC
	${COMMONSOURCE_DIR}/include/Dynamics/General/ExternalForces.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/General/DynamicsSystem.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/General/RigidBody.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/General/RigidBodyContainer.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/General/MoreauTimeStepper.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/General/TimeStepperSettings.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/General/InitialConditionBodies.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/General/AddGyroTermVisitor.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/General/InertiaTensorCalculations.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Collision/CollisionData.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Collision/CollisionSolver.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Collider.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Collision/ContactParams.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Collision/ContactTag.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Collision/ContactPercussion.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Collision/ContactParameterMap.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/SphereGeometry.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/PlaneGeometry.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/BoxGeometry.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/MeshGeometry.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/HalfspaceGeometry.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/ContactModels.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/ContactGraph.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/GeneralGraph.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/PercussionPool.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/InclusionSolverNT.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/InclusionSolverCO.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/InclusionSolverCONoG.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/ProxFunctions.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/ConvexSets.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/InclusionSolverSettings.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/FrontBackBuffer.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/StatePool.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/StatePoolVisBackFront.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/StateRingPoolVisBackFront.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/SharedBufferPlayback.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/StateRecorder.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/StateRecorderResampler.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/DynamicsState.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/RecorderSettings.hpp
)
SET(DYNAMICS_SRC
    ${COMMONSOURCE_DIR}/src/Dynamics/Collision/CollisionData.cpp
    ${COMMONSOURCE_DIR}/src/Dynamics/Collision/ContactTag.cpp
    ${COMMONSOURCE_DIR}/src/Dynamics/Collision/ContactParameterMap.cpp
)



SET(SYSTEMS_INC
    ${COMMONSOURCE_DIR}/include/Systems/SceneParser.hpp
#    ${SOURCE_DIR}/include/Systems/SceneParserOgre.hpp
    ${COMMONSOURCE_DIR}/include/Systems/SharedBufferDynSys.hpp
)

SET(SYSTEMS_SRC
)

# CUDA SOURCE INCLUDES ====================================================================
if(SIMULATIONFRAMEWORK_USE_CUDA)

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
INCLUDE(IncludeModulesCUDA)
INCLUDE_GENERAL_EXTERN_CUDA_ESSENTIAL(GeneralCommonFiles_SRC GeneralCommonFiles_INC CUDA_COMMON_INCLUDE_DIR ${CUDA_COMMON_SOURCE_DIR})
SET(CUDA_SRC ${GeneralCommonFiles_SRC})
SET(CUDA_INC ${GeneralCommonFiles_INC})
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
    ${DYNAMICS_INC}
    ${SYSTEMS_INC}
    ${CUDA_SRC}
)

SET(INCLUDE_FILES
    ${CONFIG_INC}
    ${APP_INC}
    ${SINGELTON_INC}
    ${STATES_INC}
    ${COMMON_INC}
    ${DYNAMICS_SRC}
    ${SYSTEMS_SRC}
    ${CUDA_INC}
)

SET(CMAKE_DEBUG_POSTFIX "_d")

#include directories
set(INCLUDE_DIRS

    #${NVIDIA_SDK_DIR}/common/inc
    #${NVIDIA_SDK_DIR}/../shared/inc
    ${BOOST_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIR}
#    ${TOON_INCLUDE_DIR}
#    ${OGRE_INCLUDE_DIRS}
#    ${OIS_INCLUDE_DIRS}
    #${CEGUI_INCLUDE_DIR}
#    ${STLSOFT_INCLUDE_DIR}
    ${TICPP_INCLUDE_DIRS}
    ${COLLISIONLIB_INCLUDE_DIRS}
    ${ASSIMP_INCLUDE_DIRS}

    ${PROJECT_BINARY_DIR}/ConfigFiles/

    ${PROJECT_SOURCE_DIR}/include/App

    #Linked Source
    #${SOURCE_DIR}/include/App
    ${SOURCE_DIR}/include/Common
    #${SOURCE_DIR}/include/Singeltons/Contexts
    #${SOURCE_DIR}/include/Singeltons/
    #${SOURCE_DIR}/include/States
    #${SOURCE_DIR}/include/States/SimulationManager
    #${SOURCE_DIR}/include/Systems

    ${COMMONSOURCE_DIR}/include/Common
    ${COMMONSOURCE_DIR}/include/Common/SRDelegates/include/
    ${COMMONSOURCE_DIR}/include/Dynamics/Buffers
    ${COMMONSOURCE_DIR}/include/Dynamics/Collision
    ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry
    ${COMMONSOURCE_DIR}/include/Dynamics/General
    ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion
    ${COMMONSOURCE_DIR}/include/Systems
    ${COMMONSOURCE_DIR}/include/Singeltons
    ${COMMONSOURCE_DIR}/include/States
    ${COMMONSOURCE_DIR}/include/States/SimulationManager

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

source_group("Common Source" FILES ${COMMON_FILES_SRC})
source_group("Common Header" FILES ${COMMON_FILES_INC})


message(STATUS "Add executable:...")
if(SIMULATIONFRAMEWORK_USE_CUDA)
CUDA_ADD_EXECUTABLE(${PROJECT_NAME}  ${SOURCE_FILES} ${INCLUDE_FILES} ${COMMON_FILES_SRC} ${COMMON_FILES_INC})
else()
ADD_EXECUTABLE(${PROJECT_NAME}  ${SOURCE_FILES} ${INCLUDE_FILES} ${COMMON_FILES_SRC} ${COMMON_FILES_INC})
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
