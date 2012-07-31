

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


#Set Source Code (its in the GeneralRigidBodySimulation Folder)
SET(SOURCE_DIR "${PROJECT_SOURCE_DIR}/../GeneralRigidBodySimulation/")

SET(CONFIG_INC ${PROJECT_BINARY_DIR}/ConfigFiles/ConfigureFile.hpp)

INCLUDE(IncludeModulesCUDA)

# Configure File for the BUILD!
string(REPLACE "." "" CUDA_VERSION_INTEGER ${CUDA_VERSION})
configure_file (
  ${PROJECT_SOURCE_DIR}/include/ConfigFiles/ConfigureFile.hpp.in
  ${PROJECT_BINARY_DIR}/ConfigFiles/ConfigureFile.hpp
)




SET( APP_INC
)
SET( APP_SRC
	${PROJECT_SOURCE_DIR}/src/App/main.cpp
)

SET(SOURCE_DIR "${SOURCE_DIR}/../GeneralRigidBodySimulation/")

SET(SINGELTON_INC
#	${SOURCE_DIR}/include/Singeltons/Contexts/GuiContext.hpp
#	${SOURCE_DIR}/include/Singeltons/Contexts/InputContext.hpp
#	${SOURCE_DIR}/include/Singeltons/Contexts/RenderContext.hpp
	${SOURCE_DIR}/include/Singeltons/FileManager.hpp
)

SET( SINGELTON_SRC
#	${SOURCE_DIR}/src/Singeltons/Contexts/GuiContext.cpp
#	${SOURCE_DIR}/src/Singeltons/Contexts/InputContext.cpp
#	${SOURCE_DIR}/src/Singeltons/Contexts/RenderContext.cpp
	${SOURCE_DIR}/src/Singeltons/FileManager.cpp
)

SET( STATES_INC
#	${SOURCE_DIR}/include/States/AppState.hpp
#	${SOURCE_DIR}/include/States/AppStateManager.hpp
#	${SOURCE_DIR}/include/States/SimulationState.hpp
#       ${SOURCE_DIR}/include/States/PlaybackState.hpp
#       ${SOURCE_DIR}/include/States/VideoDropper.hpp
	${SOURCE_DIR}/include/States/SimulationManager/SimulationManagerBase.hpp
#	${SOURCE_DIR}/include/States/SimulationManager/SimulationManagerGUI.hpp
#	${SOURCE_DIR}/include/States/SimulationManager/SimulationManagerGUI.icc
	${SOURCE_DIR}/include/States/SimulationManager/SimulationManager.hpp
	${SOURCE_DIR}/include/States/SimulationManager/SimulationManager.icc
        ${PROJECT_SOURCE_DIR}/include/States/SimulationManager/SimulationManagerMPI.hpp
        ${PROJECT_SOURCE_DIR}/include/States/SimulationManager/SimulationManagerMPI.icc
#	${SOURCE_DIR}/include/States/SimulationManager/PlaybackManagerBase.hpp
#	${SOURCE_DIR}/include/States/SimulationManager/PlaybackManager.hpp
#	${SOURCE_DIR}/include/States/SimulationManager/PlaybackManager.icc
#	${SOURCE_DIR}/include/States/SimulationManager/PlaybackLoader.hpp
)
SET(STATES_SRC
#	${SOURCE_DIR}/src/States/AppStateManager.cpp
#	${SOURCE_DIR}/src/States/SimulationState.cpp
#        ${SOURCE_DIR}/src/States/PlaybackState.cpp
#	${SOURCE_DIR}/src/States/SimulationManager/SimulationManagerBase.cpp
#	${SOURCE_DIR}/src/States/SimulationManager/PlaybackManagerBase.cpp
)

SET(COMMON_INC
	${SOURCE_DIR}/include/Common/foreach_macro.hpp
	${SOURCE_DIR}/include/Common/AssertionDebug.hpp
	${PROJECT_SOURCE_DIR}/include/Common/MPI/TypeDefs.hpp
	${SOURCE_DIR}/include/Common/PlatformDefines.hpp
	${SOURCE_DIR}/include/Common/CommonFunctions.hpp
	${SOURCE_DIR}/include/Common/MeshData.hpp
        ${SOURCE_DIR}/include/Common/Singleton.hpp
        ${SOURCE_DIR}/include/Common/SimpleLogger.hpp
	#${SOURCE_DIR}/include/Common/OgreMeshExtraction.hpp
        #${SOURCE_DIR}/include/Common/CommonFunctionsOgre.hpp
	${SOURCE_DIR}/include/Common/MakeCoordinateSystem.hpp
	${SOURCE_DIR}/include/Common/VectorToSkewMatrix.hpp
        ${SOURCE_DIR}/include/Common/MatrixHelpers.hpp
	${SOURCE_DIR}/include/Common/QuaternionHelpers.hpp
	${PROJECT_SOURCE_DIR}/include/Common/MPI/LogDefines.hpp
        ${SOURCE_DIR}/include/Common/TypenameComparision.hpp
	${SOURCE_DIR}/include/Common/MultiBodySimFile.hpp
        ${SOURCE_DIR}/include/Common/BinaryFile.hpp
        ${PROJECT_SOURCE_DIR}/include/Common/AABB.hpp
        ${PROJECT_SOURCE_DIR}/include/Common/MPIInformation.hpp
)
SET(COMMON_SRC
	#${SOURCE_DIR}/src/Common/OgreMeshExtraction.cpp
	${SOURCE_DIR}/src/Common/CommonFunctions.cpp
        #${SOURCE_DIR}/src/Common/CommonFunctionsOgre.cpp
        ${SOURCE_DIR}/src/Common/SimpleLogger.cpp
	${SOURCE_DIR}/src/Common/TypeDefs.cpp
)

SET(DYNAMICS_INC
        ${PROJECT_SOURCE_DIR}/include/Dynamics/General/MPI/DynamicsSystem.hpp
	${SOURCE_DIR}/include/Dynamics/General/RigidBody.hpp
	${PROJECT_SOURCE_DIR}/include/Dynamics/General/MPI/MoreauTimeStepper.hpp
        ${SOURCE_DIR}/include/Dynamics/General/TimeStepperSettings.hpp
        ${SOURCE_DIR}/include/Dynamics/General/InitialConditionBodies.hpp
        ${SOURCE_DIR}/include/Dynamics/General/InertiaTensorCalculations.hpp
	${SOURCE_DIR}/include/Dynamics/Collision/CollisionData.hpp
	${SOURCE_DIR}/include/Dynamics/Collision/CollisionSolver.hpp
	${SOURCE_DIR}/include/Dynamics/Collision/Collider.hpp
	${SOURCE_DIR}/include/Dynamics/Collision/ContactParams.hpp
	${SOURCE_DIR}/include/Dynamics/Collision/ContactTag.hpp
	${SOURCE_DIR}/include/Dynamics/Collision/ContactPercussion.hpp
	${SOURCE_DIR}/include/Dynamics/Collision/ContactParameterMap.hpp
	${SOURCE_DIR}/include/Dynamics/Collision/Geometry/SphereGeometry.hpp
	${SOURCE_DIR}/include/Dynamics/Collision/Geometry/PlaneGeometry.hpp
        ${SOURCE_DIR}/include/Dynamics/Collision/Geometry/BoxGeometry.hpp
        ${SOURCE_DIR}/include/Dynamics/Collision/Geometry/MeshGeometry.hpp
	${SOURCE_DIR}/include/Dynamics/Collision/Geometry/HalfspaceGeometry.hpp
	${SOURCE_DIR}/include/Dynamics/Inclusion/ContactModels.hpp
        ${SOURCE_DIR}/include/Dynamics/Inclusion/ContactGraph.hpp
        ${SOURCE_DIR}/include/Dynamics/Inclusion/GeneralGraph.hpp
	${SOURCE_DIR}/include/Dynamics/Inclusion/PercussionPool.hpp
#	${SOURCE_DIR}/include/Dynamics/Inclusion/InclusionSolverNT.hpp
#	${SOURCE_DIR}/include/Dynamics/Inclusion/InclusionSolverCO.hpp
        ${PROJECT_SOURCE_DIR}/include/Dynamics/Inclusion/MPI/InclusionSolverCONoG.hpp
	${SOURCE_DIR}/include/Dynamics/Inclusion/ProxFunctions.hpp
        ${SOURCE_DIR}/include/Dynamics/Inclusion/ConvexSets.hpp
	${SOURCE_DIR}/include/Dynamics/Inclusion/InclusionSolverSettings.hpp
	${SOURCE_DIR}/include/Dynamics/Buffers/FrontBackBuffer.hpp
	${SOURCE_DIR}/include/Dynamics/Buffers/StatePool.hpp
	${SOURCE_DIR}/include/Dynamics/Buffers/StatePoolVisBackFront.hpp
	${SOURCE_DIR}/include/Dynamics/Buffers/StateRingPoolVisBackFront.hpp
	${SOURCE_DIR}/include/Dynamics/Buffers/SharedBufferPlayback.hpp
	${SOURCE_DIR}/include/Dynamics/Buffers/StateRecorder.hpp
	${SOURCE_DIR}/include/Dynamics/Buffers/StateRecorderResampler.hpp
	${SOURCE_DIR}/include/Dynamics/Buffers/DynamicsState.hpp
)
SET(DYNAMICS_SRC
	${SOURCE_DIR}/src/Dynamics/Collision/CollisionData.cpp
	${SOURCE_DIR}/src/Dynamics/Collision/ContactTag.cpp
)



SET(SYSTEMS_INC
	${SOURCE_DIR}/include/Systems/SceneParser.hpp
        #${SOURCE_DIR}/include/Systems/SceneParserOgre.hpp
	${SOURCE_DIR}/include/Systems/SharedBufferDynSys.hpp
)

SET(SYSTEMS_SRC
)

# CUDA SOURCE INCLUDES ====================================================================
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
# ===========================================================================================


SET(SOURCE_FILES
	${COMMON_SOURCE_SRC}
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
	${COMMON_SOURCE_INC}
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
        ${PROJECT_BINARY_DIR}/ConfigFiles/
	${Boost_INCLUDE_DIR}
	${EIGEN_INCLUDE_DIR}
        ${TICPP_INCLUDE_DIR}
        ${COLLISIONLIB_INCLUDE_DIR}
        ${ASSIMP_INCLUDE_DIR}
        ${MPI_CXX_INCLUDE_PATH}

       ${PROJECT_SOURCE_DIR}/include
#	${PROJECT_SOURCE_DIR}/include/App
	${PROJECT_SOURCE_DIR}/include/Common
	${PROJECT_SOURCE_DIR}/include/Dynamics
       ${PROJECT_SOURCE_DIR}/include/Dynamics/Buffers
       ${PROJECT_SOURCE_DIR}/include/Dynamics/Collision
       ${PROJECT_SOURCE_DIR}/include/Dynamics/Collision/Geometry
       ${PROJECT_SOURCE_DIR}/include/Dynamics/General
       ${PROJECT_SOURCE_DIR}/include/Dynamics/Inclusion
	${PROJECT_SOURCE_DIR}/include/Systems
	${PROJECT_SOURCE_DIR}/include/Singeltons
	${PROJECT_SOURCE_DIR}/include/Singeltons/Contexts
	${PROJECT_SOURCE_DIR}/include/States
        ${PROJECT_SOURCE_DIR}/include/States/SimulationManager

	${SOURCE_DIR}/include
#	${SOURCE_DIR}/include/App
	${SOURCE_DIR}/include/Common
        ${SOURCE_DIR}/include/Common/SRDelegates/include/
	${SOURCE_DIR}/include/Dynamics
        ${SOURCE_DIR}/include/Dynamics/Buffers
        ${SOURCE_DIR}/include/Dynamics/Collision
        ${SOURCE_DIR}/include/Dynamics/Collision/Geometry
        ${SOURCE_DIR}/include/Dynamics/General
        ${SOURCE_DIR}/include/Dynamics/Inclusion
	${SOURCE_DIR}/include/Systems
	${SOURCE_DIR}/include/Singeltons
	${SOURCE_DIR}/include/Singeltons/Contexts
	${SOURCE_DIR}/include/States
        ${SOURCE_DIR}/include/States/SimulationManager
        ${CUDA_COMMON_INCLUDE_DIR}
        ${COMMON_SOURCE_INCLUDE_DIR}
	)

include_directories(${INCLUDE_DIRS})
PRINTLIST("Include directories are:" "${INCLUDE_DIRS}")



#link directories
set(LINK_DIRS
	${Boost_LIBRARY_DIRS}
)
link_directories(${LINK_DIRS})
PRINTLIST( "Link directories are:" "${LINK_DIRS}")



set(LINK_LIBS ${Boost_LIBRARIES} ${CUDA_cublas_LIBRARY} ${TICPP_LIBRARIES} ${COLLISIONLIB_LIBRARIES} ${ASSIMP_LIBRARIES}   ${MPI_CXX_LIBRARIES})
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
CUDA_ADD_EXECUTABLE(${PROJECT_NAME}  ${SOURCE_FILES} ${INCLUDE_FILES} ${COMMON_FILES_SRC} ${COMMON_FILES_INC})
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS} )


SET(INSTALL_DIR_PROJECT ${PROJECT_BINARY_DIR}/bin/)
set_target_properties(${PROJECT_NAME} PROPERTIES
RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/bin/Debug
RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/bin/Release
)


# Install all files
if(WIN32)

#	install(TARGETS ${PROJECT_NAME}
#		RUNTIME DESTINATION ${INSTALL_DIR_PROJECT}/Debug
#		CONFIGURATIONS Debug)
#	install(TARGETS ${PROJECT_NAME}
#		RUNTIME DESTINATION ${INSTALL_DIR_PROJECT}/Release
#		CONFIGURATIONS Release)


	install(FILES
                ${CMAKE_CURRENT_SOURCE_DIR}/resources/install/SceneFile.xml
		DESTINATION ${INSTALL_DIR_PROJECT}/Release
		CONFIGURATIONS Release
	)
	install(FILES
                ${CMAKE_CURRENT_SOURCE_DIR}/resources/install/SceneFile.xml
		DESTINATION ${INSTALL_DIR_PROJECT}/Debug
		CONFIGURATIONS Debug
	)

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

#	install(TARGETS ${PROJECT_NAME}
#		RUNTIME DESTINATION ${INSTALL_DIR_PROJECT}/Debug
#		CONFIGURATIONS Debug)
#	install(TARGETS ${PROJECT_NAME}
#		RUNTIME DESTINATION ${INSTALL_DIR_PROJECT}/Release
#		CONFIGURATIONS Release)


	install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/resources/install/SceneFile.xml
		DESTINATION ${INSTALL_DIR_PROJECT}/Release
		CONFIGURATIONS Release
	)
	install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/resources/install/SceneFile.xml
		DESTINATION ${INSTALL_DIR_PROJECT}/Debug
		CONFIGURATIONS Debug
	)

endif(UNIX)

message(STATUS "=========================================================================")
