

PROJECT(SimFileConverter)

message(STATUS "==============================Project: ${PROJECT_NAME} ==================================")

include(PrintListMacro)


#Set Source Code (its in the GeneralRigidBodySimulation Folder)
SET(SOURCE_DIR "${PROJECT_SOURCE_DIR}/../GeneralRigidBodySimulation/")

SET(CONFIG_INC ${PROJECT_BINARY_DIR}/ConfigFiles/ConfigureFile.hpp)






SET( APP_INC
)
SET( APP_SRC
    ${PROJECT_SOURCE_DIR}/src/App/main.cpp
)

SET(SOURCE_DIR "${SOURCE_DIR}/../GeneralRigidBodySimulation/")

#SET(SINGELTON_INC
    #${COMMONSOURCE_DIR}/include/Singeltons/FileManager.hpp
#)

#SET( SINGELTON_SRC
    #${COMMONSOURCE_DIR}/src/Singeltons/FileManager.cpp
#)

#SET( STATES_INC
    #${PROJECT_SOURCE_DIR}/include/States/SimulationManager/SimulationManagerMPI.hpp
    #${PROJECT_SOURCE_DIR}/include/States/SimulationManager/SimulationManagerMPI.icc
#)
#SET(STATES_SRC
#)

SET(COMMON_INC
	${COMMONSOURCE_DIR}/include/Common/Exception.hpp
    ${COMMONSOURCE_DIR}/include/Common/foreach_macro.hpp
    #${PROJECT_SOURCE_DIR}/include/Common/RedirectOutput.hpp
    ${SOURCE_DIR}/include/Common/AssertionDebug.hpp
    ${SOURCE_DIR}/include/Common/TypeDefs.hpp
    ${COMMONSOURCE_DIR}/include/Common/PlatformDefines.hpp
    ${COMMONSOURCE_DIR}/include/Common/CommonFunctions.hpp
    #${COMMONSOURCE_DIR}/include/Common/MeshData.hpp
    ${COMMONSOURCE_DIR}/include/Common/Singleton.hpp
    #${COMMONSOURCE_DIR}/include/Common/SimpleLogger.hpp
    #${PROJECT_SOURCE_DIR}/include/Common/CartesianGrid.hpp
    #${COMMONSOURCE_DIR}/include/Common/MakeCoordinateSystem.hpp
    #${COMMONSOURCE_DIR}/include/Common/VectorToSkewMatrix.hpp
    #${COMMONSOURCE_DIR}/include/Common/MatrixHelpers.hpp
    #${COMMONSOURCE_DIR}/include/Common/QuaternionHelpers.hpp
    ${PROJECT_SOURCE_DIR}/include/Common/LogDefines.hpp
    #${COMMONSOURCE_DIR}/include/Common/TypenameComparision.hpp
    ${COMMONSOURCE_DIR}/include/Common/MultiBodySimFile.hpp
    #${COMMONSOURCE_DIR}/include/Common/MultiBodySimFilePart.hpp
    #${PROJECT_SOURCE_DIR}/include/Common/MultiBodySimFileMPI.hpp
    ${COMMONSOURCE_DIR}/include/Common/BinaryFile.hpp
    #${PROJECT_SOURCE_DIR}/include/Common/MPIMessages.hpp
    #${PROJECT_SOURCE_DIR}/include/Common/MPIInformation.hpp
    #${PROJECT_SOURCE_DIR}/include/Common/MPICommunication.hpp
    #${PROJECT_SOURCE_DIR}/include/Common/MPITopology.hpp
    #${PROJECT_SOURCE_DIR}/include/Common/MPISerializationHelpersEigen.hpp
    ${PROJECT_SOURCE_DIR}/include/Common/ApplicationCLOptionsConverter.hpp
)
SET(COMMON_SRC
    ${COMMONSOURCE_DIR}/src/Common/CommonFunctions.cpp
    #${COMMONSOURCE_DIR}/src/Common/SimpleLogger.cpp
    ${SOURCE_DIR}/src/Common/TypeDefs.cpp
    ${COMMONSOURCE_DIR}/src/Common/MultiBodySimFile.cpp
    #${COMMONSOURCE_DIR}/src/Common/MultiBodySimFilePart.cpp
    #${PROJECT_SOURCE_DIR}/src/Common/MultiBodySimFileMPI.cpp
    ${COMMONSOURCE_DIR}/src/Common/ApplicationCLOptions.cpp
    #${PROJECT_SOURCE_DIR}/src/Common/CartesianGrid.cpp
)

SET(DYNAMICS_INC
    #${COMMONSOURCE_DIR}/include/Dynamics/General/ExternalForces.hpp
    #${PROJECT_SOURCE_DIR}/include/Dynamics/General/DynamicsSystemMPI.hpp
    #${PROJECT_SOURCE_DIR}/include/Dynamics/General/NeighbourMap.hpp
    #${PROJECT_SOURCE_DIR}/include/Dynamics/General/NeighbourDataBodyCommunication.hpp
    #${PROJECT_SOURCE_DIR}/include/Dynamics/General/BodyInfoMap.hpp
    #${PROJECT_SOURCE_DIR}/include/Dynamics/General/BodyCommunicator.hpp
    #${PROJECT_SOURCE_DIR}/include/Dynamics/General/RigidBodyGarbageCollector.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/General/RigidBody.hpp
    #${PROJECT_SOURCE_DIR}/include/Dynamics/General/RigidBodySolverDataMPI.hpp
    #${PROJECT_SOURCE_DIR}/include/Dynamics/General/RigidBodyMPI.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/General/RigidBodyId.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/General/RigidBodyContainer.hpp
    #${PROJECT_SOURCE_DIR}/include/Dynamics/General/MoreauTimeStepperMPI.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/General/TimeStepperSettings.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/General/InitialConditionBodies.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/General/InertiaTensorCalculations.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/General/AddGyroTermVisitor.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Collision/ContactFrame.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Collision/CollisionData.hpp
    #${PROJECT_SOURCE_DIR}/include/Dynamics/Collision/CollisionSolverMPI.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Collision/Collider.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Collision/CollisionFunctions.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Collision/ContactParameter.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Collision/ContactTag.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Collision/ContactPercussion.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Collision/ContactParameterMap.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/AABB.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/SphereGeometry.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/PlaneGeometry.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/BoxGeometry.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/MeshGeometry.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/HalfspaceGeometry.hpp
    #${PROJECT_SOURCE_DIR}/include/Dynamics/Collision/Geometry/MPISerializationHelpersGeometry.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/ContactModels.hpp
    #${PROJECT_SOURCE_DIR}/include/Dynamics/Inclusion/ContactGraphMPI.hpp
    #${PROJECT_SOURCE_DIR}/include/Dynamics/Inclusion/ContactGraphMPI.icc
    #${PROJECT_SOURCE_DIR}/include/Dynamics/Inclusion/ContactGraphNodeDataMPI.hpp
    #${PROJECT_SOURCE_DIR}/include/Dynamics/Inclusion/ContactGraphVisitorsMPI.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/InclusiCon/GeneralGraph.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/PercussionPool.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/InclusionSolverNT.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/InclusionSolverCO.hpp
    #${PROJECT_SOURCE_DIR}/include/Dynamics/Inclusion/InclusionSolverCONoGMPI.hpp
    #${PROJECT_SOURCE_DIR}/include/Dynamics/Inclusion/InclusionCommunicator.hpp
    #${PROJECT_SOURCE_DIR}/include/Dynamics/Inclusion/InclusionCommunicator.icc
    #${PROJECT_SOURCE_DIR}/include/Dynamics/Inclusion/NeighbourDataInclusionCommunication.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/ProxFunctions.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/ConvexSets.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/InclusionSolverSettings.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Buffers/FrontBackBuffer.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Buffers/StatePool.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Buffers/StatePoolVisBackFront.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Buffers/StateRingPoolVisBackFront.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Buffers/SharedBufferPlayback.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Buffers/StateRecorderBody.hpp
    #${PROJECT_SOURCE_DIR}/include/Dynamics/Buffers/StateRecorderMPI.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Buffers/StateRecorderProcess.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Buffers/StateRecorderResampler.hpp
    ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/DynamicsState.hpp
    #${COMMONSOURCE_DIR}/include/Dynamics/Buffers/RecorderSettings.hpp
)
#SET(DYNAMICS_SRC
    ##${COMMONSOURCE_DIR}/src/Dynamics/Collision/Collider.cpp
    ##${COMMONSOURCE_DIR}/src/Dynamics/Collision/CollisionData.cpp
    ##${COMMONSOURCE_DIR}/src/Dynamics/Collision/Geometry/BoxGeometry.cpp
    ##${COMMONSOURCE_DIR}/src/Dynamics/Collision/ContactTag.cpp
    ##${COMMONSOURCE_DIR}/src/Dynamics/Collision/ContactParameterMap.cpp
    ##${COMMONSOURCE_DIR}/src/Dynamics/General/InitialConditionBodies.cpp
    ##${PROJECT_SOURCE_DIR}/src/Dynamics/General/DynamicsSystemMPI.cpp
    ##${PROJECT_SOURCE_DIR}/src/Dynamics/General/BodyCommunicator.cpp
    ##${PROJECT_SOURCE_DIR}/src/Dynamics/Collision/CollisionSolverMPI.cpp
    ##${PROJECT_SOURCE_DIR}/src/Dynamics/Inclusion/InclusionSolverCONoGMPI.cpp
    ##${PROJECT_SOURCE_DIR}/src/Dynamics/Inclusion/ContactGraphNodeDataMPI.cpp
#)



#SET(SYSTEMS_INC
    #${PROJECT_SOURCE_DIR}/include/Systems/SceneParserMPI.hpp
    #${COMMONSOURCE_DIR}/include/Systems/SceneParser.hpp
    #${COMMONSOURCE_DIR}/include/Systems/SharedBufferDynSys.hpp
#)

SET(SYSTEMS_SRC
)



#=========================


SET(SOURCE_FILES
    ${APP_SRC}
    ${COMMON_SRC}
    ${DYNAMICS_SRC}
    ${CUDA_SRC}
)

SET(INCLUDE_FILES
    ${APP_INC}
    ${COMMON_INC}
    ${DYNAMICS_INC}
    ${CUDA_INC}
)


SET(CMAKE_DEBUG_POSTFIX "_d")

#include directories
set(INCLUDE_DIRS
    ${Boost_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIR}

	${COMMONSOURCE_DIR}/external/getoptpp/
    ${COMMONSOURCE_DIR}/external/getoptpp/src/

    ${PROJECT_SOURCE_DIR}/include/App
	${PROJECT_SOURCE_DIR}/include/Common

	${SOURCE_DIR}/include/Common

    ${COMMONSOURCE_DIR}/include/Common
    ${COMMONSOURCE_DIR}/include/Dynamics/Buffers
    #${COMMONSOURCE_DIR}/include/Dynamics/Collision
    #${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry
    ${COMMONSOURCE_DIR}/include/Dynamics/General
    #${COMMONSOURCE_DIR}/include/Dynamics/Inclusion
    #${COMMONSOURCE_DIR}/include/Systems
)

include_directories(${INCLUDE_DIRS})
PRINTLIST("Include directories are:" "${INCLUDE_DIRS}")



#link directories
set(LINK_DIRS
    ${Boost_LIBRARY_DIRS}
)
link_directories(${LINK_DIRS})
PRINTLIST( "Link directories are:" "${LINK_DIRS}")



set(LINK_LIBS ${Boost_LIBRARIES})
PRINTLIST( "Linked libraries are:" "${LINK_LIBS}")


source_group("Source Files\\App" FILES ${APP_SRC})
source_group("Source Files\\Common" FILES ${COMMON_SRC})
source_group("Source Files\\Dynamics" FILES ${DYNAMICS_SRC})

source_group("Header Files\\App" FILES ${APP_INC})
source_group("Header Files\\Common" FILES ${COMMON_INC})
source_group("Header Files\\Dynamics" FILES ${DYNAMICS_INC})


message(STATUS "Add executable:...")
ADD_EXECUTABLE(${PROJECT_NAME}  ${SOURCE_FILES} ${INCLUDE_FILES})
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

    #install(FILES
        #${COLLISIONLIB_REL}
        #${ASSIMP_REL}
        #DESTINATION ${INSTALL_DIR_PROJECT}/Release
        #CONFIGURATIONS Release
    #)

    #install(FILES
        #${COLLISIONLIB_DBG}
        #${ASSIMP_DBG}
        #DESTINATION ${INSTALL_DIR_PROJECT}/Debug
        #CONFIGURATIONS Debug
    #)

   # as of sdk 1.7.2 we need to copy the boost dll's as well
   # because they're not linked statically (it worked with 1.7.1 though)
   #install(FILES ${Boost_DATE_TIME_LIBRARY_RELEASE}
      #${Boost_THREAD_LIBRARY_RELEASE}
      #DESTINATION ${INSTALL_DIR_PROJECT}/Release
      #CONFIGURATIONS Release
   #)
   #install(FILES ${Boost_DATE_TIME_LIBRARY_DEBUG}
      #${Boost_THREAD_LIBRARY_DEBUG}
      #DESTINATION ${INSTALL_DIR_PROJECT}/Debug
      #CONFIGURATIONS Debug
   #)

endif(WIN32)

if(UNIX)

#   install(TARGETS ${PROJECT_NAME}
#       RUNTIME DESTINATION ${INSTALL_DIR_PROJECT}/Debug
#       CONFIGURATIONS Debug)
#   install(TARGETS ${PROJECT_NAME}
#       RUNTIME DESTINATION ${INSTALL_DIR_PROJECT}/Release
#       CONFIGURATIONS Release)


    #install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/resources/install/SceneFile.xml
    #    DESTINATION ${INSTALL_DIR_PROJECT}/Release
    #    CONFIGURATIONS Release
   # )
    #install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/resources/install/SceneFile.xml
    #    DESTINATION ${INSTALL_DIR_PROJECT}/Debug
    #    CONFIGURATIONS Debug
    #)

endif(UNIX)

message(STATUS "=========================================================================")
