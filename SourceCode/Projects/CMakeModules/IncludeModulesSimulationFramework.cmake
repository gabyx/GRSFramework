MACRO(INCLUDE_SIMULATION_FRAMEWORK_GUI SRC INC INCLUDE_DIRS COMMONSOURCE_DIR )

    SET(${INC}
        ${COMMONSOURCE_DIR}/include/GRSF/Singeltons/FileManager.hpp

        ${COMMONSOURCE_DIR}/include/GRSF/Common/PlatformDefines.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/CommonFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/BitCount.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/Singleton.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/SimpleLogger.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/TypenameComparision.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/BinaryFile.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/ApplicationCLOptions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/Delegates.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/CPUTimer.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/XMLMacros.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/foreach_macro.hpp
        
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/MyMatrixTypeDefs.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/LayoutConfigTypeDefs.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/MeshData.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/MultiBodySimFile.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/MakeCoordinateSystem.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/VectorToSkewMatrix.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/MatrixHelpers.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/QuaternionHelpers.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/DynamicsSystem.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/DynamicsSystemBase.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/ExternalForces.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/RigidBodyId.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/RigidBody.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/RigidBodySolverData.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/RigidBodyContainer.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/MoreauTimeStepper.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/TimeStepperBase.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/TimeStepperSettings.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/InitialConditionBodies.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/InertiaTensorCalculations.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/AddGyroTermVisitor.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/ContactFrame.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/CollisionData.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/CollisionSolver.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/CollisionFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Collider.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/ContactTag.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/ContactPercussion.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/AABB.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/Ray.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/SphereGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/PlaneGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/BoxGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/MeshGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/HalfspaceGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactParameterMap.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactParameter.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactModels.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactGraph.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactGraphVisitors.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactGraphNodeData.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactGraphVisitors.hpp not yet made!
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/GeneralGraph.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/PercussionPool.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/InclusionSolverNT.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/InclusionSolverCO.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/InclusionSolverCONoG.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ProxFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ConvexSets.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/InclusionSolverSettings.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/FrontBackBuffer.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/StateRecorder.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/StateRecorderResampler.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/DynamicsState.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/RigidBodyState.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/RecorderSettings.hpp

        ${COMMONSOURCE_DIR}/include/GRSF/Systems/SceneParser.hpp

        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/StatePool.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/StatePoolVisBackFront.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/StateRingPoolVisBackFront.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/SharedBufferPlayback.hpp

        ${COMMONSOURCE_DIR}/include/GRSF/Systems/SharedBufferDynSys.hpp
        
        
        ${COMMONSOURCE_DIR}/external/pugixml/src/pugixml.hpp
        ${COMMONSOURCE_DIR}/external/rtnorm/rtnorm.hpp
    )


    SET(${SRC}
        ${COMMONSOURCE_DIR}/src/GRSF/Singeltons/FileManager.cpp

        ${COMMONSOURCE_DIR}/src/GRSF/Common/CommonFunctions.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Common/SimpleLogger.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Common/ApplicationCLOptions.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Common/BitCount.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/General/MyMatrixTypeDefs.cpp

        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/General/MultiBodySimFile.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Inclusion/InclusionSolverCONoG.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Inclusion/InclusionSolverCO.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Collision/CollisionSolver.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/General/MoreauTimeStepper.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Collision/Collider.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Buffers/StateRecorder.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Buffers/StatePoolVisBackFront.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/General/DynamicsSystem.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/General/DynamicsSystemBase.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/General/InitialConditionBodies.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Collision/CollisionData.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Collision/ContactTag.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Collision/Geometry/BoxGeometry.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Collision/Geometry/Ray.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Inclusion/ConvexSets.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Inclusion/ContactModels.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Inclusion/ContactParameterMap.cpp

        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Buffers/StatePoolVisBackFront.cpp
        
        
        ${COMMONSOURCE_DIR}/external/pugixml/src/pugixml.cpp
    )

    set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}}
        ${COMMONSOURCE_DIR}/include/
        ${COMMONSOURCE_DIR}/external/tinyformat/
        ${COMMONSOURCE_DIR}/external/SRDelegates/include/
        ${COMMONSOURCE_DIR}/external/FastFunc/include/
        ${COMMONSOURCE_DIR}/external/getoptpp/
        ${COMMONSOURCE_DIR}/external/getoptpp/src/
        ${COMMONSOURCE_DIR}/external/pugixml/src
        ${COMMONSOURCE_DIR}/external/
    )
endmacro(INCLUDE_SIMULATION_FRAMEWORK_GUI)


MACRO(INCLUDE_SIMULATION_FRAMEWORK_NOGUI SRC INC INCLUDE_DIRS COMMONSOURCE_DIR )
    INCLUDE_SIMULATION_FRAMEWORK_GUI( ${SRC} ${INC} ${INCLUDE_DIRS} ${COMMONSOURCE_DIR} )
endmacro(INCLUDE_SIMULATION_FRAMEWORK_NOGUI)

MACRO(INCLUDE_SIMULATION_FRAMEWORK_MPI SRC INC INCLUDE_DIRS COMMONSOURCE_DIR )
    
    SET(${INC}
        ${COMMONSOURCE_DIR}/include/GRSF/Singeltons/FileManager.hpp

        ${COMMONSOURCE_DIR}/include/GRSF/Common/PlatformDefines.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/CommonFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/BitCount.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/Singleton.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/SimpleLogger.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/TypenameComparision.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/BinaryFile.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/ApplicationCLOptions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/Delegates.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/CPUTimer.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/XMLMacros.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/foreach_macro.hpp
        
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/MyMatrixTypeDefs.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/LayoutConfigTypeDefs.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/MeshData.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/MultiBodySimFile.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/MakeCoordinateSystem.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/VectorToSkewMatrix.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/MatrixHelpers.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/QuaternionHelpers.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/DynamicsSystem.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/DynamicsSystemBase.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/ExternalForces.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/RigidBodyId.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/RigidBody.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/RigidBodySolverData.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/RigidBodyContainer.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/MoreauTimeStepper.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/TimeStepperBase.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/TimeStepperSettings.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/InitialConditionBodies.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/InertiaTensorCalculations.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/AddGyroTermVisitor.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/ContactFrame.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/CollisionData.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/CollisionSolver.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/CollisionFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Collider.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/ContactTag.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/ContactPercussion.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/AABB.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/Ray.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/SphereGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/PlaneGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/BoxGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/MeshGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/HalfspaceGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactParameterMap.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactParameter.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactModels.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactGraph.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactGraphNodeData.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactGraphVisitors.hpp not yet made!
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/GeneralGraph.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/PercussionPool.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/InclusionSolverNT.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/InclusionSolverCO.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/InclusionSolverCONoG.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ProxFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ConvexSets.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/InclusionSolverSettings.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/FrontBackBuffer.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/StateRecorder.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/StateRecorderResampler.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/DynamicsState.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/RigidBodyState.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/RecorderSettings.hpp

        ${COMMONSOURCE_DIR}/include/GRSF/Systems/SceneParser.hpp

        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/StatePool.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/StatePoolVisBackFront.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/StateRingPoolVisBackFront.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/SharedBufferPlayback.hpp

        ${COMMONSOURCE_DIR}/include/GRSF/Systems/SharedBufferDynSys.hpp
        
        
        ${COMMONSOURCE_DIR}/external/pugixml/src/pugixml.hpp
        ${COMMONSOURCE_DIR}/external/rtnorm/rtnorm.hpp
    )


    SET(${SRC}
        ${COMMONSOURCE_DIR}/src/GRSF/Singeltons/FileManager.cpp

        ${COMMONSOURCE_DIR}/src/GRSF/Common/CommonFunctions.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Common/SimpleLogger.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Common/ApplicationCLOptions.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Common/BitCount.cpp
        
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/General/MyMatrixTypeDefs.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/General/MultiBodySimFile.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Inclusion/InclusionSolverCONoG.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Inclusion/InclusionSolverCO.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Collision/CollisionSolver.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/General/MoreauTimeStepper.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Collision/Collider.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Buffers/StateRecorder.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Buffers/StatePoolVisBackFront.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/General/DynamicsSystem.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/General/DynamicsSystemBase.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/General/InitialConditionBodies.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Collision/CollisionData.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Collision/ContactTag.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Collision/Geometry/BoxGeometry.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Collision/Geometry/Ray.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Inclusion/ConvexSets.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Inclusion/ContactModels.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Inclusion/ContactParameterMap.cpp

        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Buffers/StatePoolVisBackFront.cpp
        
        
        ${COMMONSOURCE_DIR}/external/pugixml/src/pugixml.cpp
    )

    set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}}
        ${COMMONSOURCE_DIR}/include/
        ${COMMONSOURCE_DIR}/external/tinyformat/
        ${COMMONSOURCE_DIR}/external/SRDelegates/include/
        ${COMMONSOURCE_DIR}/external/FastFunc/include/
        ${COMMONSOURCE_DIR}/external/getoptpp/
        ${COMMONSOURCE_DIR}/external/getoptpp/src/
        ${COMMONSOURCE_DIR}/external/pugixml/src
        ${COMMONSOURCE_DIR}/external/

    )
endmacro(INCLUDE_SIMULATION_FRAMEWORK_MPI)

MACRO(INCLUDE_SIMULATION_FRAMEWORK_CONVERTER SRC INC INCLUDE_DIRS COMMONSOURCE_DIR )

    SET(${INC}
        ${COMMONSOURCE_DIR}/include/GRSF/Singeltons/FileManager.hpp

        ${COMMONSOURCE_DIR}/include/GRSF/Common/PlatformDefines.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/CommonFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/BitCount.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/Singleton.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/SimpleLogger.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/TypenameComparision.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/BinaryFile.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/ApplicationCLOptions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/Delegates.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/CPUTimer.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/XMLMacros.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/foreach_macro.hpp
        
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/MyMatrixTypeDefs.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/LayoutConfigTypeDefs.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/MeshData.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/MultiBodySimFile.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/MakeCoordinateSystem.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/VectorToSkewMatrix.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/MatrixHelpers.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/QuaternionHelpers.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/DynamicsSystem.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/DynamicsSystemBase.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/ExternalForces.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/RigidBodyId.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/RigidBody.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/RigidBodySolverData.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/RigidBodyContainer.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/MoreauTimeStepper.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/TimeStepperBase.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/TimeStepperSettings.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/InitialConditionBodies.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/InertiaTensorCalculations.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/AddGyroTermVisitor.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/ContactFrame.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/CollisionData.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/CollisionSolver.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/CollisionFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Collider.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/ContactTag.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/ContactPercussion.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/AABB.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/Ray.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/SphereGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/PlaneGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/BoxGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/MeshGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/HalfspaceGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactParameterMap.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactParameter.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactModels.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactGraph.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactGraphNodeData.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactGraphVisitors.hpp not yet made!
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/GeneralGraph.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/PercussionPool.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/InclusionSolverNT.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/InclusionSolverCO.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/InclusionSolverCONoG.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ProxFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ConvexSets.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/InclusionSolverSettings.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/FrontBackBuffer.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/StateRecorder.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/StateRecorderResampler.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/DynamicsState.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/RigidBodyState.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/RecorderSettings.hpp

        

        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/StatePool.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/StatePoolVisBackFront.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/StateRingPoolVisBackFront.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/SharedBufferPlayback.hpp
        
        ${COMMONSOURCE_DIR}/include/GRSF/Systems/SceneParser.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Systems/SharedBufferDynSys.hpp
        
        ${COMMONSOURCE_DIR}/include/GRSF/Logic/LogicCommon.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Logic/LogicNode.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Logic/LogicSocket.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Logic/LookUpTable.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Logic/ExecutionTreeInOut.hpp
        
        ${COMMONSOURCE_DIR}/external/pugixml/src/pugixml.hpp
        ${COMMONSOURCE_DIR}/external/rtnorm/rtnorm.hpp
        ${COMMONSOURCE_DIR}/external/exprtk/exprtk.hpp
    )


    SET(${SRC}
        #${COMMONSOURCE_DIR}/src/GRSF/Singeltons/FileManager.cpp

        ${COMMONSOURCE_DIR}/src/GRSF/Common/CommonFunctions.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Common/SimpleLogger.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Common/ApplicationCLOptions.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Common/BitCount.cpp
        
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/General/MyMatrixTypeDefs.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/General/MultiBodySimFile.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Inclusion/InclusionSolverCONoG.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Inclusion/InclusionSolverCO.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Collision/CollisionSolver.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/General/MoreauTimeStepper.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Collision/Collider.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Buffers/StateRecorder.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Buffers/StatePoolVisBackFront.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/General/DynamicsSystem.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/General/InitialConditionBodies.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Collision/CollisionData.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Collision/ContactTag.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Collision/Geometry/BoxGeometry.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Collision/Geometry/Ray.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Inclusion/ConvexSets.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Inclusion/ContactModels.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Inclusion/ContactParameterMap.cpp
        
        ${COMMONSOURCE_DIR}/src/GRSF/Logic/LogicNode.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Logic/LogicSocket.cpp
        
        ${COMMONSOURCE_DIR}/external/pugixml/src/pugixml.cpp
        
        
    )

    set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}}
        ${COMMONSOURCE_DIR}/include/
        ${COMMONSOURCE_DIR}/external/tinyformat/
        ${COMMONSOURCE_DIR}/external/SRDelegates/include/
        ${COMMONSOURCE_DIR}/external/FastFunc/include/
        ${COMMONSOURCE_DIR}/external/getoptpp/
        ${COMMONSOURCE_DIR}/external/getoptpp/src/
        ${COMMONSOURCE_DIR}/external/pugixml/src
        ${COMMONSOURCE_DIR}/external/exprtk/
        ${COMMONSOURCE_DIR}/external/ 
    )
endmacro(INCLUDE_SIMULATION_FRAMEWORK_CONVERTER)


# TEST FRAMEWORK
MACRO(INCLUDE_SIMULATION_FRAMEWORK_NOGUI_TEST SRC INC INCLUDE_DIRS COMMONSOURCE_DIR )

    SET(${INC}
        ${COMMONSOURCE_DIR}/include/GRSF/Singeltons/FileManager.hpp

        ${COMMONSOURCE_DIR}/include/GRSF/Common/PlatformDefines.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/CommonFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/BitCount.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/Singleton.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/SimpleLogger.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/TypenameComparision.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/BinaryFile.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/ApplicationCLOptions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/Delegates.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/CPUTimer.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/XMLMacros.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Common/foreach_macro.hpp
        
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/MyMatrixTypeDefs.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/LayoutConfigTypeDefs.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/MeshData.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/MultiBodySimFile.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/MakeCoordinateSystem.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/VectorToSkewMatrix.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/MatrixHelpers.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/QuaternionHelpers.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/DynamicsSystem.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/DynamicsSystemBase.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/ExternalForces.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/RigidBodyId.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/RigidBody.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/RigidBodySolverData.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/RigidBodyContainer.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/MoreauTimeStepper.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/TimeStepperBase.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/TimeStepperSettings.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/InitialConditionBodies.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/InertiaTensorCalculations.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/General/AddGyroTermVisitor.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/ContactFrame.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/CollisionData.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/CollisionSolver.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/CollisionFunctions.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Collider.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/ContactTag.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/ContactPercussion.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/AABB.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/Ray.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/SphereGeometry.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/PlaneGeometry.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/BoxGeometry.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/MeshGeometry.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Collision/Geometry/HalfspaceGeometry.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactParameterMap.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactParameter.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactModels.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactGraph.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactGraphVisitors.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactGraphNodeData.hpp
        ##${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ContactGraphVisitors.hpp not yet made!
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/GeneralGraph.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/PercussionPool.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/InclusionSolverNT.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/InclusionSolverCO.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/InclusionSolverCONoG.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ProxFunctions.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/ConvexSets.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Inclusion/InclusionSolverSettings.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/FrontBackBuffer.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/StateRecorder.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/StateRecorderResampler.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/DynamicsState.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/RigidBodyState.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/RecorderSettings.hpp

        #${COMMONSOURCE_DIR}/include/GRSF/Systems/SceneParser.hpp

        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/StatePool.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/StatePoolVisBackFront.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/StateRingPoolVisBackFront.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/Dynamics/Buffers/SharedBufferPlayback.hpp

        #${COMMONSOURCE_DIR}/include/GRSF/Systems/SharedBufferDynSys.hpp
        
        
        ${COMMONSOURCE_DIR}/external/pugixml/src/pugixml.hpp
        ${COMMONSOURCE_DIR}/external/rtnorm/rtnorm.hpp
    )


    SET(${SRC}
        ${COMMONSOURCE_DIR}/src/GRSF/Singeltons/FileManager.cpp

        ${COMMONSOURCE_DIR}/src/GRSF/Common/CommonFunctions.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Common/SimpleLogger.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Common/ApplicationCLOptions.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/Common/BitCount.cpp
        
        ${COMMONSOURCE_DIR}/src/GRSF/Dynamics/General/MyMatrixTypeDefs.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/General/MultiBodySimFile.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Inclusion/InclusionSolverCONoG.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Inclusion/InclusionSolverCO.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Collision/CollisionSolver.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/General/MoreauTimeStepper.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Collision/Collider.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Buffers/StateRecorder.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Buffers/StatePoolVisBackFront.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/General/DynamicsSystem.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/General/InitialConditionBodies.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Collision/CollisionData.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Collision/ContactTag.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Collision/Geometry/BoxGeometry.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Collision/Geometry/Ray.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Inclusion/ConvexSets.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Inclusion/ContactModels.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Inclusion/ContactParameterMap.cpp

        #${COMMONSOURCE_DIR}/src/GRSF/Dynamics/Buffers/StatePoolVisBackFront.cpp
        
        
        ${COMMONSOURCE_DIR}/external/pugixml/src/pugixml.cpp
        

    )

    set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}}
        ${COMMONSOURCE_DIR}/include/
        ${COMMONSOURCE_DIR}/external/tinyformat/
        ${COMMONSOURCE_DIR}/external/SRDelegates/include/
        ${COMMONSOURCE_DIR}/external/FastFunc/include/
        ${COMMONSOURCE_DIR}/external/getoptpp/
        ${COMMONSOURCE_DIR}/external/getoptpp/src/
        ${COMMONSOURCE_DIR}/external/pugixml/src
        ${COMMONSOURCE_DIR}/external/

    )
endmacro(INCLUDE_SIMULATION_FRAMEWORK_NOGUI_TEST)

