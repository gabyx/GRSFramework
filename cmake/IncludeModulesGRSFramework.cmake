MACRO(INCLUDE_SIMULATION_FRAMEWORK_GUI SRC INC INCLUDE_DIRS COMMONSOURCE_DIR )

    SET(${INC}
        ${COMMONSOURCE_DIR}/include/GRSF/singeltons/FileManager.hpp
        
        ${COMMONSOURCE_DIR}/include/GRSF/common/DemangleTypes.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/PlatformDefines.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/CommonFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/BitCount.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/Singleton.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/SimpleLogger.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/TypenameComparision.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/BinaryFile.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/ApplicationCLOptions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/Delegates.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/CPUTimer.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/XMLMacros.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/foreach_macro.hpp
        
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/MyMatrixTypeDefs.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/LayoutConfigTypeDefs.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/MeshData.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/MultiBodySimFile.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/MakeCoordinateSystem.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/VectorToSkewMatrix.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/MatrixHelpers.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/QuaternionHelpers.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/CartesianGrid.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/KdTree.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/DynamicsSystem.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/DynamicsSystemBase.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/ExternalForces.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/RigidBodyId.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/RigidBody.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/RigidBodySolverData.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/RigidBodyContainer.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/MoreauTimeStepper.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/TimeStepperBase.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/TimeStepperSettings.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/InitialConditionBodies.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/InertiaTensorCalculations.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/AddGyroTermVisitor.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/ContactFrame.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/CollisionData.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/CollisionSolver.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/CollisionFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/Collider.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/ContactTag.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/AABB.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/Ray.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/SphereGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/PlaneGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/BoxGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/MeshGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/HalfspaceGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactParameterMap.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactParameter.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactModels.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactGraph.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactGraphVisitors.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactGraphNodeData.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactGraphVisitors.hpp not yet made!
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/GeneralGraph.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/PercussionPool.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactPercussion.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/InclusionSolverNT.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/InclusionSolverCO.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/InclusionSolverCONoG.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ProxFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ConvexSets.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/InclusionSolverSettings.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/FrontBackBuffer.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/StateRecorder.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/StateRecorderResampler.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/DynamicsState.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/RigidBodyState.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/RecorderSettings.hpp

        ${COMMONSOURCE_DIR}/include/GRSF/systems/SceneParser.hpp

        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/StatePool.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/StatePoolVisBackFront.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/StateRingPoolVisBackFront.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/SharedBufferPlayback.hpp

        ${COMMONSOURCE_DIR}/include/GRSF/systems/SharedBufferDynSys.hpp
        
        ${COMMONSOURCE_DIR}/external/rtnorm/rtnorm.hpp
    )


    SET(${SRC}
        ${COMMONSOURCE_DIR}/src/GRSF/singeltons/FileManager.cpp
        
        ${COMMONSOURCE_DIR}/src/GRSF/common/DemangleTypes.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/common/CommonFunctions.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/common/SimpleLogger.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/common/ApplicationCLOptions.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/common/BitCount.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/general/MyMatrixTypeDefs.cpp

        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/general/MultiBodySimFile.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/inclusion/InclusionSolverCONoG.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/inclusion/InclusionSolverCO.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/CollisionSolver.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/general/MoreauTimeStepper.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/Collider.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/buffers/StateRecorder.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/buffers/StatePoolVisBackFront.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/general/DynamicsSystem.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/general/DynamicsSystemBase.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/general/InitialConditionBodies.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/CollisionData.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/ContactTag.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/geometry/BoxGeometry.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/geometry/Ray.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/inclusion/ConvexSets.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/inclusion/ContactModels.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/inclusion/ContactParameterMap.cpp

        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/buffers/StatePoolVisBackFront.cpp
        
    )

    set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}}
        ${COMMONSOURCE_DIR}/include/
        ${COMMONSOURCE_DIR}/external/tinyformat/
        ${COMMONSOURCE_DIR}/external/SRDelegates/include/
        ${COMMONSOURCE_DIR}/external/FastFunc/include/
        ${COMMONSOURCE_DIR}/external/getoptpp/
        ${COMMONSOURCE_DIR}/external/getoptpp/src/
        ${COMMONSOURCE_DIR}/external/
    )
endmacro(INCLUDE_SIMULATION_FRAMEWORK_GUI)


MACRO(INCLUDE_SIMULATION_FRAMEWORK_NOGUI SRC INC INCLUDE_DIRS COMMONSOURCE_DIR )
    INCLUDE_SIMULATION_FRAMEWORK_GUI( ${SRC} ${INC} ${INCLUDE_DIRS} ${COMMONSOURCE_DIR} )
endmacro(INCLUDE_SIMULATION_FRAMEWORK_NOGUI)

MACRO(INCLUDE_SIMULATION_FRAMEWORK_MPI SRC INC INCLUDE_DIRS COMMONSOURCE_DIR )
    
    SET(${INC}
        ${COMMONSOURCE_DIR}/include/GRSF/singeltons/FileManager.hpp
        
        ${COMMONSOURCE_DIR}/include/GRSF/common/DemangleTypes.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/PlatformDefines.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/CommonFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/BitCount.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/Singleton.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/SimpleLogger.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/TypenameComparision.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/BinaryFile.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/ApplicationCLOptions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/Delegates.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/CPUTimer.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/XMLMacros.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/foreach_macro.hpp
        
        ${COMMONSOURCE_DIR}/include/GRSF/common/SerializationHelpersTuple.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/SerializationHelpersEigen.hpp
        
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/MyMatrixTypeDefs.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/LayoutConfigTypeDefs.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/MeshData.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/MultiBodySimFile.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/MakeCoordinateSystem.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/VectorToSkewMatrix.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/MatrixHelpers.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/QuaternionHelpers.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/CartesianGrid.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/KdTree.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/DynamicsSystem.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/DynamicsSystemBase.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/ExternalForces.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/RigidBodyId.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/RigidBody.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/RigidBodySolverData.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/RigidBodyContainer.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/MoreauTimeStepper.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/TimeStepperBase.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/TimeStepperSettings.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/InitialConditionBodies.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/InertiaTensorCalculations.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/SerializationHelpersKdTree.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/AddGyroTermVisitor.hpp
        
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/ContactFrame.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/CollisionData.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/CollisionSolver.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/CollisionFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/Collider.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/ContactTag.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/SerializationHelpersGeometries.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/AABB.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/Ray.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/SphereGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/PlaneGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/BoxGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/MeshGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/HalfspaceGeometry.hpp
        
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactParameterMap.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactParameter.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactModels.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactGraph.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactGraphNodeData.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactGraphVisitors.hpp not yet made!
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/GeneralGraph.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactPercussion.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/PercussionPool.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/InclusionSolverNT.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/InclusionSolverCO.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/InclusionSolverCONoG.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ProxFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ConvexSets.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/InclusionSolverSettings.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/FrontBackBuffer.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/StateRecorder.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/StateRecorderResampler.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/DynamicsState.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/RigidBodyState.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/RecorderSettings.hpp

        ${COMMONSOURCE_DIR}/include/GRSF/systems/SceneParser.hpp

        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/StatePool.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/StatePoolVisBackFront.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/StateRingPoolVisBackFront.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/SharedBufferPlayback.hpp

        ${COMMONSOURCE_DIR}/include/GRSF/systems/SharedBufferDynSys.hpp
        
        ${COMMONSOURCE_DIR}/external/rtnorm/rtnorm.hpp
    )


    SET(${SRC}
        ${COMMONSOURCE_DIR}/src/GRSF/singeltons/FileManager.cpp
        
        ${COMMONSOURCE_DIR}/src/GRSF/common/DemangleTypes.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/common/CommonFunctions.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/common/SimpleLogger.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/common/ApplicationCLOptions.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/common/BitCount.cpp
        
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/general/MyMatrixTypeDefs.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/general/MultiBodySimFile.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/inclusion/InclusionSolverCONoG.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/inclusion/InclusionSolverCO.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/CollisionSolver.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/general/MoreauTimeStepper.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/Collider.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/buffers/StateRecorder.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/buffers/StatePoolVisBackFront.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/general/DynamicsSystem.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/general/DynamicsSystemBase.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/general/InitialConditionBodies.cpp
        
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/SerializationHelpersGeometries.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/CollisionData.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/ContactTag.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/geometry/BoxGeometry.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/geometry/Ray.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/inclusion/ConvexSets.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/inclusion/ContactModels.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/inclusion/ContactParameterMap.cpp

        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/buffers/StatePoolVisBackFront.cpp
    
    )

    set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}}
        ${COMMONSOURCE_DIR}/include/
        ${COMMONSOURCE_DIR}/external/tinyformat/
        ${COMMONSOURCE_DIR}/external/SRDelegates/include/
        ${COMMONSOURCE_DIR}/external/FastFunc/include/
        ${COMMONSOURCE_DIR}/external/getoptpp/
        ${COMMONSOURCE_DIR}/external/getoptpp/src/
        ${COMMONSOURCE_DIR}/external/

    )
endmacro(INCLUDE_SIMULATION_FRAMEWORK_MPI)

MACRO(INCLUDE_SIMULATION_FRAMEWORK_CONVERTER SRC INC INCLUDE_DIRS COMMONSOURCE_DIR )

    SET(${INC}
        ${COMMONSOURCE_DIR}/include/GRSF/singeltons/FileManager.hpp
    
        ${COMMONSOURCE_DIR}/include/GRSF/common/DemangleTypes.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/PlatformDefines.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/CommonFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/BitCount.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/Singleton.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/SimpleLogger.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/TypenameComparision.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/BinaryFile.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/ApplicationCLOptions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/Delegates.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/CPUTimer.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/XMLMacros.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/foreach_macro.hpp
        
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/MyMatrixTypeDefs.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/LayoutConfigTypeDefs.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/MeshData.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/MultiBodySimFile.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/MakeCoordinateSystem.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/VectorToSkewMatrix.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/MatrixHelpers.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/QuaternionHelpers.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/CartesianGrid.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/KdTree.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/DynamicsSystem.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/DynamicsSystemBase.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/ExternalForces.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/RigidBodyId.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/RigidBody.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/RigidBodySolverData.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/RigidBodyContainer.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/MoreauTimeStepper.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/TimeStepperBase.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/TimeStepperSettings.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/InitialConditionBodies.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/InertiaTensorCalculations.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/AddGyroTermVisitor.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/ContactFrame.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/CollisionData.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/CollisionSolver.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/CollisionFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/Collider.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/ContactTag.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/AABB.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/Ray.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/SphereGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/PlaneGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/BoxGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/MeshGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/HalfspaceGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactParameterMap.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactParameter.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactModels.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactGraph.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactGraphNodeData.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactGraphVisitors.hpp not yet made!
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/GeneralGraph.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactPercussion.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/PercussionPool.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/InclusionSolverNT.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/InclusionSolverCO.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/InclusionSolverCONoG.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ProxFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ConvexSets.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/InclusionSolverSettings.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/FrontBackBuffer.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/StateRecorder.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/StateRecorderResampler.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/DynamicsState.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/RigidBodyState.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/RecorderSettings.hpp

        

        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/StatePool.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/StatePoolVisBackFront.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/StateRingPoolVisBackFront.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/SharedBufferPlayback.hpp
        
        ${COMMONSOURCE_DIR}/include/GRSF/systems/SceneParser.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/systems/SharedBufferDynSys.hpp
        
        ${COMMONSOURCE_DIR}/include/GRSF/logic/LogicCommon.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/logic/LogicNode.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/logic/LogicSocket.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/logic/LookUpTable.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/logic/ExecutionTreeInOut.hpp
        
        ${COMMONSOURCE_DIR}/external/rtnorm/rtnorm.hpp
        ${COMMONSOURCE_DIR}/external/exprtk/exprtk.hpp
    )


    SET(${SRC}
        #${COMMONSOURCE_DIR}/src/GRSF/singeltons/FileManager.cpp
        
        ${COMMONSOURCE_DIR}/src/GRSF/common/DemangleTypes.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/common/CommonFunctions.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/common/SimpleLogger.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/common/ApplicationCLOptions.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/common/BitCount.cpp
        
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/general/MyMatrixTypeDefs.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/general/MultiBodySimFile.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/inclusion/InclusionSolverCONoG.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/inclusion/InclusionSolverCO.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/CollisionSolver.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/general/MoreauTimeStepper.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/Collider.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/buffers/StateRecorder.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/buffers/StatePoolVisBackFront.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/general/DynamicsSystem.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/general/InitialConditionBodies.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/CollisionData.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/ContactTag.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/geometry/BoxGeometry.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/geometry/Ray.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/inclusion/ConvexSets.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/inclusion/ContactModels.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/inclusion/ContactParameterMap.cpp
        
        ${COMMONSOURCE_DIR}/src/GRSF/logic/LogicNode.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/logic/LogicSocket.cpp
        
    )

    set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}}
        ${COMMONSOURCE_DIR}/include/
        ${COMMONSOURCE_DIR}/external/tinyformat/
        ${COMMONSOURCE_DIR}/external/SRDelegates/include/
        ${COMMONSOURCE_DIR}/external/FastFunc/include/
        ${COMMONSOURCE_DIR}/external/getoptpp/
        ${COMMONSOURCE_DIR}/external/getoptpp/src/
        ${COMMONSOURCE_DIR}/external/exprtk/
        ${COMMONSOURCE_DIR}/external/ 
    )
endmacro(INCLUDE_SIMULATION_FRAMEWORK_CONVERTER)


# TEST FRAMEWORK
MACRO(INCLUDE_SIMULATION_FRAMEWORK_NOGUI_TEST SRC INC INCLUDE_DIRS COMMONSOURCE_DIR )

    SET(${INC}
        ${COMMONSOURCE_DIR}/include/GRSF/singeltons/FileManager.hpp
        
        ${COMMONSOURCE_DIR}/include/GRSF/common/DemangleTypes.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/PlatformDefines.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/CommonFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/BitCount.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/Singleton.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/SimpleLogger.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/TypenameComparision.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/BinaryFile.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/ApplicationCLOptions.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/Delegates.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/CPUTimer.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/XMLMacros.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/common/foreach_macro.hpp
        
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/MyMatrixTypeDefs.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/LayoutConfigTypeDefs.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/MeshData.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/MultiBodySimFile.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/MakeCoordinateSystem.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/VectorToSkewMatrix.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/MatrixHelpers.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/QuaternionHelpers.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/CartesianGrid.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/KdTree.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/DynamicsSystem.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/DynamicsSystemBase.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/ExternalForces.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/RigidBodyId.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/RigidBody.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/RigidBodySolverData.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/RigidBodyContainer.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/MoreauTimeStepper.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/TimeStepperBase.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/TimeStepperSettings.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/InitialConditionBodies.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/InertiaTensorCalculations.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/general/AddGyroTermVisitor.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/ContactFrame.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/CollisionData.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/CollisionSolver.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/CollisionFunctions.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/Collider.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/ContactTag.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/AABB.hpp
        ${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/Ray.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/SphereGeometry.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/PlaneGeometry.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/BoxGeometry.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/MeshGeometry.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/collision/geometry/HalfspaceGeometry.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactParameterMap.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactParameter.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactModels.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactGraph.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactGraphVisitors.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactGraphNodeData.hpp
        ##${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactGraphVisitors.hpp not yet made!
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/GeneralGraph.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/PercussionPool.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ContactPercussion.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/InclusionSolverNT.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/InclusionSolverCO.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/InclusionSolverCONoG.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ProxFunctions.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/ConvexSets.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/inclusion/InclusionSolverSettings.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/FrontBackBuffer.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/StateRecorder.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/StateRecorderResampler.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/DynamicsState.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/RigidBodyState.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/RecorderSettings.hpp

        #${COMMONSOURCE_DIR}/include/GRSF/systems/SceneParser.hpp

        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/StatePool.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/StatePoolVisBackFront.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/StateRingPoolVisBackFront.hpp
        #${COMMONSOURCE_DIR}/include/GRSF/dynamics/buffers/SharedBufferPlayback.hpp

        #${COMMONSOURCE_DIR}/include/GRSF/systems/SharedBufferDynSys.hpp


        ${COMMONSOURCE_DIR}/external/rtnorm/rtnorm.hpp
    )


    SET(${SRC}
        ${COMMONSOURCE_DIR}/src/GRSF/singeltons/FileManager.cpp
        
        ${COMMONSOURCE_DIR}/src/GRSF/common/DemangleTypes.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/common/CommonFunctions.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/common/SimpleLogger.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/common/ApplicationCLOptions.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/common/BitCount.cpp
        
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/general/MyMatrixTypeDefs.cpp
        ${COMMONSOURCE_DIR}/src/GRSF/dynamics/general/MultiBodySimFile.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/inclusion/InclusionSolverCONoG.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/inclusion/InclusionSolverCO.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/CollisionSolver.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/general/MoreauTimeStepper.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/Collider.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/buffers/StateRecorder.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/buffers/StatePoolVisBackFront.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/general/DynamicsSystem.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/general/InitialConditionBodies.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/CollisionData.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/ContactTag.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/geometry/BoxGeometry.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/collision/geometry/Ray.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/inclusion/ConvexSets.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/inclusion/ContactModels.cpp
        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/inclusion/ContactParameterMap.cpp

        #${COMMONSOURCE_DIR}/src/GRSF/dynamics/buffers/StatePoolVisBackFront.cpp

        

    )

    set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}}
        ${COMMONSOURCE_DIR}/include/
        ${COMMONSOURCE_DIR}/external/tinyformat/
        ${COMMONSOURCE_DIR}/external/SRDelegates/include/
        ${COMMONSOURCE_DIR}/external/FastFunc/include/
        ${COMMONSOURCE_DIR}/external/getoptpp/
        ${COMMONSOURCE_DIR}/external/getoptpp/src/
        ${COMMONSOURCE_DIR}/external/

    )
endmacro(INCLUDE_SIMULATION_FRAMEWORK_NOGUI_TEST)

