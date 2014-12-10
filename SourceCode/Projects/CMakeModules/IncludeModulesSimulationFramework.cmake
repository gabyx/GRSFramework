MACRO(INCLUDE_SIMULATION_FRAMEWORK_GUI SRC INC INCLUDE_DIRS COMMONSOURCE_DIR )

    SET(${INC}
        ${COMMONSOURCE_DIR}/include/GMSF/Singeltons/FileManager.hpp

        ${COMMONSOURCE_DIR}/include/GMSF/Common/PlatformDefines.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/CommonFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/BitCount.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/Singleton.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/SimpleLogger.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/TypenameComparision.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/BinaryFile.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/ApplicationCLOptions.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/Delegates.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/CPUTimer.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/XMLMacros.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/foreach_macro.hpp
        
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/MyMatrixDefs.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/LayoutConfigDefs.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/MeshData.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/MultiBodySimFile.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/MakeCoordinateSystem.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/VectorToSkewMatrix.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/MatrixHelpers.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/QuaternionHelpers.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/DynamicsSystem.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/DynamicsSystemBase.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/ExternalForces.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/RigidBodyId.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/RigidBody.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/RigidBodySolverData.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/RigidBodyContainer.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/MoreauTimeStepper.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/TimeStepperSettings.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/InitialConditionBodies.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/InertiaTensorCalculations.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/AddGyroTermVisitor.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/ContactFrame.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/CollisionData.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/CollisionSolver.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/CollisionFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Collider.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/ContactTag.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/ContactPercussion.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/AABB.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/Ray.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/SphereGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/PlaneGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/BoxGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/MeshGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/HalfspaceGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactParameterMap.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactParameter.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactModels.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactGraph.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactGraphVisitors.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactGraphNodeData.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactGraphVisitors.hpp not yet made!
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/GeneralGraph.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/PercussionPool.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/InclusionSolverNT.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/InclusionSolverCO.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/InclusionSolverCONoG.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ProxFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ConvexSets.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/InclusionSolverSettings.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/FrontBackBuffer.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/StateRecorder.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/StateRecorderResampler.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/DynamicsState.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/RigidBodyState.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/RecorderSettings.hpp

        ${COMMONSOURCE_DIR}/include/GMSF/Systems/SceneParser.hpp

        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/StatePool.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/StatePoolVisBackFront.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/StateRingPoolVisBackFront.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/SharedBufferPlayback.hpp

        ${COMMONSOURCE_DIR}/include/GMSF/Systems/SharedBufferDynSys.hpp
        
        
        ${COMMONSOURCE_DIR}/external/pugixml/src/pugixml.hpp
        ${COMMONSOURCE_DIR}/external/libgdiam/gdiam.hpp
    )


    SET(${SRC}
        ${COMMONSOURCE_DIR}/src/GMSF/Singeltons/FileManager.cpp

        ${COMMONSOURCE_DIR}/src/GMSF/Common/CommonFunctions.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Common/SimpleLogger.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Common/ApplicationCLOptions.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Common/BitCount.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/General/MyMatrixDefs.cpp

        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/General/MultiBodySimFile.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Inclusion/InclusionSolverCONoG.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Inclusion/InclusionSolverCO.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Collision/CollisionSolver.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/General/MoreauTimeStepper.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Collision/Collider.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Buffers/StateRecorder.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Buffers/StatePoolVisBackFront.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/General/DynamicsSystem.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/General/DynamicsSystemBase.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/General/InitialConditionBodies.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Collision/CollisionData.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Collision/ContactTag.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Collision/Geometry/BoxGeometry.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Collision/Geometry/Ray.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Inclusion/ConvexSets.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Inclusion/ContactModels.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Inclusion/ContactParameterMap.cpp

        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Buffers/StatePoolVisBackFront.cpp
        
        
        ${COMMONSOURCE_DIR}/external/pugixml/src/pugixml.cpp
        ${COMMONSOURCE_DIR}/external/libgdiam/gdiam.cpp
    )

    set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}}
        ${COMMONSOURCE_DIR}/include/
        ${COMMONSOURCE_DIR}/external/tinyformat/
        ${COMMONSOURCE_DIR}/external/SRDelegates/include/
        ${COMMONSOURCE_DIR}/external/FastFunc/include/
        ${COMMONSOURCE_DIR}/external/getoptpp/
        ${COMMONSOURCE_DIR}/external/getoptpp/src/
        ${COMMONSOURCE_DIR}/external/pugixml/src
        ${COMMONSOURCE_DIR}/external/libgdiam/
    )
endmacro(INCLUDE_SIMULATION_FRAMEWORK_GUI)


MACRO(INCLUDE_SIMULATION_FRAMEWORK_NOGUI SRC INC INCLUDE_DIRS COMMONSOURCE_DIR )
    INCLUDE_SIMULATION_FRAMEWORK_GUI( ${SRC} ${INC} ${INCLUDE_DIRS} ${COMMONSOURCE_DIR} )
endmacro(INCLUDE_SIMULATION_FRAMEWORK_NOGUI)

MACRO(INCLUDE_SIMULATION_FRAMEWORK_MPI SRC INC INCLUDE_DIRS COMMONSOURCE_DIR )
    
    SET(${INC}
        ${COMMONSOURCE_DIR}/include/GMSF/Singeltons/FileManager.hpp

        ${COMMONSOURCE_DIR}/include/GMSF/Common/PlatformDefines.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/CommonFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/BitCount.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/Singleton.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/SimpleLogger.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/TypenameComparision.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/BinaryFile.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/ApplicationCLOptions.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/Delegates.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/CPUTimer.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/XMLMacros.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/foreach_macro.hpp
        
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/MyMatrixDefs.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/LayoutConfigDefs.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/MeshData.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/MultiBodySimFile.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/MakeCoordinateSystem.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/VectorToSkewMatrix.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/MatrixHelpers.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/QuaternionHelpers.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/DynamicsSystem.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/DynamicsSystemBase.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/ExternalForces.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/RigidBodyId.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/RigidBody.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/RigidBodySolverData.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/RigidBodyContainer.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/MoreauTimeStepper.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/TimeStepperSettings.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/InitialConditionBodies.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/InertiaTensorCalculations.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/AddGyroTermVisitor.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/ContactFrame.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/CollisionData.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/CollisionSolver.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/CollisionFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Collider.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/ContactTag.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/ContactPercussion.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/AABB.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/Ray.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/SphereGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/PlaneGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/BoxGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/MeshGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/HalfspaceGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactParameterMap.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactParameter.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactModels.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactGraph.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactGraphNodeData.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactGraphVisitors.hpp not yet made!
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/GeneralGraph.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/PercussionPool.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/InclusionSolverNT.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/InclusionSolverCO.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/InclusionSolverCONoG.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ProxFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ConvexSets.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/InclusionSolverSettings.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/FrontBackBuffer.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/StateRecorder.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/StateRecorderResampler.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/DynamicsState.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/RigidBodyState.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/RecorderSettings.hpp

        ${COMMONSOURCE_DIR}/include/GMSF/Systems/SceneParser.hpp

        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/StatePool.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/StatePoolVisBackFront.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/StateRingPoolVisBackFront.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/SharedBufferPlayback.hpp

        ${COMMONSOURCE_DIR}/include/GMSF/Systems/SharedBufferDynSys.hpp
        
        
        ${COMMONSOURCE_DIR}/external/pugixml/src/pugixml.hpp
        ${COMMONSOURCE_DIR}/external/libgdiam/gdiam.hpp
    )


    SET(${SRC}
        ${COMMONSOURCE_DIR}/src/GMSF/Singeltons/FileManager.cpp

        ${COMMONSOURCE_DIR}/src/GMSF/Common/CommonFunctions.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Common/SimpleLogger.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Common/ApplicationCLOptions.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Common/BitCount.cpp
        
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/General/MyMatrixDefs.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/General/MultiBodySimFile.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Inclusion/InclusionSolverCONoG.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Inclusion/InclusionSolverCO.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Collision/CollisionSolver.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/General/MoreauTimeStepper.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Collision/Collider.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Buffers/StateRecorder.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Buffers/StatePoolVisBackFront.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/General/DynamicsSystem.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/General/DynamicsSystemBase.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/General/InitialConditionBodies.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Collision/CollisionData.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Collision/ContactTag.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Collision/Geometry/BoxGeometry.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Collision/Geometry/Ray.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Inclusion/ConvexSets.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Inclusion/ContactModels.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Inclusion/ContactParameterMap.cpp

        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Buffers/StatePoolVisBackFront.cpp
        
        
        ${COMMONSOURCE_DIR}/external/pugixml/src/pugixml.cpp
        ${COMMONSOURCE_DIR}/external/libgdiam/gdiam.cpp
    )

    set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}}
        ${COMMONSOURCE_DIR}/include/
        ${COMMONSOURCE_DIR}/external/tinyformat/
        ${COMMONSOURCE_DIR}/external/SRDelegates/include/
        ${COMMONSOURCE_DIR}/external/FastFunc/include/
        ${COMMONSOURCE_DIR}/external/getoptpp/
        ${COMMONSOURCE_DIR}/external/getoptpp/src/
        ${COMMONSOURCE_DIR}/external/pugixml/src
        ${COMMONSOURCE_DIR}/external/libgdiam/
    )
endmacro(INCLUDE_SIMULATION_FRAMEWORK_MPI)

MACRO(INCLUDE_SIMULATION_FRAMEWORK_CONVERTER SRC INC INCLUDE_DIRS COMMONSOURCE_DIR )

    SET(${INC}
        ${COMMONSOURCE_DIR}/include/GMSF/Singeltons/FileManager.hpp

        ${COMMONSOURCE_DIR}/include/GMSF/Common/PlatformDefines.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/CommonFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/BitCount.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/Singleton.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/SimpleLogger.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/TypenameComparision.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/BinaryFile.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/ApplicationCLOptions.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/Delegates.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/CPUTimer.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/XMLMacros.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/foreach_macro.hpp
        
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/MyMatrixDefs.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/LayoutConfigDefs.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/MeshData.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/MultiBodySimFile.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/MakeCoordinateSystem.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/VectorToSkewMatrix.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/MatrixHelpers.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/QuaternionHelpers.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/DynamicsSystem.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/DynamicsSystemBase.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/ExternalForces.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/RigidBodyId.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/RigidBody.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/RigidBodySolverData.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/RigidBodyContainer.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/MoreauTimeStepper.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/TimeStepperSettings.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/InitialConditionBodies.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/InertiaTensorCalculations.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/AddGyroTermVisitor.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/ContactFrame.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/CollisionData.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/CollisionSolver.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/CollisionFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Collider.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/ContactTag.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/ContactPercussion.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/AABB.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/Ray.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/SphereGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/PlaneGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/BoxGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/MeshGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/HalfspaceGeometry.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactParameterMap.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactParameter.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactModels.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactGraph.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactGraphNodeData.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactGraphVisitors.hpp not yet made!
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/GeneralGraph.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/PercussionPool.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/InclusionSolverNT.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/InclusionSolverCO.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/InclusionSolverCONoG.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ProxFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ConvexSets.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/InclusionSolverSettings.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/FrontBackBuffer.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/StateRecorder.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/StateRecorderResampler.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/DynamicsState.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/RigidBodyState.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/RecorderSettings.hpp

        

        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/StatePool.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/StatePoolVisBackFront.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/StateRingPoolVisBackFront.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/SharedBufferPlayback.hpp
        
        ${COMMONSOURCE_DIR}/include/GMSF/Systems/SceneParser.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Systems/SharedBufferDynSys.hpp
        
        ${COMMONSOURCE_DIR}/include/GMSF/Logic/LogicCommon.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Logic/LogicNode.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Logic/LogicSocket.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Logic/LookUpTable.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Logic/ExecutionTreeInOut.hpp
        
        ${COMMONSOURCE_DIR}/external/pugixml/src/pugixml.hpp
        #${COMMONSOURCE_DIR}/external/libgdiam/gdiam.hpp
    )


    SET(${SRC}
        #${COMMONSOURCE_DIR}/src/GMSF/Singeltons/FileManager.cpp

        ${COMMONSOURCE_DIR}/src/GMSF/Common/CommonFunctions.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Common/SimpleLogger.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Common/ApplicationCLOptions.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Common/BitCount.cpp
        
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/General/MyMatrixDefs.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/General/MultiBodySimFile.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Inclusion/InclusionSolverCONoG.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Inclusion/InclusionSolverCO.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Collision/CollisionSolver.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/General/MoreauTimeStepper.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Collision/Collider.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Buffers/StateRecorder.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Buffers/StatePoolVisBackFront.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/General/DynamicsSystem.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/General/InitialConditionBodies.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Collision/CollisionData.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Collision/ContactTag.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Collision/Geometry/BoxGeometry.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Collision/Geometry/Ray.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Inclusion/ConvexSets.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Inclusion/ContactModels.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Inclusion/ContactParameterMap.cpp
        
        ${COMMONSOURCE_DIR}/src/GMSF/Logic/LogicNode.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Logic/LogicSocket.cpp
        
        ${COMMONSOURCE_DIR}/external/pugixml/src/pugixml.cpp
        
        ${COMMONSOURCE_DIR}/external/exprtk/exprtk.hpp
    )

    set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}}
        ${COMMONSOURCE_DIR}/include/
        ${COMMONSOURCE_DIR}/external/tinyformat/
        ${COMMONSOURCE_DIR}/external/SRDelegates/include/
        ${COMMONSOURCE_DIR}/external/FastFunc/include/
        ${COMMONSOURCE_DIR}/external/getoptpp/
        ${COMMONSOURCE_DIR}/external/getoptpp/src/
        ${COMMONSOURCE_DIR}/external/pugixml/src
        ${COMMONSOURCE_DIR}/external/libgdiam/
        ${COMMONSOURCE_DIR}/external/exprtk/  
    )
endmacro(INCLUDE_SIMULATION_FRAMEWORK_CONVERTER)


# TEST FRAMEWORK
MACRO(INCLUDE_SIMULATION_FRAMEWORK_NOGUI_TEST SRC INC INCLUDE_DIRS COMMONSOURCE_DIR )

    SET(${INC}
        ${COMMONSOURCE_DIR}/include/GMSF/Singeltons/FileManager.hpp

        ${COMMONSOURCE_DIR}/include/GMSF/Common/PlatformDefines.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/CommonFunctions.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/BitCount.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/Singleton.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/SimpleLogger.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/TypenameComparision.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/BinaryFile.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/ApplicationCLOptions.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/Delegates.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/CPUTimer.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/XMLMacros.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Common/foreach_macro.hpp
        
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/MyMatrixDefs.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/LayoutConfigDefs.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/MeshData.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/MultiBodySimFile.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/MakeCoordinateSystem.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/VectorToSkewMatrix.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/MatrixHelpers.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/QuaternionHelpers.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/DynamicsSystem.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/DynamicsSystemBase.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/ExternalForces.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/RigidBodyId.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/RigidBody.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/RigidBodySolverData.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/RigidBodyContainer.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/MoreauTimeStepper.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/TimeStepperSettings.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/InitialConditionBodies.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/InertiaTensorCalculations.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/General/AddGyroTermVisitor.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/ContactFrame.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/CollisionData.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/CollisionSolver.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/CollisionFunctions.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Collider.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/ContactTag.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/ContactPercussion.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/AABB.hpp
        ${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/Ray.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/SphereGeometry.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/PlaneGeometry.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/BoxGeometry.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/MeshGeometry.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Collision/Geometry/HalfspaceGeometry.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactParameterMap.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactParameter.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactModels.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactGraph.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactGraphVisitors.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactGraphNodeData.hpp
        ##${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ContactGraphVisitors.hpp not yet made!
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/GeneralGraph.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/PercussionPool.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/InclusionSolverNT.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/InclusionSolverCO.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/InclusionSolverCONoG.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ProxFunctions.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/ConvexSets.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Inclusion/InclusionSolverSettings.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/FrontBackBuffer.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/StateRecorder.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/StateRecorderResampler.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/DynamicsState.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/RigidBodyState.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/RecorderSettings.hpp

        #${COMMONSOURCE_DIR}/include/GMSF/Systems/SceneParser.hpp

        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/StatePool.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/StatePoolVisBackFront.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/StateRingPoolVisBackFront.hpp
        #${COMMONSOURCE_DIR}/include/GMSF/Dynamics/Buffers/SharedBufferPlayback.hpp

        #${COMMONSOURCE_DIR}/include/GMSF/Systems/SharedBufferDynSys.hpp
        
        
        ${COMMONSOURCE_DIR}/external/pugixml/src/pugixml.hpp
        #${COMMONSOURCE_DIR}/external/libgdiam/gdiam.hpp
        
        ${COMMONSOURCE_DIR}/external/diameter/EstimateDiameter.hpp
        
        ${COMMONSOURCE_DIR}/external/geometrypredicates/GeometryPredicates.hpp
        
    )


    SET(${SRC}
        ${COMMONSOURCE_DIR}/src/GMSF/Singeltons/FileManager.cpp

        ${COMMONSOURCE_DIR}/src/GMSF/Common/CommonFunctions.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Common/SimpleLogger.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Common/ApplicationCLOptions.cpp
        ${COMMONSOURCE_DIR}/src/GMSF/Common/BitCount.cpp
        
        ${COMMONSOURCE_DIR}/src/GMSF/Dynamics/General/MyMatrixDefs.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/General/MultiBodySimFile.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Inclusion/InclusionSolverCONoG.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Inclusion/InclusionSolverCO.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Collision/CollisionSolver.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/General/MoreauTimeStepper.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Collision/Collider.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Buffers/StateRecorder.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Buffers/StatePoolVisBackFront.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/General/DynamicsSystem.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/General/InitialConditionBodies.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Collision/CollisionData.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Collision/ContactTag.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Collision/Geometry/BoxGeometry.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Collision/Geometry/Ray.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Inclusion/ConvexSets.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Inclusion/ContactModels.cpp
        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Inclusion/ContactParameterMap.cpp

        #${COMMONSOURCE_DIR}/src/GMSF/Dynamics/Buffers/StatePoolVisBackFront.cpp
        
        
        ${COMMONSOURCE_DIR}/external/pugixml/src/pugixml.cpp
        ${COMMONSOURCE_DIR}/external/geometrypredicates/GeometryPredicates.cpp
        ${COMMONSOURCE_DIR}/external/libgdiam/gdiam.cpp
        
        ${COMMONSOURCE_DIR}/external/diameter/EstimateDiameter.cpp
        ${COMMONSOURCE_DIR}/external/diameter/diameterUtils/alloc.cpp
        ${COMMONSOURCE_DIR}/external/diameter/diameterUtils/util.cpp
        ${COMMONSOURCE_DIR}/external/diameter/diameterUtils/rand.cpp
    )

    set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}}
        ${COMMONSOURCE_DIR}/include/
        ${COMMONSOURCE_DIR}/external/tinyformat/
        ${COMMONSOURCE_DIR}/external/SRDelegates/include/
        ${COMMONSOURCE_DIR}/external/FastFunc/include/
        ${COMMONSOURCE_DIR}/external/getoptpp/
        ${COMMONSOURCE_DIR}/external/getoptpp/src/
        ${COMMONSOURCE_DIR}/external/pugixml/src
        ${COMMONSOURCE_DIR}/external/geometrypredicates/
        ${COMMONSOURCE_DIR}/external/libgdiam/
        ${COMMONSOURCE_DIR}/external/diameter/

    )
endmacro(INCLUDE_SIMULATION_FRAMEWORK_NOGUI_TEST)

