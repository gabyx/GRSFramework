MACRO(INCLUDE_SIMULATION_FRAMEWORK_GUI SRC INC INCLUDE_DIRS COMMONSOURCE_DIR )

    SET(${INC}
        ${COMMONSOURCE_DIR}/include/Singeltons/FileManager.hpp

        ${COMMONSOURCE_DIR}/include/Common/PlatformDefines.hpp
        ${COMMONSOURCE_DIR}/include/Common/CommonFunctions.hpp
        ${COMMONSOURCE_DIR}/include/Common/BitCount.hpp
        ${COMMONSOURCE_DIR}/include/Common/Singleton.hpp
        ${COMMONSOURCE_DIR}/include/Common/SimpleLogger.hpp
        ${COMMONSOURCE_DIR}/include/Common/TypenameComparision.hpp
        ${COMMONSOURCE_DIR}/include/Common/BinaryFile.hpp
        ${COMMONSOURCE_DIR}/include/Common/ApplicationCLOptions.hpp
        ${COMMONSOURCE_DIR}/include/Common/Delegates.hpp
        ${COMMONSOURCE_DIR}/include/Common/CPUTimer.hpp
        ${COMMONSOURCE_DIR}/include/Common/foreach_macro.hpp

        ${COMMONSOURCE_DIR}/include/Dynamics/General/LayoutConfigDefs.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/MeshData.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/MultiBodySimFile.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/MakeCoordinateSystem.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/VectorToSkewMatrix.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/MatrixHelpers.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/QuaternionHelpers.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/DynamicsSystem.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/DynamicsSystemBase.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/ExternalForces.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/RigidBodyId.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/RigidBody.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/RigidBodySolverData.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/RigidBodyContainer.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/MoreauTimeStepper.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/TimeStepperSettings.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/InitialConditionBodies.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/InertiaTensorCalculations.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/AddGyroTermVisitor.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/ContactFrame.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/CollisionData.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/CollisionSolver.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/CollisionFunctions.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Collider.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/ContactTag.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/ContactPercussion.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/AABB.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/Ray.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/SphereGeometry.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/PlaneGeometry.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/BoxGeometry.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/MeshGeometry.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/HalfspaceGeometry.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/ContactParameterMap.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/ContactParameter.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/ContactModels.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/ContactGraph.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/ContactGraphVisitors.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/ContactGraphNodeData.hpp
        #${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/ContactGraphVisitors.hpp not yet made!
        ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/GeneralGraph.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/PercussionPool.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/InclusionSolverNT.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/InclusionSolverCO.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/InclusionSolverCONoG.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/ProxFunctions.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/ConvexSets.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/InclusionSolverSettings.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/FrontBackBuffer.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/StateRecorder.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/StateRecorderResampler.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/DynamicsState.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/RigidBodyState.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/RecorderSettings.hpp

        ${COMMONSOURCE_DIR}/include/Systems/SceneParser.hpp

        ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/StatePool.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/StatePoolVisBackFront.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/StateRingPoolVisBackFront.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/SharedBufferPlayback.hpp

        ${COMMONSOURCE_DIR}/include/Systems/SharedBufferDynSys.hpp
        
        
        ${COMMONSOURCE_DIR}/external/pugixml/src/pugixml.hpp
    )


    SET(${SRC}
        ${COMMONSOURCE_DIR}/src/Singeltons/FileManager.cpp

        ${COMMONSOURCE_DIR}/src/Common/CommonFunctions.cpp
        ${COMMONSOURCE_DIR}/src/Common/SimpleLogger.cpp
        ${COMMONSOURCE_DIR}/src/Common/ApplicationCLOptions.cpp
        ${COMMONSOURCE_DIR}/src/Common/BitCount.cpp

        ${COMMONSOURCE_DIR}/src/Dynamics/General/MultiBodySimFile.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/Inclusion/InclusionSolverCONoG.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/Inclusion/InclusionSolverCO.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/Collision/CollisionSolver.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/General/MoreauTimeStepper.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/Collision/Collider.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/Buffers/StateRecorder.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/Buffers/StatePoolVisBackFront.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/General/DynamicsSystem.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/General/InitialConditionBodies.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/Collision/CollisionData.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/Collision/ContactTag.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/Collision/Geometry/BoxGeometry.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/Collision/Geometry/Ray.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/Inclusion/ConvexSets.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/Inclusion/ContactModels.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/Inclusion/ContactParameterMap.cpp

        ${COMMONSOURCE_DIR}/src/Dynamics/Buffers/StatePoolVisBackFront.cpp
        
        
        ${COMMONSOURCE_DIR}/external/pugixml/src/pugixml.cpp
    )

    set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}}
        ${COMMONSOURCE_DIR}/include/Common
        ${COMMONSOURCE_DIR}/external/SRDelegates/include/
        ${COMMONSOURCE_DIR}/external/FastFunc/include/
        ${COMMONSOURCE_DIR}/external/getoptpp/
        ${COMMONSOURCE_DIR}/external/getoptpp/src/
        ${COMMONSOURCE_DIR}/external/pugixml/src
        ${COMMONSOURCE_DIR}/include/Dynamics/Buffers
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry
        ${COMMONSOURCE_DIR}/include/Dynamics/General
        ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion
        ${COMMONSOURCE_DIR}/include/Systems
        ${COMMONSOURCE_DIR}/include/Singeltons
        ${COMMONSOURCE_DIR}/include/States
    )
endmacro(INCLUDE_SIMULATION_FRAMEWORK_GUI)


MACRO(INCLUDE_SIMULATION_FRAMEWORK_NOGUI SRC INC INCLUDE_DIRS COMMONSOURCE_DIR )
    INCLUDE_SIMULATION_FRAMEWORK_GUI( ${SRC} ${INC} ${INCLUDE_DIRS} ${COMMONSOURCE_DIR} )
endmacro(INCLUDE_SIMULATION_FRAMEWORK_NOGUI)

MACRO(INCLUDE_SIMULATION_FRAMEWORK_MPI SRC INC INCLUDE_DIRS COMMONSOURCE_DIR )
    
    SET(${INC}
        ${COMMONSOURCE_DIR}/include/Singeltons/FileManager.hpp

        ${COMMONSOURCE_DIR}/include/Common/PlatformDefines.hpp
        ${COMMONSOURCE_DIR}/include/Common/CommonFunctions.hpp
        ${COMMONSOURCE_DIR}/include/Common/BitCount.hpp
        ${COMMONSOURCE_DIR}/include/Common/Singleton.hpp
        ${COMMONSOURCE_DIR}/include/Common/SimpleLogger.hpp
        ${COMMONSOURCE_DIR}/include/Common/TypenameComparision.hpp
        ${COMMONSOURCE_DIR}/include/Common/BinaryFile.hpp
        ${COMMONSOURCE_DIR}/include/Common/ApplicationCLOptions.hpp
        ${COMMONSOURCE_DIR}/include/Common/Delegates.hpp
        ${COMMONSOURCE_DIR}/include/Common/CPUTimer.hpp
        ${COMMONSOURCE_DIR}/include/Common/foreach_macro.hpp

        ${COMMONSOURCE_DIR}/include/Dynamics/General/LayoutConfigDefs.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/MeshData.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/MultiBodySimFile.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/MakeCoordinateSystem.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/VectorToSkewMatrix.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/MatrixHelpers.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/QuaternionHelpers.hpp
        #${COMMONSOURCE_DIR}/include/Dynamics/General/DynamicsSystem.hpp
        #${COMMONSOURCE_DIR}/include/Dynamics/General/DynamicsSystemBase.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/ExternalForces.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/RigidBodyId.hpp
        #${COMMONSOURCE_DIR}/include/Dynamics/General/RigidBody.hpp
        #${COMMONSOURCE_DIR}/include/Dynamics/General/RigidBodySolverData.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/RigidBodyContainer.hpp
        #${COMMONSOURCE_DIR}/include/Dynamics/General/MoreauTimeStepper.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/TimeStepperSettings.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/InitialConditionBodies.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/InertiaTensorCalculations.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/AddGyroTermVisitor.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/ContactFrame.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/CollisionData.hpp
        #${COMMONSOURCE_DIR}/include/Dynamics/Collision/CollisionSolver.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/CollisionFunctions.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Collider.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/ContactTag.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/ContactPercussion.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/AABB.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/Ray.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/SphereGeometry.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/PlaneGeometry.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/BoxGeometry.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/MeshGeometry.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry/HalfspaceGeometry.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/ContactParameterMap.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/ContactParameter.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/ContactModels.hpp
        #${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/ContactGraph.hpp
        #${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/ContactGraphNodeData.hpp
        #${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/ContactGraphVisitors.hpp not yet made!
        ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/GeneralGraph.hpp
        #${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/PercussionPool.hpp
        #${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/InclusionSolverNT.hpp
        #${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/InclusionSolverCO.hpp
        #${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/InclusionSolverCONoG.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/ProxFunctions.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/ConvexSets.hpp
        #${COMMONSOURCE_DIR}/include/Dynamics/Inclusion/InclusionSolverSettings.hpp
        #${COMMONSOURCE_DIR}/include/Dynamics/Buffers/FrontBackBuffer.hpp
        #${COMMONSOURCE_DIR}/include/Dynamics/Buffers/StateRecorder.hpp
        #${COMMONSOURCE_DIR}/include/Dynamics/Buffers/StateRecorderResampler.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/DynamicsState.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/RigidBodyState.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/RecorderSettings.hpp

        ${COMMONSOURCE_DIR}/include/Systems/SceneParser.hpp

        ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/StatePool.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/StatePoolVisBackFront.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/StateRingPoolVisBackFront.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/SharedBufferPlayback.hpp

        ${COMMONSOURCE_DIR}/include/Systems/SharedBufferDynSys.hpp
        
        
        ${COMMONSOURCE_DIR}/external/pugixml/src/pugixml.hpp
    )


    SET(${SRC}
        ${COMMONSOURCE_DIR}/src/Singeltons/FileManager.cpp

        ${COMMONSOURCE_DIR}/src/Common/CommonFunctions.cpp
        ${COMMONSOURCE_DIR}/src/Common/SimpleLogger.cpp
        ${COMMONSOURCE_DIR}/src/Common/ApplicationCLOptions.cpp
        ${COMMONSOURCE_DIR}/src/Common/BitCount.cpp

        ${COMMONSOURCE_DIR}/src/Dynamics/General/MultiBodySimFile.cpp
        #${COMMONSOURCE_DIR}/src/Dynamics/Inclusion/InclusionSolverCONoG.cpp
        #${COMMONSOURCE_DIR}/src/Dynamics/Inclusion/InclusionSolverCO.cpp
        #${COMMONSOURCE_DIR}/src/Dynamics/Collision/CollisionSolver.cpp
        #${COMMONSOURCE_DIR}/src/Dynamics/General/MoreauTimeStepper.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/Collision/Collider.cpp
        #${COMMONSOURCE_DIR}/src/Dynamics/Buffers/StateRecorder.cpp
        #${COMMONSOURCE_DIR}/src/Dynamics/Buffers/StatePoolVisBackFront.cpp
        #${COMMONSOURCE_DIR}/src/Dynamics/General/DynamicsSystem.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/General/InitialConditionBodies.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/Collision/CollisionData.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/Collision/ContactTag.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/Collision/Geometry/BoxGeometry.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/Collision/Geometry/Ray.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/Inclusion/ConvexSets.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/Inclusion/ContactModels.cpp
        ${COMMONSOURCE_DIR}/src/Dynamics/Inclusion/ContactParameterMap.cpp

        #${COMMONSOURCE_DIR}/src/Dynamics/Buffers/StatePoolVisBackFront.cpp
        
        
        ${COMMONSOURCE_DIR}/external/pugixml/src/pugixml.cpp
    )

    set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}}
        ${COMMONSOURCE_DIR}/include/Common
        ${COMMONSOURCE_DIR}/external/SRDelegates/include/
        ${COMMONSOURCE_DIR}/external/FastFunc/include/
        ${COMMONSOURCE_DIR}/external/getoptpp/
        ${COMMONSOURCE_DIR}/external/getoptpp/src/
        ${COMMONSOURCE_DIR}/external/pugixml/src
        ${COMMONSOURCE_DIR}/include/Dynamics/Buffers
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision
        ${COMMONSOURCE_DIR}/include/Dynamics/Collision/Geometry
        ${COMMONSOURCE_DIR}/include/Dynamics/General
        ${COMMONSOURCE_DIR}/include/Dynamics/Inclusion
        ${COMMONSOURCE_DIR}/include/Systems
        ${COMMONSOURCE_DIR}/include/Singeltons
        ${COMMONSOURCE_DIR}/include/States
    )
endmacro(INCLUDE_SIMULATION_FRAMEWORK_MPI)

MACRO(INCLUDE_SIMULATION_FRAMEWORK_CONVERTER SRC INC INCLUDE_DIRS COMMONSOURCE_DIR )

    SET(${INC}
        ${COMMONSOURCE_DIR}/include/Common/Exception.hpp
        ${COMMONSOURCE_DIR}/include/Common/foreach_macro.hpp
        ${COMMONSOURCE_DIR}/include/Common/PlatformDefines.hpp
        ${COMMONSOURCE_DIR}/include/Common/CommonFunctions.hpp
        ${COMMONSOURCE_DIR}/include/Common/Singleton.hpp
        ${COMMONSOURCE_DIR}/include/Common/BinaryFile.hpp
        ${COMMONSOURCE_DIR}/include/Common/CPUTimer.hpp
        ${COMMONSOURCE_DIR}/include/Common/SimpleLogger.hpp
        
        ${COMMONSOURCE_DIR}/include/Dynamics/General/MultiBodySimFile.hpp
        ${COMMONSOURCE_DIR}/include/Dynamics/General/RigidBodyId.hpp

        ${COMMONSOURCE_DIR}/include/Dynamics/Buffers/DynamicsState.hpp
    )


    SET(${SRC}
    
        
    
        ${COMMONSOURCE_DIR}/src/Common/CommonFunctions.cpp
        ${COMMONSOURCE_DIR}/src/Common/ApplicationCLOptions.cpp
        ${COMMONSOURCE_DIR}/src/Common/SimpleLogger.cpp
        
        ${COMMONSOURCE_DIR}/src/Dynamics/General/MultiBodySimFile.cpp
    )

    set(${INCLUDE_DIRS} ${${INCLUDE_DIRS}}
        ${COMMONSOURCE_DIR}/include/Common
        ${COMMONSOURCE_DIR}/include/Dynamics/Buffers
        ${COMMONSOURCE_DIR}/include/Dynamics/General
        ${COMMONSOURCE_DIR}/external/getoptpp/
        ${COMMONSOURCE_DIR}/external/getoptpp/src/
    )
endmacro(INCLUDE_SIMULATION_FRAMEWORK_CONVERTER)

