﻿#ifndef DynamicsSystem_hpp
#define DynamicsSystem_hpp

#include <vector>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include RigidBody_INCLUDE_FILE

#include "RigidBodyContainer.hpp"
#include "DynamicsState.hpp"
#include "ContactParameterMap.hpp"
#include "ExternalForces.hpp"
#include "SimpleLogger.hpp"

#include "InitialConditionBodies.hpp"

#include "RecorderSettings.hpp"
#include "InclusionSolverSettings.hpp"
#include "TimeStepperSettings.hpp"


class DynamicsSystem {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    DynamicsSystem(){};
    ~DynamicsSystem();

    // General related variables
    double m_gravity;
    Vector3 m_gravityDir;

    ContactParameterMap m_ContactParameterMap;

    typedef ExternalForceList ExternalForceListType;
    ExternalForceListType m_externalForces; ///< Special class of function objects

    //All Global Geometries used in the System
    typedef std::unordered_map< unsigned int /* id */, typename RigidBodyType::GeometryType> GlobalGeometryMapType;
    GlobalGeometryMapType m_globalGeometries;

    // All RigidBodies which are owned by this class!
    typedef RigidBodyContainer RigidBodyContainerType;
    typedef RigidBodyContainerType RigidBodySimContainerType;
    RigidBodySimContainerType m_SimBodies;    // Simulated Objects
    typedef RigidBodySimContainerType RigidBodyStaticContainer;
    RigidBodyStaticContainer m_Bodies;    // all not simulated objects

    //All initial conditions for all bodies
    //We need an order, which is sorted according to the id!
    typedef std::map<RigidBodyIdType, RigidBodyState> RigidBodyStatesContainerType;
    RigidBodyStatesContainerType m_simBodiesInitStates;


    void initializeLog(Logging::Log* pLog);

    inline void applyInitStatesToBodies();
    inline void applySimBodiesToDynamicsState(DynamicsState & state);
    inline void applyDynamicsStateToSimBodies(const DynamicsState & state);

    void initMassMatrixAndHTerm(); // MassMatrix is const

    void doFirstHalfTimeStep(PREC ts, PREC timestep);
    void doSecondHalfTimeStep(PREC te, PREC timestep);

    void getSettings(RecorderSettings & settingsRecorder) const;
    void setSettings(const RecorderSettings & settingsRecorder);
    void getSettings(TimeStepperSettings &settingsTimestepper, InclusionSolverSettings &settingsInclusionSolver) const;
    void setSettings(const TimeStepperSettings &settingsTimestepper, const InclusionSolverSettings &settingsInclusionSolver);

    void reset();
    inline  void afterFirstTimeStep() {};
    inline  void afterSecondTimeStep() {};
    void doInputTimeStep(PREC T) {};

    double m_CurrentStateEnergy;

protected:

    RecorderSettings m_SettingsRecorder;
    TimeStepperSettings m_SettingsTimestepper;
    InclusionSolverSettings m_SettingsInclusionSolver;

    //Function
    //This is a minimal update of F, no checking if constant values are correct
    void updateFMatrix(const Quaternion & q, Matrix43 & F_i);


    // Log
    Logging::Log*	m_pSolverLog;
    std::stringstream logstream;
};

inline void DynamicsSystem::applyInitStatesToBodies(){
    // Apply all init states to the sim bodies
    InitialConditionBodies::applyRigidBodyStatesToBodies(m_SimBodies, m_simBodiesInitStates);
}

inline void DynamicsSystem::applySimBodiesToDynamicsState(DynamicsState & state) {
    InitialConditionBodies::applyBodiesToDynamicsState<RigidBodyType, RigidBodySimContainerType>(m_SimBodies,state);
}

inline void DynamicsSystem::applyDynamicsStateToSimBodies(const DynamicsState & state) {
    ERRORMSG("NOT USED")
    //InitialConditionBodies::applyDynamicsStateToBodies<RigidBodyType, RigidBodySimContainerType>(state,m_SimBodies);
}

#endif
