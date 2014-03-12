#ifndef DynamicsSystem_hpp
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
#include InclusionSolverSettings_INCLUDE_FILE
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
    void getSettings(TimeStepperSettings &settingsTimestepper) const;
    void getSettings(InclusionSolverSettingsType &settingsInclusionSolver) const;
    void getSettings(TimeStepperSettings &settingsTimestepper, InclusionSolverSettingsType &settingsInclusionSolver) const;

    void setSettings(const RecorderSettings & settingsRecorder);
    void setSettings(const TimeStepperSettings &settingsTimestepper);
    void setSettings(const InclusionSolverSettingsType &settingsInclusionSolver);
    void setSettings(const TimeStepperSettings &settingsTimestepper, const InclusionSolverSettingsType &settingsInclusionSolver);

    void reset();
    inline  void afterFirstTimeStep() {};
    inline  void afterSecondTimeStep() {};
    void doInputTimeStep(PREC T) {};

    PREC m_currentTotEnergy;
    PREC m_currentPotEnergy;
    PREC m_currentKinEnergy;
    PREC m_currentTransKinEnergy;
    PREC m_currentRotKinEnergy;
    PREC m_currentSpinNorm;

protected:

    RecorderSettings m_SettingsRecorder;
    TimeStepperSettings m_SettingsTimestepper;
    InclusionSolverSettingsType m_SettingsInclusionSolver;

    //Function
    //This is a minimal update of F, no checking if constant values are correct
    void updateFMatrix(const Quaternion & q, Matrix43 & F_i);


    // Log
    Logging::Log*	m_pSolverLog;
    std::stringstream logstream;
};

inline void DynamicsSystem::applyInitStatesToBodies(){
    // Apply all init states to the sim bodies
//    for(auto it = m_simBodiesInitStates.begin(); it!=m_simBodiesInitStates.end();it++){
//                LOG(&std::cout, "\t---> state id: " << RigidBodyId::getBodyIdString(it->first)
//                    << std::endl << "\t\t---> q: " << it->second.m_q.transpose()
//                    << std::endl << "\t\t---> u: " << it->second.m_u.transpose() << std::endl;
//                    );
//    }
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
