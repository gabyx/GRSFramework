#ifndef DynamicsSystem_hpp
#define DynamicsSystem_hpp

#include <vector>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include RigidBody_INCLUDE_FILE
//#include "RigidBodyId.hpp" //Not used

#include "RigidBodyContainer.hpp"
#include "DynamicsState.hpp"
#include "ContactParameterMap.hpp"

#include "AddGyroTermVisitor.hpp"
#include "InitialConditionBodies.hpp"
#include "InclusionSolverSettings.hpp"
#include "CommonFunctions.hpp"

#include "ExternalForces.hpp"

#include "RecorderSettings.hpp"
#include "TimeStepperSettings.hpp"

#include "SimpleLogger.hpp"




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
    typedef std::map< unsigned int /* id */, typename RigidBodyType::GeometryType> GlobalGeometryMapType;
    GlobalGeometryMapType m_globalGeometries;

    // All RigidBodies which are owned by this class!
    typedef RigidBodyContainer RigidBodySimContainerType;
    RigidBodySimContainerType m_SimBodies;    // Simulated Objects
    typedef RigidBodySimContainerType RigidBodyNotAniContainer;
    RigidBodyNotAniContainer m_Bodies;    // all not simulated objects

    void initializeLog(Logging::Log* pLog);


    inline void applySimBodiesToDynamicsState(DynamicsState & state);
    inline void applyDynamicsStateToSimBodies(const DynamicsState & state);

    void initMassMatrixAndHTerm(); // MassMatrix is const

    void doFirstHalfTimeStep(PREC ts, PREC timestep);
    void doSecondHalfTimeStep(PREC te, PREC timestep);

    void getSettings(RecorderSettings & SettingsRecorder) const;
    void setSettings(const RecorderSettings & SettingsRecorder);
    void getSettings(TimeStepperSettings &SettingsTimestepper, InclusionSolverSettings &SettingsInclusionSolver) const;
    void setSettings(const TimeStepperSettings &SettingsTimestepper, const InclusionSolverSettings &SettingsInclusionSolver);

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


inline void DynamicsSystem::applySimBodiesToDynamicsState(DynamicsState & state) {
    InitialConditionBodies::applyBodiesToDynamicsState<RigidBodyType, RigidBodySimContainerType>(m_SimBodies,state);
}

inline void DynamicsSystem::applyDynamicsStateToSimBodies(const DynamicsState & state) {
    InitialConditionBodies::applyDynamicsStateToBodies<RigidBodyType, RigidBodySimContainerType>(state,m_SimBodies);
}

#endif
