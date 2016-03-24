#ifndef GRSF_dynamics_general_DynamicsSystemBase_hpp
#define GRSF_dynamics_general_DynamicsSystemBase_hpp


#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/LogDefines.hpp"

#include <map>
#include <unordered_map>
#include <vector>

#include RigidBody_INCLUDE_FILE

#include "GRSF/dynamics/general/RigidBodyContainer.hpp"
#include "GRSF/dynamics/inclusion/ContactParameterMap.hpp"
#include "GRSF/dynamics/general/ExternalForces.hpp"

#include "GRSF/dynamics/buffers/RecorderSettings.hpp"
#include InclusionSolverSettings_INCLUDE_FILE
#include "GRSF/dynamics/general/TimeStepperSettings.hpp"

#include "GRSF/dynamics/buffers/DynamicsState.hpp"
#include "GRSF/dynamics/buffers/RigidBodyState.hpp"
#include "GRSF/dynamics/general/InitialConditionBodies.hpp"

/** This is the define for all DynamicsSystem classes
* It only consists of several types which are essetial.
*/
#define  DEFINE_DYNAMICSYSTEM_BASE_TYPES  \
     DEFINE_DYNAMICSSYTEM_CONFIG_TYPES\
    using TimeStepperSettingsType      = TimeStepperSettings;\
    using RecorderSettingsType         = RecorderSettings;\
    using ContactParameterMapType      = ContactParameterMap;\
    using ExternalForceListType        = ExternalForceList;\
    using GlobalGeometryMapType        = std::unordered_map< unsigned int, typename RigidBodyType::GeometryType>;\
    using RigidBodyContainerType       = RigidBodyContainer;\
    using RigidBodySimContainerType    = RigidBodyContainerType;\
    using RigidBodyStaticContainerType = RigidBodySimContainerType;\
    using RigidBodyStatesVectorType    = StdVecAligned<RigidBodyState>; \
    using RigidBodyStatesContainerType = StdUMapAligned<RigidBodyIdType, RigidBodyState>;


class DynamicsSystemBase {
public:

    DEFINE_DYNAMICSYSTEM_BASE_TYPES
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DynamicsSystemBase();
    ~DynamicsSystemBase();

    ContactParameterMapType m_ContactParameterMap;
    ExternalForceListType m_externalForces; ///< Special class of function objects

    //All Global Geometries used in the System
    GlobalGeometryMapType m_globalGeometries;

    // All RigidBodies which are owned by this class!
    RigidBodySimContainerType m_simBodies;    // Simulated Objects
    RigidBodyStaticContainerType m_staticBodies;    // all not simulated objects

    //All initial conditions for all bodies
    //We need an order, which is sorted according to the id!
    RigidBodyStatesContainerType m_bodiesInitStates;

    void initializeLog(Logging::Log* pLog);

    inline void applyInitStatesToBodies(){
        InitialConditionBodies::applyBodyStatesTo(m_bodiesInitStates, m_simBodies);
    }
    inline void applySimBodiesToDynamicsState(DynamicsState & state) {
        state.applyBodies<true>(m_simBodies);
    }



    inline const RecorderSettingsType & getSettingsRecorder() { return m_settingsRecorder;}
    inline const TimeStepperSettingsType & getSettingsTimeStepper() {return m_settingsTimestepper;}
    inline const InclusionSolverSettingsType & getSettingsInclusionSolver() { return m_settingsInclusionSolver;}

    inline void setStartTime(PREC startTime){ m_settingsTimestepper.m_startTime = startTime;}

    void reset();
    void resetEnergy();

    PREC m_currentTotEnergy;
    PREC m_currentPotEnergy;
    PREC m_currentKinEnergy;
    PREC m_currentTransKinEnergy;
    PREC m_currentRotKinEnergy;
    PREC m_currentSpinNorm;


protected:

    RecorderSettingsType m_settingsRecorder;
    TimeStepperSettingsType m_settingsTimestepper;
    InclusionSolverSettingsType m_settingsInclusionSolver;

    // Log
    Logging::Log*	m_pSolverLog;
};




#endif // DynamicsSystemBase_hpp
