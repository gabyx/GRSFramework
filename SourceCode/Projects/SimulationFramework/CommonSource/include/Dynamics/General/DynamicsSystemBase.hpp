#ifndef DynamicsSystemBase_hpp
#define DynamicsSystemBase_hpp


#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include <map>
#include <unordered_map>
#include <vector>

#include RigidBody_INCLUDE_FILE

#include "RigidBodyContainer.hpp"
#include "ContactParameterMap.hpp"
#include "ExternalForces.hpp"

#include "RecorderSettings.hpp"
#include InclusionSolverSettings_INCLUDE_FILE
#include "TimeStepperSettings.hpp"

#include "DynamicsState.hpp"
#include "RigidBodyState.hpp"
#include "InitialConditionBodies.hpp"

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
    using RigidBodyStatesContainerType = StdMapAligned<RigidBodyIdType, RigidBodyState>;


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

    void doFirstHalfTimeStep(PREC ts, PREC timestep);
    void doSecondHalfTimeStep(PREC te, PREC timestep);

    const RecorderSettingsType        & getSettingsRecorder() const;
    const TimeStepperSettingsType     & getSettingsTimeStepper() const;
    const InclusionSolverSettingsType & getSettingsInclusionSolver() const;

    void getSettings(TimeStepperSettingsType &settingsTimestepper, InclusionSolverSettingsType &settingsInclusionSolver) const;

    void setSettings(const RecorderSettingsType & settingsRecorder);
    void setSettings(const TimeStepperSettingsType &settingsTimestepper);
    void setSettings(const InclusionSolverSettingsType &settingsInclusionSolver);
    void setSettings(const TimeStepperSettingsType &settingsTimestepper, const InclusionSolverSettingsType &settingsInclusionSolver);

    void reset();


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

    //Function
    //This is a minimal update of F, no checking if constant values are correct
    void updateFMatrix(const Quaternion & q, Matrix43 & F_i);

    // Log
    Logging::Log*	m_pSolverLog;
};




#endif // DynamicsSystemBase_hpp
