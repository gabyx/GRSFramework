#ifndef DynamicsSystemMPI_hpp
#define DynamicsSystemMPI_hpp

#include <vector>
#include <list>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include RigidBody_INCLUDE_FILE
#include "RigidBodyContainer.hpp"
#include "ContactParameterMap.hpp"
#include "SimpleLogger.hpp"
#include "ExternalForces.hpp"

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

    // All global RigidBodies Container for this Process, these bodies which are owned by this class!"============================
    typedef RigidBodyContainer<RigidBodyType *> RigidBodySimContainerType;
    RigidBodySimContainerType m_SimBodies;        // simulated objects
    RigidBodySimContainerType m_RemoteSimBodies;  // all remote bodies

    typedef RigidBodySimContainerType RigidBodyStaticContainer;
    RigidBodyStaticContainer m_Bodies;        // all not simulated objects
    // ============================================================================




    inline void addSimBodyPtr(RigidBodyType * ptr ) { m_SimBodies.addBody(ptr); }
    inline void addBodyPtr(RigidBodyType * ptr ) { m_Bodies.addBody(ptr); }

    void initializeLog(Logging::Log* pLog);

    void initMassMatrixAndHTerm();


    void doFirstHalfTimeStep(PREC ts, PREC timestep);
    void doSecondHalfTimeStep(PREC te, PREC timestep);

    void getSettings(RecorderSettings & settingsRecorder) const;
    void setSettings(const RecorderSettings & settingsRecorder);
    void getSettings(TimeStepperSettings &settingsTimestepper, InclusionSolverSettings &settingsInclusionSolver);
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

#endif
