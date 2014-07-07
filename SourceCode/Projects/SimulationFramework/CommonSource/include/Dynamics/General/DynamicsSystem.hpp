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


class DynamicsSystemBase{
public:
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    using TimeStepperSettingsType      = TimeStepperSettings;
    using RecorderSettingsType         = RecorderSettings;
    using ContactParameterMapType      = ContactParameterMap;
    using ExternalForceListType        = ExternalForceList;
    using GlobalGeometryMapType        = std::unordered_map< unsigned int, typename RigidBodyType::GeometryType>;
    using RigidBodyContainerType       = RigidBodyContainer;
    using RigidBodySimContainerType    = RigidBodyContainerType;
    using RigidBodyStaticContainerType = RigidBodySimContainerType;
    using RigidBodyStatesContainerType = std::map<RigidBodyIdType, RigidBodyState>;
};

class DynamicsSystem : public DynamicsSystemBase{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    DynamicsSystem();
    ~DynamicsSystem();

    // General related variables
    double m_gravity;
    Vector3 m_gravityDir;

    ContactParameterMapType m_ContactParameterMap;
    ExternalForceListType m_externalForces; ///< Special class of function objects

    //All Global Geometries used in the System
    GlobalGeometryMapType m_globalGeometries;

    // All RigidBodies which are owned by this class!

    RigidBodySimContainerType m_SimBodies;    // Simulated Objects
    RigidBodyStaticContainerType m_Bodies;    // all not simulated objects

    //All initial conditions for all bodies
    //We need an order, which is sorted according to the id!
    RigidBodyStatesContainerType m_bodiesInitStates;


    void initializeLog(Logging::Log* pLog);

    inline void applyInitStatesToBodies();
    inline void applySimBodiesToDynamicsState(DynamicsState & state);
    inline void applyDynamicsStateToSimBodies(const DynamicsState & state);

    void initMassMatrixAndHTerm(); // MassMatrix is const

    void doFirstHalfTimeStep(PREC ts, PREC timestep);
    void doSecondHalfTimeStep(PREC te, PREC timestep);

    void getSettings(RecorderSettingsType & settingsRecorder) const;
    void getSettings(TimeStepperSettingsType &settingsTimestepper) const;
    void getSettings(InclusionSolverSettingsType &settingsInclusionSolver) const;
    void getSettings(TimeStepperSettingsType &settingsTimestepper, InclusionSolverSettingsType &settingsInclusionSolver) const;

    void setSettings(const RecorderSettingsType & settingsRecorder);
    void setSettings(const TimeStepperSettingsType &settingsTimestepper);
    void setSettings(const InclusionSolverSettingsType &settingsInclusionSolver);
    void setSettings(const TimeStepperSettingsType &settingsTimestepper, const InclusionSolverSettingsType &settingsInclusionSolver);

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


public:

     // Create SceneParser Modules
     template<typename TParser>
     std::tuple< std::unique_ptr<typename TParser::SettingsModuleType >,
                 std::unique_ptr<typename TParser::BodyModuleType >,
                 std::unique_ptr<typename TParser::InitStatesModuleType >,
                 std::unique_ptr<typename TParser::GeometryModuleType >,
                 std::unique_ptr<typename TParser::ExternalForcesModuleType >,
                 std::unique_ptr<typename TParser::ContactParamModuleType>
              >
     createParserModules(TParser * p){

        using SettingsModuleType       = typename TParser::SettingsModuleType ;
        using ContactParamModuleType   = typename TParser::ContactParamModuleType;
        using GeometryModuleType       = typename TParser::GeometryModuleType ;
        using InitStatesModuleType     = typename TParser::InitStatesModuleType ;
        using ExternalForcesModuleType = typename TParser::ExternalForcesModuleType ;
        using BodyModuleType           = typename TParser::BodyModuleType ;
        using VisModType               = typename TParser::BodyModuleType::VisModType ;


        auto sett = std::unique_ptr<SettingsModuleType >(new SettingsModuleType(p, &m_SettingsRecorder,
                                                                                   &m_SettingsTimestepper,
                                                                                   &m_SettingsInclusionSolver));

        auto geom = std::unique_ptr<GeometryModuleType >(new GeometryModuleType(p, &m_globalGeometries) );

        auto is = std::unique_ptr<InitStatesModuleType >(new InitStatesModuleType(p,&m_bodiesInitStates, sett.get()));

        auto bm = std::unique_ptr<BodyModuleType>(new BodyModuleType(p,  geom.get(), is.get(), std::unique_ptr<VisModType>(nullptr), &m_SimBodies, &m_Bodies )) ;
        auto es = std::unique_ptr<ExternalForcesModuleType >(new ExternalForcesModuleType(p, &m_externalForces));
        auto con = std::unique_ptr<ContactParamModuleType>(new ContactParamModuleType(p,&m_ContactParameterMap));

        return std::make_tuple(std::move(sett),std::move(bm),std::move(is),std::move(geom),std::move(es),std::move(con));
     }

protected:

    RecorderSettingsType m_SettingsRecorder;
    TimeStepperSettingsType m_SettingsTimestepper;
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
//    for(auto it = m_bodiesInitStates.begin(); it!=m_bodiesInitStates.end();it++){
//                LOG(&std::cout, "\t---> state id: " << RigidBodyId::getBodyIdString(it->first)
//                    << std::endl << "\t\t---> q: " << it->second.m_q.transpose()
//                    << std::endl << "\t\t---> u: " << it->second.m_u.transpose() << std::endl;
//                    );
//    }
    InitialConditionBodies::applyBodyStatesTo(m_bodiesInitStates, m_SimBodies);
}

inline void DynamicsSystem::applySimBodiesToDynamicsState(DynamicsState & state) {
    InitialConditionBodies::applyBodiesTo<RigidBodyType, RigidBodySimContainerType>(m_SimBodies,state);
}

inline void DynamicsSystem::applyDynamicsStateToSimBodies(const DynamicsState & state) {
    ERRORMSG("NOT USED");
}

#endif
