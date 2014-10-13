#ifndef DynamicsSystem_hpp
#define DynamicsSystem_hpp


#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include <map>
#include <unordered_map>
#include <vector>

#include "SceneParser.hpp"

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
    using RigidBodyStatesContainerType = std::map<RigidBodyIdType, RigidBodyState>;


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

    inline void applyInitStatesToBodies();
    inline void applySimBodiesToDynamicsState(DynamicsState & state);
    inline void applyDynamicsStateToSimBodies(const DynamicsState & state);

    void initMassMatrixAndHTerm(); // MassMatrix is const

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

    RecorderSettingsType m_settingsRecorder;
    TimeStepperSettingsType m_settingsTimestepper;
    InclusionSolverSettingsType m_settingsInclusionSolver;

    //Function
    //This is a minimal update of F, no checking if constant values are correct
    void updateFMatrix(const Quaternion & q, Matrix43 & F_i);

    // Log
    Logging::Log*	m_pSolverLog;
};



inline void DynamicsSystemBase::applyInitStatesToBodies() {
    InitialConditionBodies::applyBodyStatesTo(m_bodiesInitStates, m_simBodies);
}

inline void DynamicsSystemBase::applySimBodiesToDynamicsState(DynamicsState & state) {
    state.applyBodies<true>(m_simBodies);
}


class DynamicsSystem : public DynamicsSystemBase {
public:

    struct SceneParserCreator{
        SceneParserCreator( DynamicsSystem * p): m_p(p){}
        DynamicsSystem * m_p;

        template<typename TSceneParser, typename TDynamicsSystem>
        struct SceneParserTraits : SceneParserBaseTraits<TSceneParser,TDynamicsSystem> {
            // Module typedefs
            using SettingsModuleType         = ParserModules::SettingsModule<SceneParserTraits>;
            using ExternalForcesModuleType   = ParserModules::ExternalForcesModule<SceneParserTraits>;
            using ContactParamModuleType     = ParserModules::ContactParamModule<SceneParserTraits>;
            using InitStatesModuleType       = ParserModules::InitStatesModule<SceneParserTraits> ;

            using BodyMStaticOptions         = ParserModules::BodyModuleStaticOptions<>;
            using BodyModuleType             = ParserModules::BodyModule< SceneParserTraits, BodyMStaticOptions > ;

            using GeomMStaticOptions         = ParserModules::GeometryModuleStaticOptions<>;
            using GeometryModuleType         = ParserModules::GeometryModule<SceneParserTraits,GeomMStaticOptions>;

            using VisModuleType              = ParserModules::VisModuleDummy<SceneParserTraits>;
            using MPIModuleType              = ParserModules::MPIModuleDummy<SceneParserTraits>;
        };


        template<typename TParser>
        std::tuple< std::unique_ptr<typename TParser::SettingsModuleType >,
            std::unique_ptr<typename TParser::ExternalForcesModuleType >,
            std::unique_ptr<typename TParser::ContactParamModuleType>,
            std::unique_ptr<typename TParser::InitStatesModuleType >,
            std::unique_ptr<typename TParser::BodyModuleType >,
            std::unique_ptr<typename TParser::GeometryModuleType >,
            std::unique_ptr<typename TParser::VisModuleType>,
            std::unique_ptr<typename TParser::MPIModuleType>
            >
        createParserModules(TParser * p) {

            using SettingsModuleType       = typename TParser::SettingsModuleType ;
            using ContactParamModuleType   = typename TParser::ContactParamModuleType;
            using GeometryModuleType       = typename TParser::GeometryModuleType ;
            using InitStatesModuleType     = typename TParser::InitStatesModuleType ;
            using ExternalForcesModuleType = typename TParser::ExternalForcesModuleType ;
            using BodyModuleType           = typename TParser::BodyModuleType ;
            using VisModuleType            = typename TParser::VisModuleType ;
            using MPIModuleType            = typename TParser::MPIModuleType ;

            auto sett = std::unique_ptr<SettingsModuleType >(new SettingsModuleType(p, &m_p->m_settingsRecorder,
                        &m_p->m_settingsTimestepper,
                        &m_p->m_settingsInclusionSolver));

            auto geom = std::unique_ptr<GeometryModuleType >(new GeometryModuleType(p, &m_p->m_globalGeometries) );

            auto is  = std::unique_ptr<InitStatesModuleType >(new InitStatesModuleType(p,&m_p->m_bodiesInitStates, sett.get()));
            auto vis = std::unique_ptr<VisModuleType>(nullptr); // no visualization needed
            auto bm  = std::unique_ptr<BodyModuleType>(new BodyModuleType(p,  geom.get(), is.get(), vis.get() , &m_p->m_simBodies, &m_p->m_staticBodies )) ;
            auto es  = std::unique_ptr<ExternalForcesModuleType >(new ExternalForcesModuleType(p, &m_p->m_externalForces));
            auto con = std::unique_ptr<ContactParamModuleType>(new ContactParamModuleType(p,&m_p->m_ContactParameterMap));

            auto mpi = std::unique_ptr<MPIModuleType>(nullptr);

            return std::make_tuple(std::move(sett),std::move(es),std::move(con),std::move(is),std::move(bm),std::move(geom),std::move(vis),std::move(mpi));
        };

    };


};

#endif
