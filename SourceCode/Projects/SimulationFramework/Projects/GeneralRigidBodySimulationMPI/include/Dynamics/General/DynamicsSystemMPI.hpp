#ifndef DynamicsSystemMPI_hpp
#define DynamicsSystemMPI_hpp

#include <vector>
#include <list>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "SceneParserModules.hpp"
#include "SceneParserModulesMPI.hpp"

#include RigidBody_INCLUDE_FILE

#include "RigidBodyContainer.hpp"
#include "ContactParameterMap.hpp"
#include "SimpleLogger.hpp"
#include "ExternalForces.hpp"

#include "InitialConditionBodies.hpp"

#include "RecorderSettings.hpp"
#include InclusionSolverSettings_INCLUDE_FILE
#include "TimeStepperSettings.hpp"

#include "MPITopologyBuilderSettings.hpp"


class DynamicsSystemMPI {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    DynamicsSystemMPI();
    ~DynamicsSystemMPI();

    using RecorderSettingsType = RecorderSettings;
    using TimeStepperSettingsType = TimeStepperSettings;
    using TopologyBuilderSettingsType = MPILayer::TopologyBuilderSettings;

    using ContactParameterMapType = ContactParameterMap;
    ContactParameterMapType m_ContactParameterMap;

    using ExternalForceListType = ExternalForceList;
    ExternalForceListType m_externalForces; ///< Special class of function objects

    //All Global Geometries used in the System
    typedef std::unordered_map< unsigned int /* id */, typename RigidBodyType::GeometryType> GlobalGeometryMapType;
    GlobalGeometryMapType m_globalGeometries;

    // All global RigidBodies Container for this Process, these bodies which are owned by this class!"============================
    using RigidBodyContainerType = RigidBodyContainer;
    using RigidBodySimContainerType = RigidBodyContainer;
    RigidBodySimContainerType m_simBodies;        // simulated objects
    RigidBodySimContainerType m_remoteSimBodies;  // all remote bodies

    void deleteSimBodies(); ///< deletes all local and remote bodies, static bodies are not considered!

    using RigidBodyStaticContainerType = RigidBodySimContainerType;
    RigidBodySimContainerType m_staticBodies;        // all not simulated objects
    // ============================================================================

    //All initial conditions for all bodies
    //We need an order, which is sorted according to the id!
    using RigidBodyStatesContainerType = std::map<RigidBodyIdType, RigidBodyState>;
    using RigidBodyStatesVectorType = std::vector<RigidBodyState>;
    RigidBodyStatesContainerType m_bodiesInitStates;


    inline void addSimBodyPtr(RigidBodyType * ptr ) { m_simBodies.addBody(ptr); }
    inline void addBodyPtr(RigidBodyType * ptr ) { m_staticBodies.addBody(ptr); }

    void initializeLog(Logging::Log* pLog);

    void applyInitStatesToBodies();

    void doFirstHalfTimeStep(PREC ts, PREC timestep);
    void doSecondHalfTimeStep(PREC te, PREC timestep);

    const RecorderSettingsType        & getSettingsRecorder() const;
    const TimeStepperSettingsType     & getSettingsTimeStepper() const;
    const InclusionSolverSettingsType & getSettingsInclusionSolver() const;
    const TopologyBuilderSettingsType & getSettingsTopoBuilder() const;

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


    RecorderSettings m_settingsRecorder;
    TimeStepperSettings m_settingsTimestepper;
    InclusionSolverSettingsType m_settingsInclusionSolver;
    TopologyBuilderSettingsType m_settingsTopologyBuilder;

    //Function
    //This is a minimal update of F, no checking if constant values are correct
    void updateFMatrix(const Quaternion & q, Matrix43 & F_i);

    // Log
    Logging::Log*	m_pSolverLog;

public:

     struct ParserModulesCreator1{
        ParserModulesCreator1( DynamicsSystemMPI * p): m_p(p){}
        DynamicsSystemMPI * m_p;

        template<typename TSceneParser, typename TDynamicsSystem>
        struct SceneParserTraits : SceneParserBaseTraits<TSceneParser,TDynamicsSystem> {
            // Module typedefs
            using SettingsModuleType         = ParserModules::SettingsModule<SceneParserTraits>;
            using ExternalForcesModuleType   = ParserModules::ExternalForcesModule<SceneParserTraits>;
            using ContactParamModuleType     = ParserModules::ContactParamModule<SceneParserTraits>;
            using InitStatesModuleType       = ParserModules::InitStatesModule<SceneParserTraits> ;

            // Default templates
            //           bool parseAllIfRangeEmpty = true,
            //           bool parseAllBodiesNonSelGroup = true ,
            //           bool parseSimBodies = true,
            //           bool parseStaticBodies = true ,
            //           bool allocateSimBodies = true,
            //           bool allocateStaticBodies = true,
            //           bool parseInitialCondition = true
            using BodyMStaticOptions         = ParserModules::BodyModuleStaticOptions<true,true,false,true,false,true,true>;
            using BodyModuleType             = ParserModules::BodyModule< SceneParserTraits, BodyMStaticOptions > ;

            using GeomMStaticOptions         = ParserModules::GeometryModuleStaticOptions<>;
            using GeometryModuleType         = ParserModules::GeometryModule<SceneParserTraits,GeomMStaticOptions>;

            using VisModuleType              = ParserModules::VisModuleDummy<SceneParserTraits>;
            using MPIModuleType              = ParserModules::MPIModule<SceneParserTraits>;
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

            auto mpi = std::unique_ptr<MPIModuleType>( new MPIModuleType(p,bm.get(),&m_p->m_settingsTopologyBuilder));

            return std::make_tuple(std::move(sett),std::move(es),std::move(con),std::move(is),std::move(bm),std::move(geom),std::move(vis),std::move(mpi));
        }
     };


     struct ParserModulesCreator2{
        ParserModulesCreator2( DynamicsSystemMPI * p): m_p(p){}
        DynamicsSystemMPI * m_p;

        template<typename TSceneParser, typename TDynamicsSystem>
        struct SceneParserTraits : SceneParserBaseTraits<TSceneParser,TDynamicsSystem> {
            // Module typedefs
            using SettingsModuleType         = ParserModules::SettingsModule<SceneParserTraits>;
            using ExternalForcesModuleType   = ParserModules::ExternalForcesModule<SceneParserTraits>;
            using ContactParamModuleType     = ParserModules::ContactParamModule<SceneParserTraits>;
            using InitStatesModuleType       = ParserModules::InitStatesModule<SceneParserTraits> ;

            // Default templates
            //           bool parseAllIfRangeEmpty = true,
            //           bool parseAllBodiesNonSelGroup = true ,
            //           bool parseSimBodies = true,
            //           bool parseStaticBodies = true ,
            //           bool allocateSimBodies = true,
            //           bool allocateStaticBodies = true,
            //           bool parseInitialCondition = true

            //        m_parseAllBodiesNonSelGroup = true;
            //        m_parseSimBodies = true; m_allocateSimBodies = true;
            //        m_parseStaticBodies = false; m_allocateStaticBodies = false;
            //        optBody.m_parseInitialCondition = false;
            //        m_parseAllIfRangeEmpty = false;
            using BodyMStaticOptions         = ParserModules::BodyModuleStaticOptions<false,true,true,false,true,false,false>;
            using BodyModuleType             = ParserModules::BodyModule< SceneParserTraits, BodyMStaticOptions > ;

            using GeomMStaticOptions         = ParserModules::GeometryModuleStaticOptions<>;
            using GeometryModuleType         = ParserModules::GeometryModule<SceneParserTraits,GeomMStaticOptions>;

            using VisModuleType              = ParserModules::VisModuleDummy<SceneParserTraits>;
            using MPIModuleType              = ParserModules::MPIModule<SceneParserTraits>;
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

            auto mpi = std::unique_ptr<MPIModuleType>( new MPIModuleType(p,bm.get(),&m_p->m_settingsTopologyBuilder));

            return std::make_tuple(std::move(sett),std::move(es),std::move(con),std::move(is),std::move(bm),std::move(geom),std::move(vis),std::move(mpi));
        }
     };

};


inline void DynamicsSystemMPI::applyInitStatesToBodies(){
    // Apply all init states to the sim bodies
    InitialConditionBodies::applyBodyStatesTo( m_bodiesInitStates, m_simBodies);
}


#endif
