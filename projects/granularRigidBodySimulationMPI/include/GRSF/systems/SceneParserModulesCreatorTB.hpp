#ifndef GRSF_systems_SceneParserModulesCreatorTB_hpp
#define GRSF_systems_SceneParserModulesCreatorTB_hpp


#include "GRSF/systems/SceneParserModulesMPI.hpp"


template< typename TTopoBuilder >
struct ParserModulesCreatorTopoBuilder{

    ParserModulesCreatorTopoBuilder( TTopoBuilder * p): m_p(p){}
    TTopoBuilder * m_p;

    template<typename TTSceneParser, typename TDynamicsSystem>
    struct SceneParserTraits : SceneParserBaseTraits<TTSceneParser,TDynamicsSystem> {

        using SettingsModuleType         = ParserModules::SettingsModuleDummy<SceneParserTraits>;
        using ExternalForcesModuleType   = ParserModules::ExternalForcesModuleDummy<SceneParserTraits>;
        using ContactParamModuleType     = ParserModules::ContactParamModuleDummy<SceneParserTraits>;
        using InitStatesModuleType       = ParserModules::InitStatesModule<SceneParserTraits> ;

        // Default templates
        //           bool parseAllIfRangeEmpty = true,
        //           bool parseAllBodiesNonSelGroup = true ,
        //           bool parseSimBodies = true,
        //           bool parseStaticBodies = true ,
        //           bool allocateSimBodies = true,
        //           bool allocateStaticBodies = true,
        //           bool parseInitialCondition = true

        //            m_parseAllBodiesNonSelGroup = true;
        //            m_parseSimBodies = true; m_allocateSimBodies = false;
        //            m_parseStaticBodies = false; m_allocateStaticBodies = false;
        using BodyMStaticOptions         = ParserModules::BodyModuleStaticOptions<true,true,true,false,false,false,true>;
        using BodyModuleType             = ParserModules::BodyModule< SceneParserTraits, BodyMStaticOptions > ;

        //using GeomMStaticOptions         = ParserModules::GeometryModuleStaticOptions<>;
        using GeometryModuleType         = ParserModules::GeometryModuleDummy<SceneParserTraits>;

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

        auto sett = std::unique_ptr<SettingsModuleType>(nullptr);
        auto geom = std::unique_ptr<GeometryModuleType >(nullptr);
        auto vis = std::unique_ptr<VisModuleType>(nullptr);
        auto es  = std::unique_ptr<ExternalForcesModuleType >(nullptr);
        auto con = std::unique_ptr<ContactParamModuleType>(nullptr);
        auto mpi = std::unique_ptr<MPIModuleType>( nullptr );

        auto is  = std::unique_ptr<InitStatesModuleType >(new InitStatesModuleType(p, &m_p->m_initStates, &m_p->m_timeStepperSettings ));
        auto bm  = std::unique_ptr<BodyModuleType>(new BodyModuleType(p,  nullptr , is.get(), nullptr , nullptr , nullptr )) ;

        return std::make_tuple(std::move(sett),std::move(es),std::move(con),std::move(is),std::move(bm),std::move(geom),std::move(vis),std::move(mpi));
    }
};





#endif
