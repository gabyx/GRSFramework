#ifndef GMSF_Systems_SceneParserMPI_hpp
#define GMSF_Systems_SceneParserMPI_hpp

#include "SceneParser.hpp"

#include "SceneParserModulesMPI.hpp"

/** These module types are defined when there is no derivation from scene parser */
template<typename TSceneParser, typename TDynamicsSystem>
struct SceneParserMPITraits : public SceneParserBaseTraits<TSceneParser,TDynamicsSystem>{

    using SettingsModuleType         = ParserModules::SettingsModuleMPI<SceneParserMPITraits>;
    using ExternalForcesModuleType   = ParserModules::ExternalForcesModule<SceneParserMPITraits>;
    using ContactParamModuleType     = ParserModules::ContactParamModule<SceneParserMPITraits>;
    using InitStatesModuleType       = ParserModules::InitStatesModule<SceneParserMPITraits> ;

    using BodyModuleType             = ParserModules::BodyModule< SceneParserMPITraits > ;
    using GeometryModuleType         = ParserModules::GeometryModule<SceneParserMPITraits>;

    using VisModuleType              = ParserModules::VisModuleDummy<SceneParserMPITraits>;

    using MPIModuleType              = ParserModules::MPIModule<SceneParserMPITraits>;

};

template<typename TDynamicsSystem, template<typename P, typename D> class TParserTraits = SceneParserMPITraits>
class SceneParserMPI: public SceneParser<TDynamicsSystem, TParserTraits , SceneParserMPI<TDynamicsSystem> > {
private:
    using BaseType = SceneParser<TDynamicsSystem, TParserTraits, SceneParserMPI<TDynamicsSystem> >;
public:
    using DynamicsSystemType = TDynamicsSystem;

public:
    template<typename ModuleGeneratorType>
    SceneParserMPI(ModuleGeneratorType & moduleGen, Logging::Log * log): BaseType(moduleGen, log){

    }
};




#endif
