#ifndef SceneParserMPI_hpp
#define SceneParserMPI_hpp

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

template<typename TDynamicsSystem>
class SceneParserMPI: public SceneParser<TDynamicsSystem, SceneParserMPITraits, SceneParserMPI<TDynamicsSystem> > {
private:
    using BaseType = SceneParser<TDynamicsSystem, SceneParserMPITraits, SceneParserMPI<TDynamicsSystem> >;
public:
    using DynamicsSystemType = TDynamicsSystem;

public:
    template<typename ModuleGeneratorType>
    SceneParserMPI(ModuleGeneratorType & moduleGen, Logging::Log * log): BaseType(moduleGen, log){

    }
};




#endif
