#ifndef GRSF_systems_SceneParserBaseTraits_hpp
#define GRSF_systems_SceneParserBaseTraits_hpp


#include <pugixml.hpp>

#include "GRSF/common/SimpleLogger.hpp"



#define DEFINE_PARSER_BASE_TYPE_TRAITS( TParserTraits ) \
    using ParserType = typename TParserTraits::ParserType; \
    using DynamicsSystemType = typename TParserTraits::DynamicsSystemType; \
    \
    using LogType = typename TParserTraits::LogType; \
    using XMLNodeType = typename TParserTraits::XMLNodeType;\
    using XMLNodeItType = typename TParserTraits::XMLNodeItType;\
    using XMLAttributeType = typename TParserTraits::XMLAttributeType;\
    using RandomGenType = typename TParserTraits::RandomGenType; \
    template<typename T> using UniformDistType = typename TParserTraits::template UniformDistType<T>;


/** The base traits for every scene parser */
template<typename TSceneParser, typename TDynamicsSystem>
struct SceneParserBaseTraits {
    using ParserType = TSceneParser;
    using DynamicsSystemType = TDynamicsSystem;

    using LogType = Logging::Log;

    using XMLNodeType = pugi::xml_node;
    using XMLNodeItType = pugi::xml_node_iterator;
    using XMLAttributeType = pugi::xml_attribute;

    using RandomGenType = typename DynamicsSystemType::RandomGenType;
    template<typename T>
    using UniformDistType = std::uniform_real_distribution<T>;
    template<typename T>
    using NormalDistType  = std::normal_distribution<T>;
};


#define  DEFINE_PARSER_TYPE_TRAITS( TParserTraits )  \
    DEFINE_PARSER_BASE_TYPE_TRAITS( TParserTraits ) \
    using SettingsModuleType     = typename TParserTraits::SettingsModuleType;\
    using GeometryModuleType     = typename TParserTraits::GeometryModuleType;\
    using ContactParamModuleType = typename TParserTraits::ContactParamModuleType;\
    \
    using InitStatesModuleType       = typename TParserTraits::InitStatesModuleType ;\
    using VisModuleType              = typename TParserTraits::VisModuleType;\
    using BodyModuleType             = typename TParserTraits::BodyModuleType;\
    using ExternalForcesModuleType   = typename TParserTraits::ExternalForcesModuleType ; \
    \
    using MPIModuleType              = typename TParserTraits::MPIModuleType;\



#endif
