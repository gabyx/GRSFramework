#ifndef RenderScriptParserBaseTraits_hpp
#define RenderScriptParserBaseTraits_hpp

#include "GRSF/Common/SimpleLogger.hpp"
#include <pugixml.hpp>

#define DEFINE_RENDERLOGICPARSER_BASE_TYPE_TRAITS( TParserTraits ) \
    using ParserType = typename TParserTraits::ParserType; \
    using CollectionType = typename TParserTraits::CollectionType; \
    using LogType = typename TParserTraits::LogType; \
    using XMLNodeType = typename TParserTraits::XMLNodeType;\
    using XMLNodeItType = typename TParserTraits::XMLNodeItType;\
    using XMLAttributeType = typename TParserTraits::XMLAttributeType;\
    using RandomGenType = typename TParserTraits::RandomGenType; \
    template<typename T> using UniformDistType = typename TParserTraits::template UniformDistType<T>;

/** The base traits for every MaterialsCollectionParser parser */
template<typename TSceneParser, typename TCollection>
struct RenderLogicParserBaseTraits {


    using CollectionType = TCollection;

    using ParserType = TSceneParser;
    using LogType = Logging::Log;

    using XMLNodeType = pugi::xml_node;
    using XMLNodeItType = pugi::xml_node_iterator;
    using XMLAttributeType = pugi::xml_attribute;

    using RandomGenType = typename CollectionType::RandomGenType;
    template<typename T>
    using UniformDistType = std::uniform_real_distribution<T>;
};



#define DEFINE_RENDERLOGICPARSER_TYPE_TRAITS( TParserTraits )  \
    DEFINE_RENDERLOGICPARSER_BASE_TYPE_TRAITS( TParserTraits ) \
    using MaterialsModuleType     = typename TParserTraits::MaterialsModuleType;\
    using LogicModuleType         = typename TParserTraits::LogicModuleType;\


#endif
