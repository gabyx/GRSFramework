#ifndef GRSF_General_LogicParserBaseTraits_hpp
#define GRSF_General_LogicParserBaseTraits_hpp

#include "GRSF/Common/SimpleLogger.hpp"
#include <pugixml.hpp>

/** The base traits for every MaterialsCollectionParser parser */
template<typename TSceneParser, typename TDataStorage>
struct LogicParserBaseTraits {

    using DataStorageType = TDataStorage;

    using ParserType = TSceneParser;
    using LogType = Logging::Log;

    using XMLNodeType = pugi::xml_node;
    using XMLNodeItType = pugi::xml_node_iterator;
    using XMLAttributeType = pugi::xml_attribute;
};


#define DEFINE_LOGICPARSER_BASE_TYPE_TRAITS( TParserTraits ) \
    using ParserType = typename TParserTraits::ParserType; \
    using DataStorageType = typename TParserTraits::DataStorageType; \
    using LogType = typename TParserTraits::LogType; \
    using XMLNodeType = typename TParserTraits::XMLNodeType;\
    using XMLNodeItType = typename TParserTraits::XMLNodeItType;\
    using XMLAttributeType = typename TParserTraits::XMLAttributeType;


#endif
