#ifndef GRSF_General_LogicParserBaseTraits_hpp
#define GRSF_General_LogicParserBaseTraits_hpp

#include "GRSF/Common/SimpleLogger.hpp"
#include <pugixml.hpp>


template<typename TSceneParser, typename TDataStorage>
struct LogicParserBaseTraits {

    using DataStorageType = TDataStorage;

    using ParserType = TSceneParser;
    using LogType = Logging::Log;

    using XMLNodeType = pugi::xml_node;
    using XMLNodeItType = pugi::xml_node_iterator;
    using XMLAttributeType = pugi::xml_attribute;
};


#include "GRSF/General/LogicParserBaseTraitsMacro.hpp"


#endif
