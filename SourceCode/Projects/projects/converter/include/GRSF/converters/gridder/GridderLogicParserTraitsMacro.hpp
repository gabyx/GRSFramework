#ifndef GRSF_General_GridderLogicParserTraitsMacro_hpp
#define GRSF_General_GridderLogicParserTraitsMacro_hpp

#include "GRSF/Common/SimpleLogger.hpp"
#include <pugixml.hpp>


#include "GRSF/General/LogicParserBaseTraitsMacro.hpp"

#define DEFINE_GRIDDERLOGICPARSER_TYPE_TRAITS( TParserTraits )  \
    DEFINE_LOGICPARSER_BASE_TYPE_TRAITS( TParserTraits ) \
    using LogicModuleType         = typename TParserTraits::LogicModuleType;\


#endif
