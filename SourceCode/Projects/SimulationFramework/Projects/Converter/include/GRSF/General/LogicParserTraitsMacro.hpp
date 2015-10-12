#ifndef GRSF_General_LogicParserBaseTraitsMacro_hpp
#define GRSF_General_LogicParserBaseTraitsMacro_hpp

#include "GRSF/General/LogicParserBaseTraitsMacro.hpp"

#define DEFINE_LOGICPARSER_TYPE_TRAITS( TParserTraits )  \
    DEFINE_LOGICPARSER_BASE_TYPE_TRAITS( TParserTraits ) \
    using LogicModuleType         = typename TParserTraits::LogicModuleType;\

#endif
