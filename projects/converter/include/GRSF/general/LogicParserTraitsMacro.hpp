#ifndef GRSF_general_LogicParserTraitsMacro_hpp
#define GRSF_general_LogicParserTraitsMacro_hpp

#include "GRSF/general/LogicParserBaseTraitsMacro.hpp"

#define DEFINE_LOGICPARSER_TYPE_TRAITS( TParserTraits )  \
    DEFINE_LOGICPARSER_BASE_TYPE_TRAITS( TParserTraits ) \
    using LogicModuleType         = typename TParserTraits::LogicModuleType;\

#endif
