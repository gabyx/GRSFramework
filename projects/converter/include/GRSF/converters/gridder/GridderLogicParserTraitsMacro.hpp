#ifndef GRSF_converters_gridder_GridderLogicParserTraitsMacro_hpp
#define GRSF_converters_gridder_GridderLogicParserTraitsMacro_hpp

#include "GRSF/common/SimpleLogger.hpp"
#include <pugixml.hpp>


#include "GRSF/general/LogicParserBaseTraitsMacro.hpp"

#define DEFINE_GRIDDERLOGICPARSER_TYPE_TRAITS( TParserTraits )  \
    DEFINE_LOGICPARSER_BASE_TYPE_TRAITS( TParserTraits ) \
    using LogicModuleType         = typename TParserTraits::LogicModuleType;\


#endif
