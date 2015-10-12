#ifndef GRSF_General_RenderLogicParserTraitsMacro_hpp
#define GRSF_General_RenderLogicParserTraitsMacro_hpp

#include "GRSF/Common/SimpleLogger.hpp"
#include <pugixml.hpp>


#include "GRSF/General/LogicParserBaseTraitsMacro.hpp"

#define DEFINE_RENDERLOGICPARSER_TYPE_TRAITS( TParserTraits )  \
    DEFINE_LOGICPARSER_BASE_TYPE_TRAITS( TParserTraits ) \
    using MaterialsModuleType     = typename TParserTraits::MaterialsModuleType;\
    using LogicModuleType         = typename TParserTraits::LogicModuleType;\


#endif
