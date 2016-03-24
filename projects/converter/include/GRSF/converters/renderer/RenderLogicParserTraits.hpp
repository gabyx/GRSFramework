#ifndef GRSF_converters_renderer_RenderLogicParserTraits_hpp
#define GRSF_converters_renderer_RenderLogicParserTraits_hpp

#include "GRSF/general/LogicParserBaseTraits.hpp"

#include "GRSF/converters/renderer/RenderLogicParserModules.hpp"

/** The traits for a standart RenderLogicParser class*/
template<typename TSceneParser, typename TDataStorage>
struct RenderLogicParserTraits : LogicParserBaseTraits<TSceneParser,TDataStorage> {

    // Module typedefs
    using MaterialsModuleType   = typename RenderLogicParserModules::MaterialsModule<RenderLogicParserTraits>;
    using LogicModuleType       = typename RenderLogicParserModules::LogicModule<RenderLogicParserTraits>;

    using TupleModules = std::tuple< std::unique_ptr<MaterialsModuleType> , std::unique_ptr<LogicModuleType> > ;

    template<unsigned int N>
    using getModuleType =  typename std::tuple_element<N,TupleModules>::type::element_type;
};

#include "GRSF/converters/renderer/RenderLogicParserTraitsMacro.hpp"



#endif
