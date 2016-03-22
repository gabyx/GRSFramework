#ifndef GRSF_General_RenderLogicParserTraits_hpp
#define GRSF_General_RenderLogicParserTraits_hpp

#include "GRSF/General/LogicParserBaseTraits.hpp"

#include "GRSF/Converters/Renderer/RenderLogicParserModules.hpp"

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

#include "GRSF/Converters/Renderer/RenderLogicParserTraitsMacro.hpp"



#endif
