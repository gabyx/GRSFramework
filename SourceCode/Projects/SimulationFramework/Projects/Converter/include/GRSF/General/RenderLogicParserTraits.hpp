#ifndef GRSF_General_RenderLogicParserTraits_hpp
#define GRSF_General_RenderLogicParserTraits_hpp

#include "GRSF/General/LogicParserBaseTraits.hpp"

#include "GRSF/General/RenderLogicParserModules.hpp"

/** The traits for a standart RenderLogicParser class*/
template<typename TSceneParser, typename TCollection>
struct RenderLogicParserTraits : LogicParserBaseTraits<TSceneParser,TCollection> {

    // Module typedefs
    using MaterialsModuleType   = typename RenderLogicParserModules::MaterialsModule<RenderLogicParserTraits>;
    using LogicModuleType       = typename RenderLogicParserModules::RenderLogicModule<RenderLogicParserTraits>;

    using TupleModules = std::tuple< std::unique_ptr<MaterialsModuleType> , std::unique_ptr<LogicModuleType> > ;

    template<unsigned int N>
    using getModuleType =  typename std::tuple_element<N,TupleModules>::type::element_type;
};

#include "GRSF/General/RenderLogicParserTraitsMacro.hpp"



#endif
