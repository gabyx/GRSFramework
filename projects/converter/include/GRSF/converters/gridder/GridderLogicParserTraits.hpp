#ifndef GRSF_converters_gridder_GridderLogicParserTraits_hpp
#define GRSF_converters_gridder_GridderLogicParserTraits_hpp

#include "GRSF/general/LogicParserBaseTraits.hpp"

#include "GRSF/converters/gridder/GridderLogicParserModules.hpp"

/** The traits for a standart RenderLogicParser class*/
template<typename TSceneParser, typename TDataStorage>
struct GridderLogicParserTraits : LogicParserBaseTraits<TSceneParser,TDataStorage> {

    using LogicModuleType       = typename GridderLogicParserModules::LogicModule<GridderLogicParserTraits>;

    using TupleModules = std::tuple< std::unique_ptr<LogicModuleType> > ;

    template<unsigned int N>
    using getModuleType =  typename std::tuple_element<N,TupleModules>::type::element_type;

};

#include "GRSF/converters/gridder/GridderLogicParserTraitsMacro.hpp"



#endif
