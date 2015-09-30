#ifndef GRSF_General_LogicParserTraits_hpp
#define GRSF_General_LogicParserTraits_hpp

#include "GRSF/General/LogicParserBaseTraits.hpp"
#include "GRSF/General/LogicParserModules.hpp"

/** The traits for a standart LogicParser class*/
template<typename TSceneParser, typename TCollection>
struct LogicParserTraits : LogicParserBaseTraits<TSceneParser,TCollection> {

    // LogicModule does not use the TCollection ( class where all stuff is stored from parsing)

    // Module typedefs
    using LogicModuleType       = typename LogicParserModules::LogicModule<LogicParserTraits>;

    using TupleModules = std::tuple< std::unique_ptr<LogicModuleType> > ;

    template<unsigned int N>
    using getModuleType =  typename std::tuple_element<N,TupleModules>::type::element_type;

};

#include "GRSF/General/LogicParserTraitsMacro.hpp"

#endif
