#ifndef GRSF_general_LogicParserTraits_hpp
#define GRSF_general_LogicParserTraits_hpp

#include "GRSF/general/LogicParserBaseTraits.hpp"
#include "GRSF/general/LogicParserModules.hpp"

/** The traits for a standart LogicParser class*/
template<typename TSceneParser, typename TDataStorage>
struct LogicParserTraits : LogicParserBaseTraits<TSceneParser,TDataStorage> {

    // LogicModule does not use the TDataStorage ( class where all stuff is stored from parsing)

    // Module typedefs
    using LogicModuleType       = typename LogicParserModules::LogicModule<LogicParserTraits>;

    using TupleModules = std::tuple< std::unique_ptr<LogicModuleType> > ;

    template<unsigned int N>
    using getModuleType =  typename std::tuple_element<N,TupleModules>::type::element_type;

};

#include "GRSF/general/LogicParserTraitsMacro.hpp"

#endif
