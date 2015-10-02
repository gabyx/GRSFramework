#ifndef GRSF_General_RenderLogicParserTraits_hpp
#define GRSF_General_RenderLogicParserTraits_hpp


#include "GRSF/General/LogicParserTraits.hpp"


/** The traits for a standart AnalyzerLogicParser class*/
template<typename TSceneParser, typename TDataStorage>
struct AnalyzerLogicParserTraits : LogicParserTraits<TSceneParser,TDataStorage> {


};

#include "GRSF/General/AnalyzerLogicParserTraitsMacro.hpp"



#endif
