#ifndef GRSF_General_AnalyzerLogicParserTraits_hpp
#define GRSF_General_AnalyzerLogicParserTraits_hpp


#include "GRSF/General/LogicParserTraits.hpp"


/** The traits for a standart AnalyzerLogicParser class*/
template<typename TSceneParser, typename TDataStorage>
struct AnalyzerLogicParserTraits : LogicParserTraits<TSceneParser,TDataStorage> {


};

#include "GRSF/Converters/Analyzer/AnalyzerLogicParserTraitsMacro.hpp"



#endif
