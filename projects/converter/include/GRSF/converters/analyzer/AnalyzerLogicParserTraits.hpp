#ifndef GRSF_converters_analyzer_AnalyzerLogicParserTraits_hpp
#define GRSF_converters_analyzer_AnalyzerLogicParserTraits_hpp


#include "GRSF/general/LogicParserTraits.hpp"


/** The traits for a standart AnalyzerLogicParser class*/
template<typename TSceneParser, typename TDataStorage>
struct AnalyzerLogicParserTraits : LogicParserTraits<TSceneParser,TDataStorage> {


};

#include "GRSF/converters/analyzer/AnalyzerLogicParserTraitsMacro.hpp"



#endif
