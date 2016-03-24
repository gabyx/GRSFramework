#ifndef GRSF_converters_analyzer_AnalyzerLogicParser_hpp
#define GRSF_converters_analyzer_AnalyzerLogicParser_hpp

#include <vector>
#include <fstream>

#include <boost/filesystem.hpp>
#include <pugixml.hpp>

#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/AssertionDebug.hpp"

#include "GRSF/general/LogicParser.hpp"

#include "GRSF/converters/analyzer/AnalyzerLogicParserTraits.hpp"

/** We dont need any TCollection, LogicModule only needs a execution graph */
template< typename TCollection = void, template<typename P, typename C> class TParserTraits = AnalyzerLogicParserTraits >
class AnalyzerLogicParser  : public LogicParser<TCollection,
                                       TParserTraits,
                                       AnalyzerLogicParser<TCollection,TParserTraits>
                                      >
{
public:
    using Base =  LogicParser<TCollection,
                              TParserTraits,
                              AnalyzerLogicParser<TCollection,TParserTraits>
                            >;
    using ParserTraits = TParserTraits<AnalyzerLogicParser, TCollection>;
    DEFINE_ANALZERLOGICPARSER_TYPE_TRAITS(ParserTraits);

public:

    /**
    * Constructor takes a module function which constructs all modules.
    */
    template<typename ModuleGeneratorType>
    AnalyzerLogicParser(ModuleGeneratorType & moduleGen, Logging::Log * log) : Base(moduleGen,log){}

};


#endif

