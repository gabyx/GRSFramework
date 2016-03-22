#ifndef GRSF_General_GridderLogicParser_hpp
#define GRSF_General_GridderLogicParser_hpp

#include <vector>
#include <fstream>

#include <boost/filesystem.hpp>
#include <pugixml.hpp>

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"
#include "GRSF/Common/AssertionDebug.hpp"

#include "GRSF/General/LogicParser.hpp"

#include "GRSF/Converters/Gridder/GridderLogicParserTraits.hpp"

/** We dont need any TDataStorage, LogicModule only needs a execution graph */
template< typename TDataStorage, template<typename P, typename C> class TParserTraits = GridderLogicParserTraits >
class GridderLogicParser  : public LogicParser<TDataStorage,
                                       TParserTraits,
                                       GridderLogicParser<TDataStorage,TParserTraits>
                                      >
{
public:
    using Base =  LogicParser<TDataStorage,
                              TParserTraits,
                              GridderLogicParser<TDataStorage,TParserTraits>
                            >;
    using ParserTraits = TParserTraits<GridderLogicParser, TDataStorage>;
    DEFINE_GRIDDERLOGICPARSER_TYPE_TRAITS(ParserTraits);

public:

    /**
    * Constructor takes a module function which constructs all modules.
    */
    template<typename ModuleGeneratorType>
    GridderLogicParser(ModuleGeneratorType & moduleGen, Logging::Log * log) : Base(moduleGen,log){}

};


#endif

