#ifndef RenderLogicParser_hpp
#define RenderLogicParser_hpp

#include <vector>
#include <fstream>

#include <boost/filesystem.hpp>
#include <pugixml.hpp>

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"
#include "GRSF/Common/AssertionDebug.hpp"
#include "GRSF/Common/TupleHelper.hpp"

#include "GRSF/Common/XMLMacros.hpp"
#include "GRSF/General/LogicParser.hpp"

#include "GRSF/General/RenderLogicParserTraits.hpp"

template< typename TDataStorage, template<typename P, typename C> class TParserTraits = RenderLogicParserTraits >
class RenderLogicParser  : public LogicParser<TDataStorage,
                                       TParserTraits,
                                       RenderLogicParser<TDataStorage,TParserTraits>
                                      >
{
public:
    using Base =  LogicParser<TDataStorage,
                              TParserTraits,
                              RenderLogicParser<TDataStorage,TParserTraits>
                            >;
    using ParserTraits = TParserTraits<RenderLogicParser, TDataStorage>;
    DEFINE_RENDERLOGICPARSER_TYPE_TRAITS(ParserTraits);

public:

    /**
    * Constructor takes a module function which constructs all modules.
    */
    template<typename ModuleGeneratorType>
    RenderLogicParser(ModuleGeneratorType & moduleGen, Logging::Log * log) : Base(moduleGen,log){}

};


#endif

