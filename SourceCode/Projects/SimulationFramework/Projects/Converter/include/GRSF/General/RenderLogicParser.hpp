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

template< typename TCollection, template<typename P, typename C> class TParserTraits = RenderLogicParserTraits >
class RenderLogicParser  : public LogicParser<TCollection,
                                       TParserTraits,
                                       RenderLogicParser<TCollection,TParserTraits>
                                      >
{
public:
    using Base =  LogicParser<TCollection,
                              TParserTraits,
                              RenderLogicParser<TCollection,TParserTraits>
                            >;
    using ParserTraits = TParserTraits<RenderLogicParser, TCollection>;
    DEFINE_RENDERLOGICPARSER_TYPE_TRAITS(ParserTraits);

public:

    /**
    * Constructor takes a module function which constructs all modules.
    */
    template<typename ModuleGeneratorType>
    RenderLogicParser(ModuleGeneratorType & moduleGen, Logging::Log * log) : Base(moduleGen,log){}

};


#endif

