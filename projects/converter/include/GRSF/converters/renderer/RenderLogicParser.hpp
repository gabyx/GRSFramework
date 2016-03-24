#ifndef GRSF_converters_renderer_RenderLogicParser_hpp
#define GRSF_converters_renderer_RenderLogicParser_hpp

#include <vector>
#include <fstream>

#include <boost/filesystem.hpp>
#include <pugixml.hpp>

#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/AssertionDebug.hpp"
#include "GRSF/common/TupleHelper.hpp"

#include "GRSF/common/XMLMacros.hpp"
#include "GRSF/general/LogicParser.hpp"

#include "GRSF/converters/renderer/RenderLogicParserTraits.hpp"

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

