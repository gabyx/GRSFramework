// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_converters_analyzer_AnalyzerLogicParser_hpp
#define GRSF_converters_analyzer_AnalyzerLogicParser_hpp

#include <vector>
#include <fstream>

#include <boost/filesystem.hpp>
#include <pugixml.hpp>

#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/Asserts.hpp"

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

