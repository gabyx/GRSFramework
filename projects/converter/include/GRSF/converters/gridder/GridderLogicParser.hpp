// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_converters_gridder_GridderLogicParser_hpp
#define GRSF_converters_gridder_GridderLogicParser_hpp

#include <fstream>
#include <vector>

#include <boost/filesystem.hpp>
#include <pugixml.hpp>

#include "GRSF/common/Asserts.hpp"
#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/general/LogicParser.hpp"

#include "GRSF/converters/gridder/GridderLogicParserTraits.hpp"

/** We dont need any TDataStorage, LogicModule only needs a execution graph */
template <typename TDataStorage, template <typename P, typename C> class TParserTraits = GridderLogicParserTraits>
class GridderLogicParser
    : public LogicParser<TDataStorage, TParserTraits, GridderLogicParser<TDataStorage, TParserTraits>>
{
    public:
    using Base         = LogicParser<TDataStorage, TParserTraits, GridderLogicParser<TDataStorage, TParserTraits>>;
    using ParserTraits = TParserTraits<GridderLogicParser, TDataStorage>;
    DEFINE_GRIDDERLOGICPARSER_TYPE_TRAITS(ParserTraits);

    public:
    /**
    * Constructor takes a module function which constructs all modules.
    */
    template <typename ModuleGeneratorType>
    GridderLogicParser(ModuleGeneratorType& moduleGen, Logging::Log* log) : Base(moduleGen, log)
    {
    }
};

#endif
