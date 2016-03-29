// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_converters_analyzer_AnalyzerLogicParserGenerators_hpp
#define GRSF_converters_analyzer_AnalyzerLogicParserGenerators_hpp


#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/LogDefines.hpp"

#include <memory>

// This file should only be included in compilation units!

class SimFileExecutionGraph;

namespace AnaylzerLogicParserGenerators {

    struct LogicParserGen {

        LogicParserGen( SimFileExecutionGraph * g): m_g(g){}

        SimFileExecutionGraph * m_g;

        template<typename TParser>
        typename TParser::ParserTraits::TupleModules
        createParserModules(TParser * p) {

            using ParserTraits = typename TParser::ParserTraits;
            using LogicModuleType     = typename ParserTraits::template getModuleType<0>;

            auto logic = std::unique_ptr<LogicModuleType >    (new LogicModuleType(p, m_g));

            return std::make_tuple(std::move(logic));
        };

    };
};

#endif
