#ifndef GRSF_General_AnalyzerLogicParserGenerators_hpp
#define GRSF_General_AnalyzerLogicParserGenerators_hpp


#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"

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
