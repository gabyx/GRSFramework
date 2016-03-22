#ifndef GRSF_General_GridderLogicParserGenerators_hpp
#define GRSF_General_GridderLogicParserGenerators_hpp


#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"

#include <memory>

// This file should only be included in compilation units!

class GridderData;

namespace GridderLogicParserGenerators {

    struct LogicParserGen {

        LogicParserGen( GridderData * g): m_g(g){}

        GridderData * m_g;

        template<typename TParser>
        typename TParser::ParserTraits::TupleModules
        createParserModules(TParser * p) {

            using ParserTraits = typename TParser::ParserTraits;
            using LogicModuleType     = typename ParserTraits::template getModuleType<0>;

            auto logic = std::unique_ptr<LogicModuleType >  (new LogicModuleType(p, &m_g->m_gridSettingsList));

            return std::make_tuple(std::move(logic));
        };

    };
};

#endif
