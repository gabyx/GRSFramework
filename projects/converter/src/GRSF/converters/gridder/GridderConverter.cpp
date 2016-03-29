// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/converters/gridder/GridderConverter.hpp"

//#include "GRSF/systems/SceneParser.hpp"
#include "GRSF/converters/gridder/GridderLogicParser.hpp"
#include "GRSF/converters/gridder/GridderLogicParserGenerators.hpp"

#include "GRSF/converters/gridder/GridExtractor.hpp"

GridderConverter::GridderConverter(const std::vector<boost::filesystem::path> & inputFiles,
              boost::filesystem::path sceneFile,
              boost::filesystem::path logicFile)
    : Base(inputFiles)
{
    m_sceneFile = sceneFile;
    m_logicFile = logicFile;

    setup();
}

void GridderConverter::convert() {

    /** Make list of all GridExtractors */
    std::vector<GridExtractor> l;

    for(auto & sett: m_gridderData.m_gridSettingsList){
        l.emplace_back(&sett,m_log);
    }

    /** Hand all extractors to the convert function */
    using Settings = Base::ConvertSettings<true>;
    Base::convert<Settings>(l);

}

void GridderConverter::setup() {

    {
        // LOGIC FILE
        LOGRCLEVEL1(m_log, "---> Load Logic file ..." << std::endl;)
        using ParserGen = GridderLogicParserGenerators::LogicParserGen;
        ParserGen c(&m_gridderData);

        using GridderLogicParserType = GridderLogicParser< GridderData /*DataStorage*/ /**, StandartTraits*/ >;
        GridderLogicParserType parser(c,m_log);

        parser.parse(m_logicFile);

    }
}
