#include "GRSF/Converters/GridderConverter.hpp"

//#include "GRSF/Systems/SceneParser.hpp"
#include "GRSF/General/GridderLogicParser.hpp"
#include "GRSF/General/GridderLogicParserGenerators.hpp"

#include "GRSF/General/GridExtractor.hpp"

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
