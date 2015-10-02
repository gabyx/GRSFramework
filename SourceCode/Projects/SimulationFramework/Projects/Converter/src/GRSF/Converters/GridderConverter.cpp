#include "GRSF/Converters/GridderConverter.hpp"

//#include "GRSF/Systems/SceneParser.hpp"
#include "GRSF/General/GridderLogicParser.hpp"
#include "GRSF/General/GridderLogicParserGenerators.hpp"

#include "GRSF/General/GridExtractor.hpp"

GridderConverter::GridderConverter(const std::vector<boost::filesystem::path> & inputFiles,
              boost::filesystem::path outputFile,
              boost::filesystem::path outputDir,
              boost::filesystem::path sceneFile,
              boost::filesystem::path logicFile)
    : Base(inputFiles,outputFile,outputDir)
{
    m_sceneFile = sceneFile;
    m_logicFile = logicFile;

    setup();
}

void GridderConverter::convert() {

    std::vector<GridExtractor> l;
    std::list<GridExtractor*> ptrL;
    l.reserve(m_gridderData.m_gridSettingsList.size());
    ptrL.reserve(m_gridderData.m_gridSettingsList.size());

    for(auto & sett: m_gridderData.m_gridSettingsList){
        l.emplace_back(sett);
        ptrL.emplace_back(&l.back());
    }


    Base::convert(l);

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
