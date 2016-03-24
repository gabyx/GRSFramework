#include "GRSF/converters/analyzer/AnalyzerConverter.hpp"

//#include "GRSF/systems/SceneParser.hpp"
#include "GRSF/converters/analyzer/AnalyzerLogicParser.hpp"
#include "GRSF/converters/analyzer/AnalyzerLogicParserGenerators.hpp"

AnalyzerConverter::AnalyzerConverter(const std::vector<boost::filesystem::path> & inputFiles,
              boost::filesystem::path sceneFile,
              boost::filesystem::path logicFile)
    : Base(inputFiles)
{
    m_sceneFile = sceneFile;
    m_logicFile = logicFile;

    setupExecutionGraph();
}

void AnalyzerConverter::convert() {
    Base::convert(m_executionGraph);
}

void AnalyzerConverter::setupExecutionGraph() {

    {
        // LOGIC FILE
        LOGRCLEVEL1(m_log, "---> Load Logic file ..." << std::endl;)
        using ParserGen = AnaylzerLogicParserGenerators::LogicParserGen;
        ParserGen c(&m_executionGraph);

        using AnalyzerLogicParserType = AnalyzerLogicParser</**, StandartTraits*/ >;
        AnalyzerLogicParserType parser(c,m_log);

        parser.parse(m_logicFile);

        LOGRCLEVEL1(m_log, "---> Setup Execution Graph ..." << std::endl;)
        m_executionGraph.setLog(m_log);
        m_executionGraph.setup();
    }
}
