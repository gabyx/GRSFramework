#include "GRSF/Converters/RenderScriptConverter.hpp"

#include <string>



#include "GRSF/Common/ApplicationSignalHandler.hpp"

#include "GRSF/Common/ApplicationCLOptionsConverter.hpp"

#include "GRSF/Common/CPUTimer.hpp"
#include "GRSF/Common/CommonFunctions.hpp"
#include "GRSF/Common/ProgressBarCL.hpp"

#include "GRSF/Systems/SceneParser.hpp"
#include "GRSF/General/RenderScriptParser.hpp"

#include "GRSF/General/RenderScriptParserGenerators.hpp"

#include "GRSF/General/RenderExecutionGraph.hpp"

//#include "GRSF/Logic/DummyNode.hpp"

void RenderScriptConverter::convert( const std::vector<boost::filesystem::path> & inputFiles,
              boost::filesystem::path outputFile,
              boost::filesystem::path outputDir,
              boost::filesystem::path sceneFile,
              boost::filesystem::path logicFile,
              Renderer renderer) {
    m_renderer = renderer;
    Base::convert(inputFiles,outputFile,outputDir,sceneFile,logicFile)
}

void RenderScriptConverter::setupGenerator() {

    // SCENE FILE
    LOGRCLEVEL1(m_log, "---> Load Geometries ..." << std::endl;)

    using ParserGen = RenderScriptParserGenerators::SceneParserGen;
    ParserGen c(&m_renderData);

    using SceneParserType = SceneParser< RenderData, ParserGen::SceneParserTraits >;
    SceneParserType parser(c,m_log, ApplicationCLOptionsRenderer::getSingleton().getMediaDir() );
    parser.parseScene(m_sceneFile);
    LOGRCLEVEL1(m_log, "---> Loaded: " << m_renderData.m_geometryMap.size() << " geometries, "
                << m_renderData.m_scales.size() << " scales, " << m_renderData.m_visMeshs.size() << " meshs paths" << std::endl;)
    LOGRCLEVEL1(m_log, "---> Load Geometries finished " << std::endl;)

    // LOGIC FILE
    LOGRCLEVEL1(m_log, "---> Load Logic file ..." << std::endl;)
    using ParserGen = RenderScriptParserGenerators::ScriptParserGen;
    ParserGen c(&m_renderData,&m_renderScriptGen);


    using RenderScriptParserType = RenderScriptParser<RenderData /**, StandartTraits*/ >;
    RenderScriptParserType parser(c,m_log);

    parser.parse(m_logicFile);
    LOGRCLEVEL1(m_log, "---> Load Materials finished " << std::endl;)

    LOGRCLEVEL1(m_log, "---> Setup Mapper ..." << std::endl;)
    m_logicScriptGen.setLog(m_log);
    m_logicScriptGen.setup();

    //    ExecutionTreeInOut m;
    //    LogicNode * n0 = new DummyLogicNode<1,3>(0);
    //    LogicNode * n1 = new DummyLogicNode<1,3>(1);
    //    LogicNode * n2 = new DummyLogicNode<2,3>(2);
    //    LogicNode * n3 = new DummyLogicNode<1,1>(3);
    //    LogicNode * n4 = new DummyLogicNode<2,1>(4);
    //    LogicNode * n5 = new DummyLogicNode<1,2>(5);
    //    LogicNode * n6 = new DummyLogicNode<4,1>(6);
    //
    //    //link
    //    LogicNode::linkTogether(n0,0,n1,0);
    //
    //    LogicNode::linkTogether(n1,0,n2,0);
    //    LogicNode::linkTogether(n1,1,n3,0);
    //    LogicNode::linkTogether(n1,2,n6,3);
    //
    //    LogicNode::linkTogether(n2,0,n5,0);
    //    LogicNode::linkTogether(n2,1,n6,1);
    //    LogicNode::linkTogether(n2,2,n4,0);
    //
    //
    //    LogicNode::linkTogether(n3,0,n4,1);
    //    LogicNode::linkTogether(n4,0,n6,2);
    //    LogicNode::linkTogether(n5,0,n6,0);
    //
    //    // cycle
    //    //LogicNode::linkTogether(n5,1,n2,1);
    //    LOGRCLEVEL1(m_log, "---> Setup" << std::endl;)
    //    m.addNode(n0);
    //    m.setInputNode(0);
    //    m.addNode(n1);
    //    m.addNode(n2);
    //    m.addNode(n3);
    //    m.addNode(n4);
    //    m.addNode(n5);
    //    m.addNode(n6);
    //
    //    m.setOutputNode(6);
    //    LOGRCLEVEL1(m_log, "---> Setup" << std::endl;)
    //    m.setup();
    //    LOGRCLEVEL1(m_log, "---> Execute" << std::endl;)
    //    m.execute();
    //std::cout << n6->getOSocketValue<double>(0) << std::endl;
}
