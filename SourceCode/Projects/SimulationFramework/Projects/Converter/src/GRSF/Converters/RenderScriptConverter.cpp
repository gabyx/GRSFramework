#include "GRSF/Converters/RenderScriptConverter.hpp"

#include <string>

#include "GRSF/Common/ApplicationSignalHandler.hpp"

#include "GRSF/Common/ApplicationCLOptionsConverter.hpp"

#include "GRSF/Common/CPUTimer.hpp"
#include "GRSF/Common/ProgressBarCL.hpp"

#include "GRSF/Systems/SceneParser.hpp"
#include "GRSF/General/RenderScriptParser.hpp"

#include "GRSF/General/RenderScriptParserGenerators.hpp"

#include "GRSF/General/RenderScriptGenerator.hpp"

//#include "GRSF/Logic/DummyNode.hpp"

void RenderScriptConverter::convert( const std::vector<boost::filesystem::path> & inputFiles,
              boost::filesystem::path outputFile,
              boost::filesystem::path sceneFile,
              boost::filesystem::path logicFile,
              Renderer renderer) {
    m_renderer = renderer;
    m_outputFile = outputFile;
    m_inputFiles = inputFiles;
    m_sceneFile = sceneFile;
    m_logicFile = logicFile;
    auto log = outputFile.parent_path() / "RenderScriptConverter.log";
    m_log = Logging::LogManager::getSingleton().createLog("RenderScriptConverter",true,true,log);

    LOG(m_log, "---> RenderScriptConverter started:" <<std::endl;);

    m_frameCounter = 0;

    loadGeometryCollection();
    loadMaterialCollection();

    // First open the sim file
    for(auto file : m_inputFiles) {
        convertFile(file);
    }

}

void RenderScriptConverter::loadGeometryCollection() {

    LOGRCLEVEL1(m_log, "---> Load Geometries ..." << std::endl;)

    using ParserGen = RenderScriptParserGenerators::SceneParserGen;
    ParserGen c(&m_renderData);

    using SceneParserType = SceneParser< RenderData, ParserGen::SceneParserTraits >;
    SceneParserType parser(c,m_log, ApplicationCLOptionsRenderer::getSingleton().getMediaDir() );

    parser.parseScene(m_sceneFile);

    LOGRCLEVEL1(m_log, "---> Loaded: " << m_renderData.m_geometryMap.size() << " geometries, "
                << m_renderData.m_scales.size() << " scales, " << m_renderData.m_visMeshs.size() << " meshs paths" << std::endl;)
    LOGRCLEVEL1(m_log, "---> Load Geometries finished " << std::endl;)
}

void RenderScriptConverter::loadMaterialCollection() {

    LOGRCLEVEL1(m_log, "---> Load Materials ..." << std::endl;)
    using ParserGen = RenderScriptParserGenerators::ScriptParserGen;
    ParserGen c(&m_renderData,&m_renderScriptGen);


    using RenderScriptParserType = RenderScriptParser<RenderData /**, StandartTraits*/ >;
    RenderScriptParserType parser(c,m_log);

    parser.parse(m_logicFile);
    LOGRCLEVEL1(m_log, "---> Load Materials finished " << std::endl;)

    LOGRCLEVEL1(m_log, "---> Setup Mapper ..." << std::endl;)
    m_renderScriptGen.setLog(m_log);
    m_renderScriptGen.setup();

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

void RenderScriptConverter::convertFile(const boost::filesystem::path & f,
                                        const std::string uuidString) {
    LOG(m_log, "---> Converting file:" << f << std::endl;);

    m_abort = false;
    ApplicationSignalHandler::getSingleton().registerCallback(SIGINT,
                        std::bind( &RenderScriptConverter::callbackAbort, this), "QuitRender");

    std::vector<RigidBodyStateAdd> states;

    if(!m_simFile.openRead(f,true)){
        ERRORMSG("Could not open SimFile at :" << f)
    }else{
        LOG(m_log, "---> SimFile Properties:" <<std::endl << m_simFile.getDetails().getString() << std::endl)
    }

    CPUTimer timer;
    timer.start();

    PREC start = 0,avgInitFrameTime = 0, avgStateTime = 0, avgStateLoadTime = 0;
    unsigned int bodyCounter = 0;

    std::string fileName =  m_outputFile.filename().string();
    if(fileName.empty()){
        fileName = "Frame";
    }
    if(!uuidString.empty()){
        fileName += "-id-"+uuidString;
    }

    while(m_simFile.isGood() && !m_abort){

        // Write render script for this frame
        double time;
        start = timer.elapsedMilliSec();
        m_simFile.read(states,time);

        if(states.size()==0){
            ERRORMSG("State size is zero!")
        }
        LOG(m_log, "---> Loaded state at t: " <<time << std::endl;)
        avgStateLoadTime +=  timer.elapsedMilliSec() - start;

        // Produce Render OutputFile for this state
        start = timer.elapsedMilliSec();
        m_renderScriptGen.initFrame(m_outputFile.parent_path(), fileName + tinyformat::format("-f-%06i",m_frameCounter) , time, m_frameCounter );
        avgInitFrameTime += timer.elapsedMilliSec() - start;


        start = timer.elapsedMilliSec();
        for(auto & bs: states){
            m_renderScriptGen.generateFrameData(&bs);
            bodyCounter++;
        }
        avgStateTime += timer.elapsedMilliSec() - start;


        m_renderScriptGen.finalizeFrame();
        m_frameCounter++;

    }

      LOG(m_log, "---> Converter Speed:" <<std::endl
        << "Avg. Load State Time / Frame: "   << (avgStateLoadTime / m_frameCounter) << " ms" <<std::endl
        << "Avg. Init Frame Time / Frame: "   << (avgInitFrameTime / m_frameCounter) << " ms" <<std::endl
        << "Avg. State Time / Body: " << (avgStateTime / (m_frameCounter * bodyCounter)) << " ms" <<std::endl;)

    ApplicationSignalHandler::getSingleton().unregisterCallback(SIGINT,"QuitRender");

}
