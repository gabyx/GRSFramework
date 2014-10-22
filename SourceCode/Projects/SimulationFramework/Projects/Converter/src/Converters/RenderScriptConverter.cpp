#include "RenderScriptConverter.hpp"

#include <string>
#include "CPUTimer.hpp"
#include "ProgressBarCL.hpp"

#include "SceneParser.hpp"
#include "RenderScriptParser.hpp"

#include "RenderDataParserGenerators.hpp"

#include "RenderScriptGenerator.hpp"

//#include "DummyNode.hpp"

void RenderScriptConverter::convert( const std::vector<boost::filesystem::path> & inputFiles,
              boost::filesystem::path outputFile,
              boost::filesystem::path sceneFile,
              boost::filesystem::path materialFile,
              Renderer renderer) {
    m_renderer = renderer;
    m_outputFile = outputFile;
    m_inputFiles = inputFiles;
    m_sceneFile = sceneFile;
    m_materialFile = materialFile;
    auto log = outputFile.parent_path() / "RenderScriptConverter.log";
    m_log = Logging::LogManager::getSingletonPtr()->createLog("RenderScriptConverter",true,true,log);

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

    using ParserGen = RenderDataParserGenerators::SceneParserGen;
    ParserGen c(&m_renderData);

    using SceneParserType = SceneParser< RenderData, ParserGen::SceneParserTraits >;
    SceneParserType parser(c,m_log);

    parser.parseScene(m_sceneFile);

    LOGRCLEVEL1(m_log, "---> Loaded: " << m_renderData.m_geometryMap.size() << " geometries, "
                << m_renderData.m_scales.size() << " scales, " << m_renderData.m_visMeshs.size() << " meshs paths" << std::endl;)
    LOGRCLEVEL1(m_log, "---> Load Geometries finished " << std::endl;)
}

void RenderScriptConverter::loadMaterialCollection() {

    LOGRCLEVEL1(m_log, "---> Load Materials ..." << std::endl;)
    using ParserGen = RenderDataParserGenerators::MaterialsParserGen;
    ParserGen c(&m_renderData);


    using RenderScriptParserType = RenderScriptParser<RenderData /**, StandartTraits*/ >;
    RenderScriptParserType parser(c,m_log);

    parser.parse(m_materialFile);
    LOGRCLEVEL1(m_log, "---> Load Materials finished " << std::endl;)

    LOGRCLEVEL1(m_log, "---> Setup Mapper ..." << std::endl;)
    m_renderData.m_renderScriptGen.setup();

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

void RenderScriptConverter::convertFile(const boost::filesystem::path & f) {
    LOG(m_log, "---> Converting file:" << f << std::endl;);

    std::vector<RigidBodyStateAdd> states;

    if(!m_simFile.openRead(f,true)){
        ERRORMSG("Could not open SimFile at :" << f)
    }else{
        LOG(m_log, "---> SimFile Properties:" <<std::endl << m_simFile.getDetails() << std::endl)
    }

    while(m_simFile.isGood()){

        // Write render script for this frame
        double time;
        m_simFile.read(states,time);

        if(states.size()==0){
            ERRORMSG("State size is zero!")
        }

        LOG(m_log, "---> Loaded state at t: " <<time << std::endl;)

        // Produce Render OutputFile for this state
        std::string filename = m_outputFile.filename().string() + std::to_string(m_frameCounter);

        m_renderData.m_renderScriptGen.initFrame(m_outputFile.parent_path(), filename, time, m_frameCounter );

        for(auto & bs: states){

            m_renderData.m_renderScriptGen.generateFrameData(&bs);

        }

        m_frameCounter++;

    }


}
