#ifndef RenderConverter_hpp
#define RenderConverter_hpp

#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <vector>

#include <boost/filesystem.hpp>

#include "LogDefines.hpp"
#include "TypeDefs.hpp"

#include "SimpleLogger.hpp"
#include "CPUTimer.hpp"
#include "ProgressBarCL.hpp"
#include "MultiBodySimFile.hpp"

#include "SceneParser.hpp"

#include DynamicsSystem_INCLUDE_FILE

#include "RenderMaterialMapper.hpp"

class RenderConverter {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    using Renderer = typename ApplicationCLOptionsRenderer::Renderer;

    void convert( const std::vector<boost::filesystem::path> & inputFiles,
                  boost::filesystem::path outputFile,
                  boost::filesystem::path sceneFile,
                  boost::filesystem::path materialFile,
                  Renderer renderer) {
        m_renderer = renderer;
        m_outputFile = outputFile;
        m_inputFiles = inputFiles;
        m_sceneFile = sceneFile;
        m_materialFile = materialFile;
        auto log = outputFile.parent_path() / "RenderConverter.log";
        m_log = Logging::LogManager::getSingletonPtr()->createLog("RenderConverter",true,true,log);

        LOG(m_log, "---> RenderConverter started:" <<std::endl;);


        loadGeometryCollection();
        loadMaterialCollection();

        // First open the sim file
        for(auto file : m_inputFiles) {
            convertFile(file);
        }

    }

private:

    DynamicsSystemType m_dynSys;
    RenderMaterialMapper m_materialMapper;


    void loadGeometryCollection() {

        LOGRCLEVEL1(m_log, "---> Load Geometries ..." << std::endl;)
        DynamicsSystemType::ParserModulesCreator c(&m_dynSys);

        using SceneParserType = SceneParser< DynamicsSystemType, DynamicsSystemType::ParserModulesCreator::SceneParserTraits >;
        SceneParserType parser(c,m_log);

        parser.parseScene(m_sceneFile);

        LOGRCLEVEL1(m_log, "---> Loaded: " << m_dynSys.m_geometryMap.size() << " geometries, "
                    << m_dynSys.m_scales.size() << " scales, " << m_dynSys.m_visMeshs.size() << " meshs paths" << std::endl;)
        LOGRCLEVEL1(m_log, "---> Load Geometries finished " << std::endl;)
    }

    void loadMaterialCollection() {

        LOGRCLEVEL1(m_log, "---> Load Materials ..." << std::endl;)
        DynamicsSystemType::MatCollParserModulesCreator c(&m_dynSys);

        using MatCollParserType = MaterialsCollectionParser< DynamicsSystemType>;
        MatCollParserType parser(c,m_log);

        parser.parse(m_materialFile);
        LOGRCLEVEL1(m_log, "---> Load Materials finished " << std::endl;)

        LOGRCLEVEL1(m_log, "---> Setup Mapper ..." << std::endl;)

        ExecutionTreeInOut<DummyLogicNode<1,1> > m;
        LogicNode * n1 = new DummyLogicNode<1,3>(1);
        LogicNode * n2 = new DummyLogicNode<2,3>(2);
        LogicNode * n3 = new DummyLogicNode<1,1>(3);
        LogicNode * n4 = new DummyLogicNode<2,1>(4);
        LogicNode * n5 = new DummyLogicNode<1,2>(5);
        LogicNode * n6 = new DummyLogicNode<4,1>(6);

        //link
        LogicNode::linkTogether(m.getInputNode(),0,n1,0);

        LogicNode::linkTogether(n1,0,n2,0);
        LogicNode::linkTogether(n1,1,n3,0);
        LogicNode::linkTogether(n1,2,n6,3);

        LogicNode::linkTogether(n2,0,n5,0);
        LogicNode::linkTogether(n2,1,n6,1);
        LogicNode::linkTogether(n2,2,n4,0);


        LogicNode::linkTogether(n3,0,n4,1);
        LogicNode::linkTogether(n4,0,n6,2);
        LogicNode::linkTogether(n5,0,n6,0);

        // cycle
        //LogicNode::linkTogether(n5,1,n2,1);

        m.addNode(n1);
        m.addNode(n2);
        m.addNode(n3);
        m.addNode(n4);
        m.addNode(n5);
        m.addNode(n6);

        m.setOutputNode(6);
        m.setup();

        m.execute();

        std::cout << n6->getSocketValue<double>(4) << std::endl;
    }

    void convertFile(const boost::filesystem::path & f) {
        LOG(m_log, "---> Converting file:" << f << std::endl;);

    }

    MultiBodySimFile m_simFile;

    Logging::Log * m_log;

    boost::filesystem::path m_materialFile;
    boost::filesystem::path m_sceneFile;

    boost::filesystem::path m_outputFile;
    std::vector<boost::filesystem::path> m_inputFiles;
    Renderer m_renderer;
};

#endif // RenderConverter_hpp

