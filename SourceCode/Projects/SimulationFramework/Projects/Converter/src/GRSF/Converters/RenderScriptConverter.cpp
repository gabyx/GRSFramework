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

#include "GRSF/General/RenderScriptGenerator.hpp"

//#include "GRSF/Logic/DummyNode.hpp"

void RenderScriptConverter::convert( const std::vector<boost::filesystem::path> & inputFiles,
              boost::filesystem::path outputFile,
              boost::filesystem::path outputDir,
              boost::filesystem::path sceneFile,
              boost::filesystem::path logicFile,
              Renderer renderer) {
    m_renderer = renderer;
    m_outputFile = outputFile;
    m_outputDir  = m_outputDir;
    m_inputFiles = inputFiles;
    m_sceneFile = sceneFile;
    m_logicFile = logicFile;
    auto log = outputFile.parent_path() / "RenderScriptConverter.log";
    m_log = Logging::LogManager::getSingleton().createLog("RenderScriptConverter",true,true,log);

    LOG(m_log, "---> RenderScriptConverter started:" <<std::endl;);

    // global framecounter
    m_frameCounter = 0;

    loadGeometryCollection();
    loadMaterialCollection();

    // First open the sim file (if .sim extension)
    // if .xml extension (then this is the process file where each simfile and frame index is stored)

    unsigned int fileIdx = 0;
    for(auto file : m_inputFiles) {


        if(file.extension() == ".xml"){

            // open the xml
            pugi::xml_document xmlDoc;
            xmlDoc.load_file(file.string().c_str());
            auto node = xmlDoc.child("Converter");
            if(!node){
                ERRORMSG("XML file: " << file << " contains no 'Converter' node!" )
            }

            StateIndicesType stateIndices;
            for(auto n : node.children("File")){
                stateIndices.clear();
                std::string uuid = n.attribute("uuid").value();
                boost::filesystem::path path = n.attribute("simFile").value();

                if(path.empty()){
                    LOG(m_log,"---> No simFile path given, skip this file!" << std::endl;)
                    continue;
                }

                // parse frame index list (assumed to be sorted! otherwise exception in convertFile)
                StateIdxType idx;
                unsigned int frameIdx;
                boost::filesystem::path outputFile;
                for(auto s : n.children("State")){

                    if( !Utilities::stringToType(idx, s.attribute("stateIdx").value() )  ) {
                            ERRORMSG("---> String conversion to obtain state id failed!");
                    }

                    if( !Utilities::stringToType(frameIdx, s.attribute("frameIdx").value() )  ) {
                            ERRORMSG("---> String conversion to obtain state id failed!");
                    }

                    outputFile = "";
                    auto att = s.attribute("outputFile");
                    if( att ) {
                       outputFile = att.value();
                    }

                    // add to list
                    stateIndices.push_back( StateIndex{idx,frameIdx,outputFile});
                }
                LOG(m_log,"---> Parsed " << stateIndices.size() << " state for file: " << path << from XML: " << file.filename() << std::endl;)
                if( stateIndices.size() > 0){
                    convertFile(path,uuid,std::move(stateIndices));
                }else{
                    LOG(m_log,"---> No states to process..." << std::endl;)
                }

            }

        }else{
            // try to convert sim file
            convertFile(file, std::to_string(fileIdx) );
        }

        ++fileIdx;
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
                                        const std::string uuidString,
                                        StateIndicesType stateIndices
                                        ) {
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


    StateIdxType currentStateIdx = 0;
    auto itStateIdx = stateIndices.begin();
    // Jump at beginning of first state
    if( !stateIndices.empty() && itStateIdx->m_idx > 0 ){
        currentStateIdx = itStateIdx->m_idx;
        m_simFile.seekgStates(currentStateIdx);
    }



    while(m_simFile.isGood() && !m_abort){

        std::string outputName;
        boost::filesystem::path outputDir = "./";

        // produce render output file for this state

        // load state
        double time;
        start = timer.elapsedMilliSec();
        m_simFile.read(states,time);
        avgStateLoadTime +=  timer.elapsedMilliSec() - start;

        if(states.size()==0){
            ERRORMSG("State size is zero!")
        }
        LOG(m_log, "---> Loaded state at t: " <<time << std::endl;)

        // set frame name and output dir
        // if function argument is given take this
        if(!m_outputDir.empty()){
            outputDir  = m_outputDir;
        }
        // set output file name for this state,
        if( m_outputFile.empty() && !stateIndices.empty() && !itStateIdx->m_outputFile.empty() ){
            // we have a file path given and no function argument, take from state
            if( !itStateIdx->m_outputFile.has_filename()){
                ERRORMSG(" State idx: " << itStateIdx->m_idx << " has no correct filename!")
            }
            outputDir  /= itStateIdx->m_outputFile.parent_path();
            outputName = itStateIdx->m_outputFile.filename().string();
        }else{
            // format default file name otherwise (if there is no file path in xml or function argument is given)
            std::string baseFilename =  m_outputFile.filename().string();
            if( baseFilename.empty()){
                baseFilename = "Frame";
            }
            outputDir  /= m_outputFile.parent_path();
            outputName = baseFilename +"-id-"+uuidString + "-s-" + std::to_string(m_frameCounter);
        }

        // set frame idx (if we have a list set it from the list, other wise default)
        unsigned int frameIdx = m_frameCounter;
        if( !stateIndices.empty()){
            frameIdx = itStateIdx->m_frameIdx;

        }

        LOG(m_log, "---> Init frame with: \n\toutputDir: " << outputDir
            << "\n\tframeName: " << outputName << "\n\tframeIdx: " << frameIdx << "\n\ttime: " << time << std::endl;)

        start = timer.elapsedMilliSec();
        m_renderScriptGen.initFrame(outputDir, outputName , time, frameIdx );
        avgInitFrameTime += timer.elapsedMilliSec() - start;


        start = timer.elapsedMilliSec();
        for(auto & bs: states){
            m_renderScriptGen.generateFrameData(&bs);
            bodyCounter++;
        }
        avgStateTime += timer.elapsedMilliSec() - start;


        m_renderScriptGen.finalizeFrame();

        // skip to next stateIdx if we have indices
        if(!stateIndices.empty()){
            if(++itStateIdx != stateIndices.end()){
                if(itStateIdx->m_idx < 0 || itStateIdx->m_idx == currentStateIdx){
                    ERRORMSG("Negative or same as privious state idx: " << itStateIdx->m_idx << " in xml for file: " << f)
                }
                // skip difference
                m_simFile.seekgStates(itStateIdx->m_idx - currentStateIdx -1);
                currentStateIdx = itStateIdx->m_idx;
            }else{
                m_abort = true;
            }
        }else{
            //otherwise dont skip, but update stateIdx
            ++currentStateIdx;
        }

        m_frameCounter++;
    }

    if(!stateIndices.empty()){
        if(itStateIdx != stateIndices.end()){
            LOG(m_log, "---> Warning: Reading simfile: "
                << f << " became invalid before all state inidices have been converted" << std::endl)
        }
    }

      LOG(m_log, "---> Converter Speed:" <<std::endl
        << "Avg. Load State  / Frame: "   << (avgStateLoadTime / m_frameCounter) << " ms" <<std::endl
        << "Avg. Init Frame  / Frame: "   << (avgInitFrameTime / m_frameCounter) << " ms" <<std::endl
        << "Avg. State  / Body: " << (avgStateTime / (m_frameCounter * bodyCounter)) << " ms" <<std::endl
        << "Avg. State : " << (avgStateTime / m_frameCounter) << " ms" <<std::endl;)

    ApplicationSignalHandler::getSingleton().unregisterCallback(SIGINT,"QuitRender");

}
