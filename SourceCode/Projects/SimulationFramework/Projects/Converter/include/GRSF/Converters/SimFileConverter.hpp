#ifndef LogicConverter_hpp
#define LogicConverter_hpp

#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <string>
#include <vector>

#include <pugixml.hpp>

#include <boost/filesystem.hpp>

#include "GRSF/Common/LogDefines.hpp"
#include "GRSF/Common/TypeDefs.hpp"

#include "GRSF/Common/SimpleLogger.hpp"
#include "GRSF/Dynamics/General/MultiBodySimFile.hpp"

#include "GRSF/Common/ApplicationSignalHandler.hpp"

#include "GRSF/Common/CPUTimer.hpp"
#include "GRSF/Common/CommonFunctions.hpp"
#include "GRSF/Common/ProgressBarCL.hpp"

//#include "GRSF/Logic/DummyNode.hpp"


class SimFileConverter {
public:

    using XMLNodeType = pugi::xml_node;
    using XMLNodeItType = pugi::xml_node_iterator;
    using XMLAttributeType = pugi::xml_attribute;

    SimFileConverter(const std::vector<boost::filesystem::path> & inputFiles,
                  boost::filesystem::path outputFile,
                  boost::filesystem::path outputDir){

        m_outputFile = outputFile;
        m_outputDir  = m_outputDir;
        m_inputFiles = inputFiles;

        auto log = outputFile.parent_path() / "LogicConverter.log";
        m_log = Logging::LogManager::getSingleton().createLog("LogicConverter",true,true,log);

    }

    template<typename TSimFileStepper>
    void convert( TSimFileStepper & simFileStepper)
    {

        LOG(m_log, "---> LogicConverter started:" <<std::endl;);

        // global framecounter
        m_frameCounter = 0;

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
                    LOG(m_log,"---> Parsed " << stateIndices.size() << " state for file: " << path << "from XML: " << file.filename() << std::endl;)
                    if( stateIndices.size() > 0){
                        convertFile(simFileStepper,path,uuid,std::move(stateIndices));
                    }else{
                        LOG(m_log,"---> No states to process..." << std::endl;)
                    }

                }

            }else{
                // try to convert sim file
                convertFile(simFileStepper, file, std::to_string(fileIdx) );
            }

            ++fileIdx;
        }
    }

protected:

    using StateIdxType = std::streamoff;
    struct StateIndex{
        StateIdxType m_idx;
        unsigned int m_frameIdx;
        boost::filesystem::path m_outputFile;
    };

    using StateIndicesType = std::vector< StateIndex >;

    MultiBodySimFile m_simFile;

    Logging::Log * m_log;

    boost::filesystem::path m_outputFile;
    boost::filesystem::path m_outputDir;

    std::vector<boost::filesystem::path> m_inputFiles;

    unsigned int m_frameCounter;
    bool m_terminatedByStepper;

    bool m_abort;
    void callbackAbort(){ m_abort = true; LOG(m_log, "---> Quitting ...:" <<std::endl);}

    virtual void setupStepper(){
        ERRORMSG("NOTHING IMPLEMENTED")
    }

    /** \p uuid string is a hash for the file path to identify each frame where it came from!*/
    template<typename TSimFileStepper>
    void convertFile(TSimFileStepper & simFileStepper,
                     const boost::filesystem::path & f,
                     const std::string uuidString ,
                     StateIndicesType stateIndices = {} )
    {
        LOG(m_log, "---> Converting file:" << f << std::endl;);

        m_abort = false;
        ApplicationSignalHandler::getSingleton().registerCallback(SIGINT,
                            std::bind( &SimFileConverter::callbackAbort, this), "SimFileConverter");

        std::vector<RigidBodyStateAdd> states;

        if(!m_simFile.openRead(f,true)){
            ERRORMSG("Could not open SimFile at :" << f)
        }else{
            LOG(m_log, "---> SimFile Properties:" <<std::endl << m_simFile.getDetails().getString() << std::endl)
        }

        // If we have a sim file info node, set the output
        simFileStepper.initSimInfo(m_simFile.getNSimBodies(),m_simFile.getNStates());


        CPUTimer timer;
        timer.start();

        double start = 0, avgInitFrameTime = 0, avgStateTime = 0, avgStateLoadTime = 0;
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
            //TODO get here a body state iterator instead of reading all states

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
            simFileStepper.initFrame(outputDir, outputName , time, frameIdx );
            avgInitFrameTime += timer.elapsedMilliSec() - start;


            start = timer.elapsedMilliSec();
            for(auto & bs: states){
                simFileStepper.addBodyState(&bs);

                if(simFileStepper.isStopBodyLoop()){
                    break;
                    m_terminatedByStepper=true;
                }

                bodyCounter++;
            }
            avgStateTime += timer.elapsedMilliSec() - start;


            simFileStepper.finalizeFrame();

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

            if(simFileStepper.isStopFrameLoop()){
                m_terminatedByStepper=true;
                break;
            }
        }

        if(!stateIndices.empty() && m_terminatedByStepper==false){
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

        ApplicationSignalHandler::getSingleton().unregisterCallback(SIGINT,"SimFileConverter");

    }

};

#endif // LogicConverter_hpp


