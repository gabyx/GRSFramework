// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/states/simulationManager/SimulationManagerMPI.hpp"

#include <fstream>

#define _USE_MATH_DEFINES
#include <cmath>

#include <memory>

#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/Asserts.hpp"

#include "GRSF/common/ApplicationSignalHandler.hpp"

#include "GRSF/systems/SharedBufferDynSys.hpp"
#include "GRSF/dynamics/buffers/StateRecorder.hpp"
#include "GRSF/dynamics/buffers/StateRecorderBody.hpp"
#include "GRSF/dynamics/buffers/StateRecorderProcess.hpp"
#include "GRSF/dynamics/buffers/StateRecorderMPI.hpp"

#include "GRSF/systems/SceneParserMPI.hpp"

#include TimeStepper_INCLUDE_FILE
#include DynamicsSystem_INCLUDE_FILE

#include "GRSF/common/CommonFunctions.hpp"
#include "GRSF/dynamics/general/QuaternionHelpers.hpp"

#include "GRSF/singeltons/FileManager.hpp"

#include "GRSF/dynamics/general/MPIMessages.hpp"
#include "GRSF/dynamics/general/MPICommunication.hpp"

#include "GRSF/dynamics/general/MPITopologyBuilder.hpp"


SimulationManagerMPI::SimulationManagerMPI() {

}

SimulationManagerMPI::~SimulationManagerMPI() {

    ApplicationSignalHandler::getSingleton().unregisterCallback("SimulationManagerMPI::gracefullyExit");
    DESTRUCTOR_MESSAGE

}


void SimulationManagerMPI::setup() {
    setup("SceneFile.xml");
}


void SimulationManagerMPI::setup(boost::filesystem::path sceneFilePath) {

    //Each Process copies its SceneFile.xml to the folders, that each Process can read its own file (better)
    m_sceneFileParsed = FileManager::getSingleton().copyFile(sceneFilePath,
            FileManager::getSingleton().getLocalDirectoryPath());

    //Make log!
    m_pSimulationLog = nullptr;
    if(Logging::LogManager::getSingleton().existsLog("SimulationLog")) {
        m_pSimulationLog = Logging::LogManager::getSingleton().getLog("SimulationLog");
    } else {
        boost::filesystem::path filePath = FileManager::getSingleton().getLocalDirectoryPath();
        filePath /= GLOBAL_LOG_FOLDER_DIRECTORY;
        filePath /= "SimulationLog.log";
        m_pSimulationLog = Logging::LogManager::getSingleton().createLog("SimulationLog",SIMULATION_LOG_TO_CONSOLE,true,filePath);
    }

    m_pSimulationLog->logMessage("---> SimulationManagerMPI::setup(): ");

    // MPI Process Commnunicator

    m_pProcCommunicator = std::shared_ptr<ProcessCommunicatorType >(new ProcessCommunicatorType(
                                                    MPILayer::MPIGlobalCommunicators::getSingleton().getCommunicator(
                                                      MPILayer::MPICommunicatorId::SIM_COMM
                                                    )
                                           ));
    m_pSimulationLog->logMessage("---> SimulationManagerMPI:: Added ProcessCommunicator");


    m_pDynSys = std::shared_ptr< DynamicsSystemType >( new DynamicsSystemType());
    m_pSimulationLog->logMessage("---> SimulationManagerMPI:: Added DynamicsSystem");

    // Parse the SceneSettings and Static Bodies only ==========================
    {
        DynamicsSystemType::ParserModulesCreator1 c(m_pDynSys.get());
        using SceneParserType1 = SceneParserMPI<DynamicsSystemType,  DynamicsSystemType::ParserModulesCreator1::SceneParserTraits >;
        SceneParserType1 parser( c , m_pSimulationLog ,  ApplicationCLOptions::getSingleton().getMediaDir() );
        m_pSimulationLog->logMessage("---> SimulationManagerMPI:: Added SceneParser and parse Scene");
        parser.parseScene(m_sceneFileParsed) ;

        m_nGlobalSimBodies = parser.getSpecifiedSimBodies();
        LOG(m_pSimulationLog,  "---> Scene parsing finshed: Added "<< m_pDynSys->m_simBodies.size()
            << " simulated & " << m_pDynSys->m_staticBodies.size()<<  " static bodies! "  << std::endl;);
        LOG(m_pSimulationLog,  "---> SimulationManagerMPI:: Global Bodies in Simulation "<< m_nGlobalSimBodies<< " Bodies!" << std::endl;);
    }
    // ====================================================

    // TopologyBuilder ====================
    auto & topoSettings = m_pDynSys->getSettingsTopoBuilder();
    if(topoSettings.m_type == MPILayer::TopologyBuilderEnum::GRIDBUILDER){

        m_pTopologyBuilder = std::shared_ptr< TopologyBuilderType >(
                                                new MPILayer::GridTopologyBuilder<ProcessCommunicatorType>
                                                  (m_pDynSys,m_pProcCommunicator,
                                                   topoSettings.m_rebuildSettings,
                                                   topoSettings.m_massPointPredSettings,
                                                   topoSettings.m_globalOutlierFilterSettings,
                                                   topoSettings.m_gridBuilderSettings,
                                                   m_sceneFileParsed, m_nGlobalSimBodies ) );
        m_pSimulationLog->logMessage("---> SimulationManagerMPI:: Added GridTopologyBuilder");
    }
    else if(topoSettings.m_type == MPILayer::TopologyBuilderEnum::KDTREEBUILDER){

        m_pTopologyBuilder = std::shared_ptr< TopologyBuilderType >(
                                                new MPILayer::KdTreeTopologyBuilder<ProcessCommunicatorType>
                                                  (m_pDynSys,m_pProcCommunicator,
                                                   topoSettings.m_rebuildSettings,
                                                   topoSettings.m_massPointPredSettings,
                                                   topoSettings.m_globalOutlierFilterSettings,
                                                   topoSettings.m_kdTreeBuilderSettings,
                                                   m_sceneFileParsed, m_nGlobalSimBodies ) );
        m_pSimulationLog->logMessage("---> SimulationManagerMPI:: Added KdTreeTopologyBuilder");
    }else{
        GRSF_ERRORMSG("No Topology Builder type instantiated")
    }

    //Uncomment if StateRecorderType = StateRecorderBodys
    //m_pStateRecorder  =  std::shared_ptr<StateRecorderType >(new StateRecorderType(true,m_pProcCommunicator->getProcInfo()->getRank()));

    m_pStateRecorder  =  std::shared_ptr<StateRecorderType >(new StateRecorderType(m_nGlobalSimBodies,
                                                                std::static_pointer_cast<ProcessCommunicatorType::ProcessInfoType>(m_pProcCommunicator)));



    m_pSimulationLog->logMessage("---> SimulationManagerMPI:: Added StateRecorder");
    // =====================================================


    //Uncomment if StateRecorderType = StateRecorderBodys
    //Set up delegate function from BodyCommunicator to StateRecorederBody (close and open files)
    //        m_pBodyCommunicator->addDelegateLocalRemove(
    //            BodyCommunicator<DynamicsSystemType>::AddDelegate::template
    //            from_method< StateRecorderType,&StateRecorderType::removeBody>
    //            (& *m_pStateRecorder.get())
    //        );
    //        m_pBodyCommunicator->addDelegateLocalAdd(
    //            BodyCommunicator<DynamicsSystemType>::AddDelegate::template
    //            from_method< StateRecorderType,&StateRecorderType::addBody>
    //            (& *m_pStateRecorder.get())
    //        );
    // TODO: Danger shared_ptr.get() in delegate class,
    //as along as there is the SimMgrMpi there is a NbCommunicator and a StatRecorder, so no problem
    // ===============================================

    m_pTimestepper = std::shared_ptr< TimeStepperType >( new TimeStepperType(m_pDynSys, m_pProcCommunicator, m_pTopologyBuilder) );
    m_pSimulationLog->logMessage("---> SimulationManagerMPI:: Added TimeStepper");


    auto callBack = std::bind(&SimulationManagerMPI::gracefullyExit,this,std::placeholders::_1);
    ApplicationSignalHandler::getSingleton().registerCallback({SIGINT,SIGUSR2},
                                                              callBack,"SimulationManagerMPI::gracefullyExit");

    m_pSimulationLog->logMessage("---> SimulationManagerMPI:: added ungraceful exit callback!");

    m_pSimulationLog->logMessage("---> SimulationManagerMPI::setup() finished!");


    ApplicationSignalHandler::getSingleton().handlePendingSignals();

}

void SimulationManagerMPI::gracefullyExit(int signal){
    LOG(m_pSimulationLog, "---> SimulationManagerMPI:: Caught signal: " << signal
        << " close file collectively (deadlock if not all cooperate!) " << std::endl )
    m_pStateRecorder->closeAll();
    LOG(m_pSimulationLog, "---> SimulationManagerMPI:: succesfully closed file " << std::endl )
}


void SimulationManagerMPI::checkOwnerOfBodiesInProcess() {
    for(auto & body : m_pDynSys->m_simBodies){
        RankIdType r;
        if(!m_pProcCommunicator->getProcTopo()->belongsBodyToProcess(body,r)){
            GRSF_ERRORMSG("Body with id: " << body->m_id << " belongs to rank: " << r << "instead of: " << m_pProcCommunicator->getRank()) ;
        }
    }
    m_pProcCommunicator->waitBarrier();
}

void SimulationManagerMPI::checkNumberOfBodiesInProcess() {

    int numberOfBodiesInSimulation = 0;
    int nSimBodies = m_pDynSys->m_simBodies.size();

    CPUTimer timer;

    timer.start();
    LOG(m_pSimulationLog,  "MPI> Body check,  MPI_Reduce... "<< std::endl;)
    MPI_Reduce(&nSimBodies,&numberOfBodiesInSimulation, 1, MPI_INT, MPI_SUM, m_pProcCommunicator->getMasterRank(), m_pProcCommunicator->getCommunicator());
    double elapsed = timer.elapsedSec();

    if(m_pProcCommunicator->hasMasterRank()) {
        LOG(m_pSimulationLog,  "MPI> Allreduce needed: " << elapsed <<"sec."<<std::endl; );

        if( static_cast<unsigned int>(numberOfBodiesInSimulation) != m_nGlobalSimBodies) {
            LOG(m_pSimulationLog,  "ERROR:---> Body check NOT successful! : "<< numberOfBodiesInSimulation <<"!="<< m_nGlobalSimBodies<< std::endl;);
            GRSF_ERRORMSG("Body check Not successfull");
        }

        LOG(m_pSimulationLog,  "---> Body check successfull!: "<<numberOfBodiesInSimulation<<" SimBodies in Simulation" << std::endl;);
    } else {
        LOG(m_pSimulationLog,  "---> Body check ... (sent # of bodies) !"<< std::endl;);
    }

}


void SimulationManagerMPI::getMaxRuntime(PREC runtime){

    LOG(m_pSimulationLog,  "MPI> Get Max Runtime,  MPI_Reduce... "<< std::endl;);
    PREC maxRuntime;

    MPI_Reduce(&runtime,&maxRuntime, 1, MPI_DOUBLE, MPI_MAX, m_pProcCommunicator->getMasterRank(), m_pProcCommunicator->getCommunicator());

    if(m_pProcCommunicator->hasMasterRank()){
        LOG(m_pSimulationLog, "---> Total runtime for simulation needed: "<<maxRuntime<< " sec. " << std::endl;);
    }else{
        LOG(m_pSimulationLog, "---> Runtime sent ... !"<< std::endl;);
    }


}



void SimulationManagerMPI::startSim() {


    m_pSimulationLog->logMessage("---> SimulationManager: Simulation entering...");

    initSim();

    if(m_pTimestepper->m_settings.m_eSimulateFromReference == TimeStepperSettings::NONE) {
        m_pStateRecorder->write(m_pTimestepper->getTimeCurrent(),m_pDynSys->m_simBodies);
    }

    PREC runtime;
    m_globalTimer.start();

    while(1){

        ApplicationSignalHandler::getSingleton().handlePendingSignals();

        // If rebuild happens, reset TimeStepper accordingly! ====================================
        bool rebuild = m_pTopologyBuilder->checkAndRebuild(m_pTimestepper->getIterationCount(),
                                                           m_pTimestepper->getTimeCurrent());
        if(rebuild){
            // Reload Scene
            m_pSimulationLog->logMessage("---> Reload SceneObjects ...");
            reloadScene();
             // Reset TimeStepper
            m_pSimulationLog->logMessage("---> Reset TimeStepper Topology ...");
            m_pTimestepper->resetTopology();
        }

        // Do one time step =====================================================================
        m_pTimestepper->doTimeStep();

        writeAllOutput();

        // Check if simulation can be aborted! ==================================================
        if(m_pTimestepper->finished()) {
            runtime = m_globalTimer.elapsedSec();
            LOG(m_pSimulationLog, "---> SimulationManager: Timestepper finished, needed: " << runtime << " sec. , exit..."<< std::endl );
            break;
        }

        // Roll all registered log files ========================================================
        Logging::LogManager::getSingleton().rollAllLogs();
    }

    m_pTimestepper->closeAllFiles();
    m_pStateRecorder->closeAll();

    // MPI Communicat times, get the max runtime ====================================
    getMaxRuntime(runtime);


m_pSimulationLog->logMessage("---> SimulationManager: Simulation leaving...");
return;
}


void SimulationManagerMPI::writeAllOutput() {

    if(m_RecorderSettings.outputCheck(m_pTimestepper->getIterationCount())) {
        // Write Data to SystemDataFile (maps all data to back buffer!)
        m_pTimestepper->writeIterationToSystemDataFile(m_globalTimer.elapsedSec());
        // Write Data to CollisionDataFile
        //m_pTimestepper->writeIterationToCollisionDataFile();
        // Write  State to Sim File
        //m_pStateRecorder->write(m_pTimestepper->getFrontStateBuffer().get());

        // if the recorder would support parallel write , interleaving the calculation we could
        // wait here for the recorder to finish up
        // m_pStateRecorder->wait(); // waits till the recorder has finished its parallel stuff

        m_pStateRecorder->write(m_pTimestepper->getTimeCurrent(), m_pDynSys->m_simBodies);
        // this would then be asynchronous call (copies states internally and writes everything out
    }
}


void SimulationManagerMPI::initSim() {

    MPILayer::GenericMessage<boost::filesystem::path> message;
    if(m_pProcCommunicator->hasMasterRank()) { // Only master process
        // Get new sim folder path relative to global directory
        m_SimFolderPath = FileManager::getSingleton().getNewSimFolderPath(SIMULATION_FOLDER_PATH,SIM_FOLDER_PREFIX_RECORD);

        LOG(m_pSimulationLog,"MPI> Broadcast new simulation directory: "<< m_SimFolderPath <<std::endl;);
        boost::filesystem::path sceneFilePath = m_SimFolderPath;
        std::string filename = SIM_SCENE_FILE_NAME;
        filename += ".xml"; sceneFilePath /= filename;

        LOG(m_pSimulationLog,"---> Copy parsed scene file to: " << sceneFilePath <<std::endl;)
        FileManager::getSingleton().copyFile(m_sceneFileParsed,sceneFilePath,true);

        // Need to broadcast the filename of the actual folder
        // Broadcast: m_SimFolderPath
        std::get<0>(message.m_data) = m_SimFolderPath;
        m_pProcCommunicator->sendBroadcast(message);

    } else { // All other processes
        m_pProcCommunicator->receiveBroadcast(message, m_pProcCommunicator->getMasterRank());
        m_SimFolderPath = std::get<0>(message.m_data);
        LOG(m_pSimulationLog,"MPI> Received new simulation directory: "<< m_SimFolderPath <<std::endl;);
    }

    // Create same folder name in Local Process Directory, to save all data...
    boost::filesystem::path localSimFolderPath = FileManager::getSingleton().getLocalDirectoryPath();
    localSimFolderPath /= m_SimFolderPath.filename();
    if(!boost::filesystem::create_directories(localSimFolderPath)) {
        LOG(m_pSimulationLog,  "ERROR:---> Could not create directory: " << localSimFolderPath.string() << std::endl;);
        GRSF_ERRORMSG("ERROR:---> Could not create directory: " << localSimFolderPath.string() << std::endl )
    }


    m_pStateRecorder->setDirectoryPath(m_SimFolderPath);

    if(!m_pStateRecorder->createSimFile(true)) { // Truncate the files, important!
        LOG(m_pSimulationLog,  "ERROR:---> Could not open SimFilePart: " << localSimFolderPath.string() << std::endl;);
        GRSF_ERRORMSG("ERROR:---> Could not open SimFilePart: " << localSimFolderPath.string() << std::endl)
    }
    // ===============================================

    // Get the actual RecorderSettings
    m_RecorderSettings = m_pDynSys->getSettingsRecorder();


    // Init Topology and load all sim bodies
    m_pTopologyBuilder->initLogs(localSimFolderPath);
    m_pTopologyBuilder->initTopology();

    // Reload Scene
    m_pSimulationLog->logMessage("---> Reload SceneObjects ...");
    reloadScene();

    m_pSimulationLog->logMessage("---> Initialize ProcessCommunicator buffers...");
    m_pProcCommunicator->initializeNeighbourBuffers();

     // Reset TimeStepper
    m_pTimestepper->reset();
    m_pTimestepper->initLogs(localSimFolderPath);

}

void SimulationManagerMPI::reloadScene() {

    LOG(m_pSimulationLog,"---> Clear all remote/local bodies in DynamicsSystem" <<std::endl;)
    m_pDynSys->deleteSimBodies();

    // Parse the scene objects from the XML, but selectively on the ids in m_pDynSys->bodiesInitStates
    {
        DynamicsSystemType::ParserModulesCreator2 c(m_pDynSys.get());
        using SceneParserType2 = SceneParserMPI<DynamicsSystemType, DynamicsSystemType::ParserModulesCreator2::SceneParserTraits >;
        SceneParserType2 parser( c , m_pSimulationLog,  ApplicationCLOptions::getSingleton().getMediaDir());

        SceneParserType2::SceneParserDynamicOptionsType optScene;
        optScene.m_parseSceneSettings = false;
        optScene.m_parseSceneObjects = true;

        SceneParserType2::BodyModuleDynamicOptionsType optBodyM;
        std::set<RigidBodyIdType> bodiesToload;
        for(auto & s: m_pDynSys->m_bodiesInitStates){ bodiesToload.insert(s.second.m_id);}
        optBodyM.m_bodyIdRange = std::move(bodiesToload);

        parser.parseScene(m_sceneFileParsed, optScene , std::move(optBodyM));

        // Apply initial states (received from master) to bodies
        m_pDynSys->applyInitStatesToBodies();

        LOG(m_pSimulationLog,  "---> Scene parsing finshed: Added "<< m_pDynSys->m_simBodies.size()
        << " simulated & " << m_pDynSys->m_staticBodies.size()<<  " static bodies! "  << std::endl;);

        checkOwnerOfBodiesInProcess();
    }

    //=====================================================


    // Do Safty Check
    // Do a Reduction of all Simbodies, to count if no bodies are missing!
    m_pSimulationLog->logMessage("---> Check number of Bodies in Process:");
    checkNumberOfBodiesInProcess();
    // =====================================================

}
