#include <fstream>

#define _USE_MATH_DEFINES
#include <cmath>

#include <boost/timer/timer.hpp>

#include "LogDefines.hpp"
#include "AssertionDebug.hpp"

#include "SimulationManager.hpp"

#include "DynamicsState.hpp"
#include "SharedBufferDynSys.hpp"
#include "StateRecorder.hpp"

// HIER ALL Includes welche man für diesen Manager braucht, welche getemplatet sind mit einem
// Config, unterhalb in den Subclassen müssen diese nicht mehr hinzugefügt werden?
//
#include TimeStepper_INCLUDE_FILE
#include InclusionSolver_INCLUDE_FILE
#include DynamicsSystem_INCLUDE_FILE

#include "CommonFunctions.hpp"
#include "QuaternionHelpers.hpp"

#include "FileManager.hpp"


using namespace std;



SimulationManager::SimulationManager() {

    m_pSimulationLog = NULL;

    if(Logging::LogManager::getSingletonPtr()->existsLog("SimulationLog")) {
        m_pSimulationLog = Logging::LogManager::getSingletonPtr()->getLog("SimulationLog");
    } else {
        boost::filesystem::path filePath = FileManager::getSingletonPtr()->getLocalDirectoryPath();
        filePath /= GLOBAL_LOG_FOLDER_DIRECTORY;
        filePath /= "SimulationLog.log";
        m_pSimulationLog = Logging::LogManager::getSingletonPtr()->createLog("SimulationLog",true,true,filePath);
        m_pSimulationLog->logMessage("---> Creating SimulationManager...");
    }


}

SimulationManager::~SimulationManager() {
    DECONSTRUCTOR_MESSAGE

}


void SimulationManager::setup() {
    setup("SceneFile.xml");
}


void SimulationManager::setup(boost::filesystem::path sceneFilePath) {


    m_pSimulationLog->logMessage("---> SimulationManager::setup(): ");


    m_pDynSys = boost::shared_ptr< DynamicsSystemType >( new DynamicsSystemType());

    // Parse the Scene from XML! ==========================
    m_pSceneParser = boost::shared_ptr< SceneParser >( new SceneParser( m_pDynSys) );
    m_pSceneParser->parseScene(sceneFilePath);

    m_nSimBodies = m_pSceneParser->getNumberOfSimBodies();
    LOG(m_pSimulationLog,  "---> Scene parsing finshed: Added "<< m_nSimBodies << " Bodies!"  << std::endl;);
    // =====================================================


    m_pSharedBuffer = boost::shared_ptr<SharedBufferDynSys >(new SharedBufferDynSys(m_nSimBodies));
    m_pTimestepper = boost::shared_ptr< TimeStepperType >( new TimeStepperType(m_pDynSys, m_pSharedBuffer) );


    m_pStateRecorder = boost::shared_ptr<StateRecorder >(new StateRecorder(m_nSimBodies));

    m_pSharedBuffer->resetStatePool(m_pDynSys->m_simBodiesInitStates);

    m_pSceneParser->cleanUp();

    m_pSimulationLog->logMessage("---> setup finished! ");
}



void SimulationManager::threadRunRecord() {
    m_pSimulationLog->logMessage("---> SimulationManager: Simulation entering...");


    if(initRecordThread()) {

        // wait for vis thread! (which does some loops before)

        m_global_time.start();


        while(1) {

            // Do one iteration
            m_pTimestepper->doOneIteration();

            writeAllOutput();

            // Check if simulation can be aborted!
            if(m_pTimestepper->finished()) {
                m_pSimulationLog->logMessage("---> SimulationManager: Timestepper finished, exit...");
                break;
            }
        }

        cleanUpRecordThread();
    }

    m_pSimulationLog->logMessage("---> SimulationManager: Simulation leaving...");
    return;
}


void SimulationManager::writeAllOutput() {

            //TODO
            if(m_RecorderSettings.outputCheck(m_pTimestepper->getIterationCount())){
                 // get global time for logging!
                double timelineSimulation = ((double)m_global_time.elapsed().wall)*1e-9;
                // Write Data to SystemDataFile (maps all data to back buffer!)
                m_pTimestepper->writeIterationToSystemDataFile(timelineSimulation);
                // Write Data to CollisionDataFile
                m_pTimestepper->writeIterationToCollisionDataFile();
                // Write  State to Sim File
                m_pStateRecorder->write(m_pTimestepper->getFrontStateBuffer().get());
            }
}



bool SimulationManager::initRecordThread() {
    // Get new folder path
    m_SimFolderPath = FileManager::getSingletonPtr()->getNewSimFolderPath(SIMULATION_FOLDER_PATH,SIM_FOLDER_PREFIX_RECORD);

    // Sim file path
    m_SimFilePath = m_SimFolderPath;
    std::string filename = SIM_FILE_PREFIX;
    filename += SIM_FILE_EXTENSION;
    m_SimFilePath /= filename;

    boost::filesystem::path m_SceneFilePath = m_SimFolderPath;
    filename = SIM_SCENE_FILE_NAME;
    filename += ".xml";
    m_SceneFilePath /= filename;

    FileManager::getSingletonPtr()->copyFile(m_pSceneParser->getParsedSceneFile(),m_SceneFilePath,true);


    // Get the actual RecorderSettings
    m_pDynSys->getSettings(m_RecorderSettings);

    //Reset Timestepper!
    m_pTimestepper->reset();

    //Open File
    bool fileOK = false;
    if(m_pTimestepper->m_Settings.m_eSimulateFromReference == TimeStepperSettings::CONTINUE) {
        fileOK = m_pStateRecorder->createSimFileCopyFromReference(m_SimFilePath,m_pTimestepper->m_Settings.m_simStateReferenceFile);
    } else {
        fileOK = m_pStateRecorder->createSimFile(m_SimFilePath);
    }

    if(!fileOK) {
        return false;
    }

    // Copy File: SimulationData
    boost::filesystem::path simDataFile;
    if(!m_pTimestepper->m_Settings.m_simDataReferenceFile.empty()){
         ASSERTMSG(false,"HERE IS CODE ZU VERFOLSTÄNDIGEN! FALSCH!")
         simDataFile = FileManager::getSingletonPtr()->copyFile( m_pSceneParser->getParsedSceneFile(), m_pTimestepper->m_Settings.m_simDataReferenceFile,true);
    }
    m_pTimestepper->initLogs(m_SimFolderPath,simDataFile);

    // Write first initial value out!
    if(m_pTimestepper->m_Settings.m_eSimulateFromReference == TimeStepperSettings::NONE) {
        m_pStateRecorder->write(m_pTimestepper->getTimeCurrent(), m_pDynSys->m_SimBodies);
        m_pSimulationLog->logMessage("---> Wrote first initial value to file...");
    }

    return true;
}

void SimulationManager::cleanUpRecordThread() {
    m_pTimestepper->closeAllFiles();
    m_pStateRecorder->closeAll();
}


void SimulationManager::startSim() {
    threadRunRecord();
}


