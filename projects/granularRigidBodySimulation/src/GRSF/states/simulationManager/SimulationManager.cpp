// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include <fstream>

#define _USE_MATH_DEFINES
#include <cmath>



#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/AssertionDebug.hpp"
#include "GRSF/common/ApplicationCLOptions.hpp"
#include "GRSF/states/simulationManager/SimulationManager.hpp"

#include "GRSF/dynamics/buffers/DynamicsState.hpp"
#include "GRSF/systems/SharedBufferDynSys.hpp"
#include "GRSF/dynamics/buffers/StateRecorder.hpp"

// HIER ALL Includes welche man für diesen Manager braucht, welche getemplatet sind mit einem
// Config, unterhalb in den Subclassen müssen diese nicht mehr hinzugefügt werden?
//
#include TimeStepper_INCLUDE_FILE
#include InclusionSolver_INCLUDE_FILE
#include DynamicsSystem_INCLUDE_FILE

#include "GRSF/common/CommonFunctions.hpp"
#include "GRSF/dynamics/general/QuaternionHelpers.hpp"

#include "GRSF/singeltons/FileManager.hpp"
#include "GRSF/common/CPUTimer.hpp"


using namespace std;



SimulationManager::SimulationManager() {

    m_pSimulationLog = nullptr;

    if(Logging::LogManager::getSingleton().existsLog("SimulationLog")) {
        m_pSimulationLog = Logging::LogManager::getSingleton().getLog("SimulationLog");
    } else {
        boost::filesystem::path filePath = FileManager::getSingleton().getLocalDirectoryPath();
        filePath /= GLOBAL_LOG_FOLDER_DIRECTORY;
        filePath /= "SimulationLog.log";
        m_pSimulationLog = Logging::LogManager::getSingleton().createLog("SimulationLog",true,true,filePath);
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


    m_pDynSys = std::shared_ptr< DynamicsSystemType >( new DynamicsSystemType());

    // Parse the Scene from XML! ==========================
    DynamicsSystemType::ParserModulesCreator c(m_pDynSys.get());
    m_pSceneParser = std::shared_ptr< SceneParserType >( new SceneParserType(c, m_pSimulationLog, ApplicationCLOptions::getSingleton().getMediaDir() ) );
    m_pSceneParser->parseScene(sceneFilePath);
    m_nSimBodies = m_pDynSys->m_simBodies.size();
    LOG(m_pSimulationLog,  "---> Scene parsing finshed: Added "<< m_pDynSys->m_simBodies.size()
        << " simulated & " << m_pDynSys->m_staticBodies.size()<<  " static bodies! "  << std::endl;);
    // =====================================================

    m_pSharedBuffer = std::shared_ptr<SharedBufferDynSys >(new SharedBufferDynSys(m_pDynSys->m_simBodies.beginKey(),m_pDynSys->m_simBodies.endKey() ));
    m_pTimestepper = std::shared_ptr< TimeStepperType >( new TimeStepperType(m_pDynSys, m_pSharedBuffer) );


    m_pStateRecorder = std::shared_ptr<StateRecorder >(new StateRecorder(m_nSimBodies));

    m_pSharedBuffer->resetStatePool(m_pDynSys->m_bodiesInitStates);

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
            m_pTimestepper->doTimeStep();

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
                double timelineSimulation = m_global_time.elapsedSec();
                // Write Data to SystemDataFile (maps all data to back buffer!)
                m_pTimestepper->writeIterationToSystemDataFile(timelineSimulation);
                // Write Data to CollisionDataFile
                m_pTimestepper->writeIterationToCollisionDataFile();
                // Write  State to Sim File
                m_pStateRecorder->write(m_pTimestepper->getFrontStateBuffer());
            }
}



bool SimulationManager::initRecordThread() {
    // Get new folder path
    m_SimFolderPath = FileManager::getSingleton().getNewSimFolderPath(SIMULATION_FOLDER_PATH,SIM_FOLDER_PREFIX_RECORD);

    // Sim file path
    m_SimFilePath = m_SimFolderPath;
    std::string filename = SIM_FILE_PREFIX;
    filename += SIM_FILE_EXTENSION;
    m_SimFilePath /= filename;

    boost::filesystem::path m_SceneFilePath = m_SimFolderPath;
    filename = SIM_SCENE_FILE_NAME;
    filename += ".xml";
    m_SceneFilePath /= filename;

    FileManager::getSingleton().copyFile(m_pSceneParser->getParsedSceneFile(),m_SceneFilePath,true);


    // Get the actual RecorderSettings
    m_RecorderSettings = m_pDynSys->getSettingsRecorder();

    //Reset Timestepper!
    m_pTimestepper->reset();

    //Open File
    bool fileOK = false;
    if(m_pTimestepper->m_settings.m_eSimulateFromReference == TimeStepperSettings::CONTINUE) {
        fileOK = m_pStateRecorder->createSimFileCopyFromReference(m_SimFilePath,m_pTimestepper->m_settings.m_simStateReferenceFile);
    } else {
        fileOK = m_pStateRecorder->createSimFile(m_SimFilePath);
    }

    if(!fileOK) {
        return false;
    }

    // Copy File: SimulationData
    boost::filesystem::path simDataFile;
    if(!m_pTimestepper->m_settings.m_simDataReferenceFile.empty()){
         ASSERTMSG(false,"HERE IS CODE ZU VERVOLLSTÄNDIGEN! FALSCH!")
         simDataFile = FileManager::getSingleton().copyFile( m_pSceneParser->getParsedSceneFile(), m_pTimestepper->m_settings.m_simDataReferenceFile,true);
    }
    m_pTimestepper->initLogs(m_SimFolderPath,simDataFile);

    // Write first initial value out!
    if(m_pTimestepper->m_settings.m_eSimulateFromReference == TimeStepperSettings::NONE) {
        m_pStateRecorder->write(m_pTimestepper->getTimeCurrent(), m_pDynSys->m_simBodies);
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


