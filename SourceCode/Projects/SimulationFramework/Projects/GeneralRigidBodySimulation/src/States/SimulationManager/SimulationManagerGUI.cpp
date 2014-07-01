#include <fstream>

#define _USE_MATH_DEFINES
#include <cmath>

#include "LogDefines.hpp"
#include "AssertionDebug.hpp"

#include "SimulationManagerGUI.hpp"

#include <RenderContext.hpp>

#include "DynamicsState.hpp"
#include "SharedBufferDynSys.hpp"
#include "StateRecorder.hpp"


#include TimeStepper_INCLUDE_FILE
//#include "InclusionSolverNT.hpp"
//#include "InclusionSolverCO.hpp"
#include "InclusionSolverCONoG.hpp"

#include DynamicsSystem_INCLUDE_FILE

#include "CommonFunctions.hpp"
#include "QuaternionHelpers.hpp"
#include "OgreMeshExtraction.hpp"

#include "FileManager.hpp"


using namespace std;




SimulationManagerGUI::SimulationManagerGUI(std::shared_ptr<Ogre::SceneManager> pSceneMgr):
    SimulationManagerBase() {

    m_pSimulationLog = nullptr;

    if(Logging::LogManager::getSingletonPtr()->existsLog("SimulationLog")) {
        m_pSimulationLog = Logging::LogManager::getSingletonPtr()->getLog("SimulationLog");
    } else {
        boost::filesystem::path filePath = FileManager::getSingletonPtr()->getLocalDirectoryPath();
        filePath /= GLOBAL_LOG_FOLDER_DIRECTORY;
        filePath /= "SimulationLog.log";
        m_pSimulationLog = Logging::LogManager::getSingletonPtr()->createLog("SimulationLog",SIMULATION_LOG_TO_CONSOLE,true,filePath);
        m_pSimulationLog->logMessage("---> Creating SimulationManagerGUI...");
    }


    m_KeyListenerName = "SimulationManagerGUI::KeyListener";

    m_pSceneMgr = pSceneMgr;
}

SimulationManagerGUI::~SimulationManagerGUI() {
    DECONSTRUCTOR_MESSAGE

    // Remove all SceneGraph objects!

    m_pBaseNode->removeAndDestroyAllChildren();
    m_pSceneMgr->destroySceneNode(m_pBaseNode);

    InputContext::getSingletonPtr()->removeKeyListener(this);

}

void SimulationManagerGUI::setup(boost::filesystem::path sceneFilePath) {


    m_pSimulationLog->logMessage("---> SimulationManagerGUI::setup(): ");

    m_lengthScale = 100;  // 1m = 100 Ogre units , 1cm -> 1 Ogre Unit
    m_pBaseNode = m_pSceneMgr->getSceneNode("BaseFrame")->createChildSceneNode("BaseFrameScene");
    m_pBaseNode->setScale(Ogre::Vector3(1.0,1.0,1.0)*m_lengthScale);


    m_pDynSys = std::shared_ptr< DynamicsSystemType >( new DynamicsSystemType());
    m_pSimulationLog->logMessage("---> SimulationManagerGUI:: Added DynamicsSystemType... ");

    // Parse the Scene from XML! ==========================

    m_pSceneParser = std::shared_ptr< SceneParserOgre >( new SceneParserOgre( m_pBaseNode, m_pSceneMgr,m_SceneNodeSimBodies,m_SceneNodeBodies, m_pDynSys) );
    m_pSimulationLog->logMessage("---> SimulationManagerGUI:: Added SceneParserOgre... ");


    LOG(m_pSimulationLog,"---> ScenePath: " << sceneFilePath << std::endl );
    if(sceneFilePath.empty()) {
        sceneFilePath = "SceneFile.xml";
    }
    m_pSceneParser->parseScene(sceneFilePath,std::make_pair(RigidBodyId::makeId(1,1),RigidBodyId::makeId(4,1)));

    m_nSimBodies = m_pSceneParser->getNumberOfSimBodies();
    std::cout << " m_nSimBodies: " << m_nSimBodies << std::endl;
    // =====================================================


    m_pSharedBuffer = std::shared_ptr<SharedBufferDynSys>(new SharedBufferDynSys(m_nSimBodies));
    m_pSimulationLog->logMessage("---> SimulationManagerGUI:: Added SharedBufferDynSys... ");
    m_pTimestepper = std::shared_ptr< TimeStepperType >( new TimeStepperType(m_pDynSys, m_pSharedBuffer) );
    m_pSimulationLog->logMessage("---> SimulationManagerGUI:: Added TimeStepperType... ");

    m_pStateRecorder = std::shared_ptr<StateRecorder >(new StateRecorder(m_nSimBodies));
    m_pSimulationLog->logMessage("---> SimulationManagerGUI:: Added StateRecorder... ");

    std::cout << "size simbodies: " << m_pDynSys->m_simBodiesInitStates.size() << std::endl;
    m_pSharedBuffer->resetStatePool(m_pDynSys->m_simBodiesInitStates);
    m_pSceneParser->cleanUp(); // Take care this cleans all stuff
    m_pSimulationLog->logMessage("---> SimulationManagerGUI:: Added SharedBuffer... ");

    m_pVisBuffer = m_pSharedBuffer->updateVisBuffer();

    // Update all Sim Bodies
    updateSimBodies();

    enableInput(true);

    // Init ContactFrameData for visualization
    m_dynCoordFrame.reserve(m_nSimBodies*3); // Approx. 3 contacts per body as init guess
    m_dynCoordFrame.addToScene(m_pBaseNode, "ContactFrameXAxis", "ContactFrameYAxis", "ContactFrameZAxis");

    m_pSimulationLog->logMessage("---> setup finished: ");
}



bool SimulationManagerGUI::writeInitialState() {
    MultiBodySimFile simFile;
    // Request new file Paths for all logs from FileManager
    // Get new folder path
    boost::filesystem::path file = FileManager::getSingletonPtr()->getNewSimFolderPath(SIMULATION_FOLDER_PATH,SIM_FOLDER_PREFIX_INIT);

    std::string filename = SIM_INIT_FILE_PREFIX;
    filename += SIM_INIT_FILE_EXTENSION;
    file /= filename;
    if(simFile.openWrite(file,NDOFqBody,NDOFuBody,m_nSimBodies)) {
        simFile << m_pVisBuffer;
        simFile.close();
        m_pSimulationLog->logMessage(std::string("Successfully written initial state file to:") + file.string() );
        return true;
    }
    m_pSimulationLog->logMessage(std::string("Error while trying to open and write initial state file to:") + file.string() );
    return false;
}



void SimulationManagerGUI::updateScene(double timeSinceLastFrame) {
    static bool bStateChanged;

    if (isSimThreadRunning()) {
        //std::cout << "Simthread running"<<std::endl;
        if(!m_bFirstPass) {

            m_pVisBuffer = m_pSharedBuffer->updateVisBuffer(bStateChanged);

            if(bStateChanged) {
                updateSimBodies();
                updateContactFrameVisualization();
            }
        } else { // Only do at the beginning

            m_bFirstPass = false;

            // Wait for sim thread! to synchronize
            m_barrier_start.wait();

        }
    }


}


void SimulationManagerGUI::initBeforeThreads() {
    m_bFirstPass = true;
}


void SimulationManagerGUI::updateSimBodies() {
    //update objects...
    for(int i=0; i<m_pVisBuffer->m_SimBodyStates.size(); i++) {
        // Check for Nan or Inf
//        cout << m_pVisBuffer->m_SimBodyStates[i].m_q <<endl;
        m_SceneNodeSimBodies[i]->setPosition(
            (Ogre::Real)m_pVisBuffer->m_SimBodyStates[i].m_q(0),
            (Ogre::Real)m_pVisBuffer->m_SimBodyStates[i].m_q(1),
            (Ogre::Real)m_pVisBuffer->m_SimBodyStates[i].m_q(2)
        );
        m_SceneNodeSimBodies[i]->setOrientation(
            (Ogre::Real)m_pVisBuffer->m_SimBodyStates[i].m_q(3),
            (Ogre::Real)m_pVisBuffer->m_SimBodyStates[i].m_q(4),
            (Ogre::Real)m_pVisBuffer->m_SimBodyStates[i].m_q(5),
            (Ogre::Real)m_pVisBuffer->m_SimBodyStates[i].m_q(6)
        );
    }
}


void SimulationManagerGUI::updateContactFrameVisualization() {
    // Fill the shared buffer m_contactData into the render struct
    //cout << "Update Frames" << std::endl;
    m_dynCoordFrame.updateFramesVis();
}



double SimulationManagerGUI::getSimulationTime() {
    return m_pVisBuffer->m_t;
}



void SimulationManagerGUI::threadRunSimulation() {
    m_pSimulationLog->logMessage("---> SimulationManagerGUI: SimThread entering...");
    static double timelineSimulation;
    static double state_time;


    setSimThreadRunning(true);
    initSimThread();



    // wait for vis thread! (which does some loops before)
    m_barrier_start.wait();

    resetTimelineSimulation();
    while(!isSimThreadToBeStopped()) {
        timelineSimulation = getTimelineSimulation() + m_pTimestepper->m_Settings.m_startTime;
        state_time = m_pTimestepper->getTimeCurrent();

        if ( state_time <= timelineSimulation) {
            // Read all inputs
            readSharedBuffer();
            // Do one iteration
            m_pTimestepper->doOneIteration();
            // TODO: after this call the vis buffer is new (will be rendered,
            // but the shared buffer is not yet synchronized)

            // Write outputs
            writeSharedBuffer();

            // Check if simulation can be aborted!
            if(m_pTimestepper->finished()) {
                m_pSimulationLog->logMessage("---> SimulationManagerGUI: Timestepper finished, exit...");
                break;
            }
        }
    }

    cleanUpSimThread();

    m_pSimulationLog->logMessage("---> SimulationManagerGUI: SimThread leaving...");

    setSimThreadRunning(false);

    return;
}





void SimulationManagerGUI::initSimThread() {


    // Request new file Paths for all logs from FileManager
    // Get new folder path
    m_SimFolderPath = FileManager::getSingletonPtr()->getNewSimFolderPath(SIMULATION_FOLDER_PATH,SIM_FOLDER_PREFIX_REALTIME);


    m_pTimestepper->initLogs(m_SimFolderPath);

    m_pTimestepper->reset();



}



void SimulationManagerGUI::cleanUpSimThread() {

    m_pTimestepper->closeAllFiles();
}


void SimulationManagerGUI::threadRunRecord() {

    static double timelineSimulation;

    m_pSimulationLog->logMessage("---> SimulationManagerGUI: RecordThread entering...");
    setSimThreadRunning(true);

    if(initRecordThread()) {


        // wait for vis thread! (which does some loops before)
        m_barrier_start.wait();
        resetTimelineSimulation();

        while(!isSimThreadToBeStopped()) {


            // Read all inputs
            readSharedBuffer();
            // Do one iteration
            m_pTimestepper->doOneIteration();
            // TODO: after this call the vis buffer is new (will be rendered,
            // but the shared buffer is not yet synchronized)

            // Write outputs
            writeSharedBuffer();

            writeAllOutput();

            // Check if simulation can be aborted!
            if(m_pTimestepper->finished()) {
                m_pSimulationLog->logMessage("---> SimulationManagerGUI: Timestepper finished, exit...");
                break;
            }
        }

        cleanUpRecordThread();
    } else {

    }

    setSimThreadRunning(false);
    m_pSimulationLog->logMessage("---> SimulationManagerGUI: RecordThread leaving...");
    return;
}



void SimulationManagerGUI::writeAllOutput() {

    if(m_RecorderSettings.outputCheck(m_pTimestepper->getIterationCount())) {
        //m_pSimulationLog->logMessage("---> Output: now");
        // get global time for logging!
        double timelineSimulation = m_global_time.elapsedSec();
        // Write Data to SystemDataFile (maps all data to back buffer!)
        m_pTimestepper->writeIterationToSystemDataFile(timelineSimulation);
        // Write Data to CollisionDataFile
        m_pTimestepper->writeIterationToCollisionDataFile();
        // Write  State to Sim File
        //m_pStateRecorder->write(m_pTimestepper->getFrontStateBuffer().get());
        m_pStateRecorder->write(m_pTimestepper->getTimeCurrent(), m_pDynSys->m_SimBodies);
    }
}




bool SimulationManagerGUI::initRecordThread() {
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

    // Get the actual RecorderSettings
    m_pDynSys->getSettings(m_RecorderSettings);

    //Reset Timestepper!
    m_pSimulationLog->logMessage("---> Reset Timestepper...");
    m_pTimestepper->reset();
    m_pSimulationLog->logMessage("---> Reset done...");

    // Copy Scene File
    m_pSimulationLog->logMessage("---> Copy SceneFile to right place...");
    FileManager::getSingletonPtr()->copyFile(m_pSceneParser->getParsedSceneFile(),m_SceneFilePath,true);

    //Copy SimeState file, if necessary
    //Open SimState File
    m_pSimulationLog->logMessage("---> Copy SimState.sim to right place...");
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
    m_pSimulationLog->logMessage("---> Copy SimData.dat to right place...");
    boost::filesystem::path simDataFile;
    if(!m_pTimestepper->m_Settings.m_simDataReferenceFile.empty()) {
        ASSERTMSG(false,"HERE IS CODE ZU VERFOLSTÄNDIGEN! FALSCH!")
        simDataFile = FileManager::getSingletonPtr()->copyFile( m_pSceneParser->getParsedSceneFile(), m_pTimestepper->m_Settings.m_simDataReferenceFile,true);
    }

    m_pSimulationLog->logMessage("---> Init Timestepper Logs...");
    m_pTimestepper->initLogs(m_SimFolderPath,simDataFile);

    // Write first initial value out!
    if(m_pTimestepper->m_Settings.m_eSimulateFromReference == TimeStepperSettings::NONE) {
        m_pStateRecorder->write(m_pTimestepper->getTimeCurrent(), m_pDynSys->m_SimBodies);
        m_pSimulationLog->logMessage("---> Wrote first initial value to file...");
    }

    return true;
}

void SimulationManagerGUI::cleanUpRecordThread() {
    m_pTimestepper->closeAllFiles();
    m_pStateRecorder->closeAll();
}



void SimulationManagerGUI::readSharedBuffer() {

}


void SimulationManagerGUI::writeSharedBuffer() {
    static int iterations = 0;
    iterations++;

    //if(iterations%100==0){
    setIterationTime(m_pTimestepper->m_AvgTimeForOneIteration, m_pTimestepper->m_MaxTimeForOneIteration);
    setNumberOfContacts(m_pTimestepper->m_pInclusionSolver->getNContacts());

    //}


    //Update contact frames visualization list
    if(showContactFramesEnabled()){
       auto & collSet = m_pTimestepper->m_pCollisionSolver->getCollisionSetRef();
       m_dynCoordFrame.updateFramesSim(collSet.begin(),collSet.end());
    }


}




bool SimulationManagerGUI::keyPressed(const OIS::KeyEvent &e) {

    switch (e.key) {
    case OIS::KC_M:
        LOG(m_pSimulationLog, "---> Add 0.1 to TimeScale" << std::endl;);
        addToTimeScale(0.1);
        break;
    case OIS::KC_N:
        LOG(m_pSimulationLog, "---> Add -0.1 to TimeScale" << std::endl;);
        addToTimeScale(-0.1);
        break;
    case OIS::KC_B:
        togglePauseSimulation();
        break;
    case OIS::KC_F:
        toggleShowContactFrames();
        break;
    case OIS::KC_UP:
        break;
    case OIS::KC_DOWN:

        break;
    case OIS::KC_LEFT:

        break;
    case OIS::KC_RIGHT:

        break;
    case OIS::KC_NUMPAD2:

        break;

    case OIS::KC_I:
        writeInitialState();
        break;
    }
    return true;
}


bool SimulationManagerGUI::keyReleased(const OIS::KeyEvent &e) {
    switch (e.key) {
    case OIS::KC_UP:
    case OIS::KC_DOWN:

        break;
    case OIS::KC_LEFT:
    case OIS::KC_RIGHT:

        break;
    }
    return true;
}



void SimulationManagerGUI::startSimThread(Threads threadToStart) {

    // TODO Switch Update function to the SimThread update!

    if (!isSimThreadRunning()) {

        setThreadToBeStopped(false);

        //Start Dynamics Thread===========================================
        if(threadToStart & SimulationManagerBase::REALTIME) {
            initBeforeThreads();
            m_pThread = new boost::thread( boost::bind(&SimulationManagerGUI::threadRunSimulation, &*this) );
            m_eSimThreadRunning = SimulationManagerBase::REALTIME;
            m_pSimulationLog->logMessage("---> SimulationManagerGUI:: Start Thread: SimThread in REALTIME mode started ...");
        } else if(threadToStart & SimulationManagerBase::RECORD) {
            initBeforeThreads();
            m_pThread = new boost::thread( boost::bind(&SimulationManagerGUI::threadRunRecord, &*this) );
            m_eSimThreadRunning = SimulationManagerBase::RECORD;
            m_pSimulationLog->logMessage("---> SimulationManagerGUI:: Start Thread: SimThread in RECORD mode started ...");
        }

    } else {
        m_pSimulationLog->logMessage("---> SimulationManagerGUI:: A SimThread is already running!");
    }
}


void SimulationManagerGUI::stopSimThread(Threads threadToStop, bool force_stop) {

    if (isSimThreadRunning()) {
        if((m_eSimThreadRunning & RECORD &&  threadToStop & RECORD) ||
                (m_eSimThreadRunning & REALTIME &&  threadToStop & REALTIME) || force_stop == true) {
            setThreadToBeStopped(true);
            m_pThread->join();
            m_pSimulationLog->logMessage("---> SimulationManagerGUI:: Stop Thread: SimThread has been stopped!");

            m_eSimThreadRunning = SimulationManagerBase::NONE;
        } else {
            if(threadToStop & RECORD) {
                m_pSimulationLog->logMessage("---> SimulationManagerGUI:: Stop Thread: \n To stop the thread which is running in REALTIME mode, you need to switch to REALTIME mode to stop it!");
            } else {
                m_pSimulationLog->logMessage("---> SimulationManagerGUI:: Stop Thread: \n To stop the thread which is running in RECORD mode, you need to switch to RECORD mode to stop it!");
            }
        }
    } else {
        m_pSimulationLog->logMessage("---> SimulationManagerGUI:: Stop Thread: SimThread is not running!");
    }
}


void SimulationManagerGUI::enableInput(bool value) {
    if(value) {
        // add some key,mouse listener to change the input
        InputContext::getSingletonPtr()->addKeyListener(this,m_KeyListenerName);
    } else {
        // remove key,mouse listener to change the input
        InputContext::getSingletonPtr()->removeKeyListener(this);
    }
}



void SimulationManagerGUI::toggleShowContactFrames(){
    // locak mutex
    boost::mutex::scoped_lock l (m_mutexShowContactFrames);
    if(m_bShowContactFrames){
        m_pSimulationLog->logMessage("---> SimulationManagerGUI:: Show ContactFrames: False");
        m_bShowContactFrames = false;
    }else{
        m_pSimulationLog->logMessage("---> SimulationManagerGUI:: Show ContactFrames: True");
        m_bShowContactFrames = true;
    }
}



bool SimulationManagerGUI::showContactFramesEnabled(){
    boost::mutex::scoped_lock l (m_mutexShowContactFrames);
    return m_bShowContactFrames;
}
