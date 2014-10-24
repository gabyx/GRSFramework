#include <fstream>

#define _USE_MATH_DEFINES
#include <cmath>

#include "PlaybackManager.hpp"

#include <RenderContext.hpp>
#include "SharedBufferPlayback.hpp"
#include "CommonFunctions.hpp"
#include "QuaternionHelpers.hpp"
#include "OgreMeshExtraction.hpp"

#include "SharedBufferPlayback.hpp"
#include "PlaybackLoader.hpp"

#include "FileManager.hpp"
#include "MultiBodySimFile.hpp"

#include "LogDefines.hpp"



PlaybackManager::PlaybackManager(std::shared_ptr<Ogre::SceneManager> pSceneMgr):
    PlaybackManagerBase(),
    m_bSetupSuccessful(false)
{

    m_pSimulationLog = nullptr;

    if(Logging::LogManager::getSingletonPtr()->existsLog("SimulationLog")) {
        m_pSimulationLog = Logging::LogManager::getSingletonPtr()->getLog("SimulationLog");
    } else {
        boost::filesystem::path filePath = FileManager::getSingletonPtr()->getLocalDirectoryPath();
        filePath /= GLOBAL_LOG_FOLDER_DIRECTORY;
        filePath /= "SimulationLog.log";
        m_pSimulationLog = Logging::LogManager::getSingletonPtr()->createLog("SimulationLog",true,true,filePath);
        m_pSimulationLog->logMessage("---> Creating SimulationManager...");
    }

    // Set the Log Output =========================================================================
    m_pThreadLog = new Logging::Log("PlaybackManagerThreadLog");

#if PLAYBACKLOG_TOFILE == 1
    boost::filesystem::path filePath = FileManager::getSingletonPtr()->getLocalDirectoryPath();
        filePath /= GLOBAL_LOG_FOLDER_DIRECTORY;
    filePath /= "PlaybackManagerThread.log";
    m_pThreadLog->addSink(new Logging::LogSinkFile("PlaybackManager-File",filePath));
#endif
#if PLAYBACKLOG_TOCONSOLE == 1
    m_pThreadLog->addSink(new Logging::LogSinkCout("PlaybackManager-Cout"));
#endif


    m_pSceneMgr = pSceneMgr;
}


PlaybackManager::~PlaybackManager() {
    DECONSTRUCTOR_MESSAGE

    stopPlaybackThread(true);

    // delete Log!
    delete m_pThreadLog;

    InputContext::getSingletonPtr()->removeKeyListener(this);
}


bool PlaybackManager::setup() {

    double lengthScale = 100;  // 1m = 100 Ogre units , 1cm -> 1 Ogre Unit
    m_pBaseNode = m_pSceneMgr->getSceneNode("BaseFrame")->createChildSceneNode("BaseFrameScene");
    m_pBaseNode->setScale(Ogre::Vector3(1.0,1.0,1.0)*lengthScale);

    m_pDynSys = std::shared_ptr< DynamicsSystemPlayback > (new DynamicsSystemPlayback(m_pSceneMgr,m_pBaseNode));

    typename DynamicsSystemPlayback::ParserModulesCreator c(m_pDynSys.get());
    m_pSceneParser = std::shared_ptr< SceneParserType >( new SceneParserType(c, m_pSimulationLog ) );

    // Parse the Scene from XML! ==========================
    if(!parseScene()) {
        m_bSetupSuccessful = false;
        return false;
    }
    unsigned int nSimBodies = m_pDynSys->m_SceneNodeSimBodies.size();
    // =====================================================

    //init shared buffer
    m_pSharedBuffer = std::shared_ptr<SharedBufferPlayback >(
                          new SharedBufferPlayback(m_pDynSys->m_SceneNodeSimBodies.beginKey(),
                                                   m_pDynSys->m_SceneNodeSimBodies.endKey())
                      );
    m_pSharedBuffer->resetStateRingPool(m_pDynSys->m_bodiesInitStates);


    m_pFileLoader = std::shared_ptr< PlaybackLoader<StateRingPoolVisBackFront > >(
                        new PlaybackLoader< StateRingPoolVisBackFront >(nSimBodies, m_pSharedBuffer)
                    );

    //Make a videodropper
    m_pVideoDropper = std::shared_ptr<VideoDropper>(new VideoDropper());
    m_pVideoDropper->reset();

    //Make a Sim File Resampler
    m_pStateRecorderResampler = std::shared_ptr<StateRecorderResampler >(new StateRecorderResampler(nSimBodies));
    m_pStateRecorderResampler->reset();

    m_pVisBuffer = m_pSharedBuffer->getVisBuffer();


    enableInput(true);

    m_bSetupSuccessful = true;

    return m_bSetupSuccessful;
}


bool PlaybackManager::parseScene() {
    boost::filesystem::path sceneFilePath = FileManager::getSingleton().getPathSceneFileOfCurrentSimFolder();
    if(sceneFilePath.empty()) {
        return false;
    }


    m_pSceneParser->parseScene(sceneFilePath);

    LOG(m_pSimulationLog,  "---> Scene parsing finshed: Added "<< m_pDynSys->m_SceneNodeSimBodies.size()
        << " simulated & " << m_pDynSys->m_SceneNodeBodies.size() <<  " static bodies! "  << std::endl;);

    return true;
}


void PlaybackManager::updateScene(double timeSinceLastFrame) {
    static bool bStateChanged;
    static double state_time;
    std::stringstream logstream;

    if (isSimThreadRunning() && m_bSetupSuccessful) {
        if(!m_settingsVisThread.m_bFirstPass) {

            m_pVisBuffer = m_pSharedBuffer->updateVisBuffer(bStateChanged);
            if(bStateChanged) {
                updateSimBodies();
            }

            if(m_settingsVisThread.m_bVideoDrop) {
                m_pVideoDropper->tryToDropFrame();
            }

        } else { // Only do at the beginning

            m_settingsVisThread.m_bFirstPass = false;
            // Wait for sim thread! to synchronize
            m_barrier_start.wait();
        }
    }


}


void PlaybackManager::updateSimBodies() {
    //update objects...
    static decltype(m_pDynSys->m_SceneNodeSimBodies.begin()) gBodyIt;
    static decltype(m_pVisBuffer->m_SimBodyStates.begin()) stateIt;
    static unsigned int size;

    gBodyIt = m_pDynSys->m_SceneNodeSimBodies.begin();
    stateIt = m_pVisBuffer->m_SimBodyStates.begin();

    size = std::min(m_pDynSys->m_SceneNodeSimBodies.size(),m_pVisBuffer->m_SimBodyStates.size());

    WARNINGMSG( m_pDynSys->m_SceneNodeSimBodies.size() == m_pVisBuffer->m_SimBodyStates.size()
               , "State Container and SimBodyContainer not the same size: "
               << m_pVisBuffer->m_SimBodyStates.size() <<"!="<< m_pDynSys->m_SceneNodeSimBodies.size());

    for(unsigned int i=0; i<size;++i) {
        gBodyIt->applyBodyState(*stateIt);
        ++gBodyIt;  ++stateIt;
    }
}


double PlaybackManager::getSimulationTime() {
    if(m_pVisBuffer) {
        return m_pVisBuffer->m_t;
    }

    return 0;
}


void PlaybackManager::initBeforeThreads() {
    std::stringstream logstream;

    m_settingsVisThread.m_bFirstPass = true;

    //Set once before the start all values for the video drop!
    m_pVideoDropper->setFPS(m_VideoDropSettings.m_FPS);
    m_settingsVisThread.m_bVideoDrop = m_VideoDropSettings.m_bVideoDrop;
    m_settingsSimThread.m_bVideoDrop = m_VideoDropSettings.m_bVideoDrop;
    if(m_settingsSimThread.m_bVideoDrop) {
        logstream <<"PlaybackManager: Drop Video: "<< "true" << std::endl;
    } else {
        logstream <<"PlaybackManager: Drop Video: "<< "false" << std::endl;
    }

    m_pStateRecorderResampler->setFPS(m_VideoDropSettings.m_FPS);
    m_pStateRecorderResampler->setTimeRange(m_SimFileDropSettings.m_startTime,m_SimFileDropSettings.m_endTime);
    m_settingsSimThread.m_bSimFileDrop = m_SimFileDropSettings.m_bSimFileDrop;
    m_settingsSimThread.m_bSimFileDrop = m_SimFileDropSettings.m_bSimFileDrop;
    m_settingsSimThread.m_bSimFileDropInterpolate = m_SimFileDropSettings.m_bSimFileDropInterpolate;
    if(m_settingsSimThread.m_bSimFileDrop) {
        logstream <<"PlaybackManager: Drop Sim File: "<< "true" << std::endl;
    } else {
        logstream <<"PlaybackManager: Drop Sim File: "<< "false" << std::endl;
    }

    m_pThreadLog->logMessage(logstream);

}

void PlaybackManager::threadRunSimulation() {
    static bool bchangedState;
    static double timelineSimulation, state_time, old_state_time, deltaT;

    m_pThreadLog->logMessage("---> PlaybackManager: SimThread entering...");
    setSimThreadRunning(true);

    std::stringstream logstream;

    initSimThread();

    // Starts file reader thread which reads in all values and pushes ahead the front buffer
    m_pFileLoader->startLoaderThread();
    // Wait for loader thread
    m_pFileLoader->m_barrier_start.wait();

    // Update Simbuffer
    DynamicsState * currentState;
    bool updated = false;
    int i = 0;
//    while(!updated && i < 100){
//        currentState = m_pSharedBuffer->advanceSimBuffer(updated);
    currentState = m_pSharedBuffer->getSimBuffer();
//            for(auto & s : currentState->m_SimBodyStates){
//                std::cout <<"SimState: "<< RigidBodyId::getBodyIdString(s.m_id) << "," << s.m_q << std::endl;
//            }
//        i++;
//    }
//    if(!updated){
//        ERRORMSG("Could not advance sim buffer to first state!")
//    }else{
        LOG(m_pThreadLog,"---> PlaybackManager: Advanced sim buffer to first state @ m_t: " << currentState->m_t << std::endl;);
//    }

    // Wait for vis thread
    m_barrier_start.wait();

    state_time = 0;
    old_state_time = 0;
    bchangedState = false;
    resetTimelineSimulation();

    while(!isSimThreadToBeStopped()) {
        timelineSimulation = getTimelineSimulation();


        if(m_settingsSimThread.m_bVideoDrop) {
            // Dont sync with timeline,

            //Make one step!
            currentState = m_pSharedBuffer->advanceSimBuffer(bchangedState); // Will move to the initial state at the beginning!
            if(bchangedState) {
                state_time = currentState->m_t;
                deltaT = state_time - old_state_time; // Will be zero at the beginning

                m_pVideoDropper->addToCurrentTime(deltaT); // if deltaT==0 or if more than one frames fits into deltaT, toggle dropping!

                m_pVideoDropper->tryToWaitForFrameDrop(); // if toggled frame drop, wait for the drop, which happens in the vis thread!!
            }

        } else {
            // Sync with timeline!
            state_time = currentState->m_t;

            if ( state_time <= timelineSimulation) {
                currentState = m_pSharedBuffer->advanceSimBuffer(bchangedState);
            }
        }

        if(bchangedState) {
            if(m_settingsSimThread.m_bSimFileDrop) { // When the sim buffer has been moved, then ... At the biginning, this moves to the initial state basically!
                //Do a resampling of the states!
                LOG(m_pThreadLog, "---> PlaybackManager: Try to resample state time: " << currentState->m_t<<std::endl;);
                m_pStateRecorderResampler->tryToWrite(currentState,m_settingsSimThread.m_bSimFileDropInterpolate);
            }


            if(currentState->m_StateType == DynamicsState::ENDSTATE) {
                LOG(m_pThreadLog, "---> PlaybackManager: Detected end state at " << currentState->m_t <<" --> leaving..."<<std::endl;);
                break;
            }

            /* LOG(m_pThreadLog, "m_t:" << currentState->m_t <<std::endl;); */

            bchangedState = false;
        }

        old_state_time = state_time;

    }

    m_pFileLoader->stopLoaderThread();

    cleanUpSimThread();

    m_pThreadLog->logMessage("---> PlaybackManager: SimThread leaving...");
    setSimThreadRunning(false);

}


void PlaybackManager::initSimThread() {

    m_pSharedBuffer->resetStateRingPool(m_pDynSys->m_bodiesInitStates);

    if(m_settingsSimThread.m_bVideoDrop) {
        m_pVideoDropper->reset();

        // Request new file Paths for all logs from FileManager
        // Get new folder path
        boost::filesystem::path videoFolderPath = FileManager::getSingletonPtr()->getNewSimFolderPath(VIDEO_FOLDER_PATH,VIDEO_FOLDER_PREFIX);

        m_pVideoDropper->setFolderPath(videoFolderPath);
    }

    if(m_settingsSimThread.m_bSimFileDrop) {

        //Make FileLoader load the whole state!
        m_pFileLoader->setReadFullState(true);
        m_pStateRecorderResampler->reset();

        boost::filesystem::path resampleFolderPath = FileManager::getSingletonPtr()->getNewSimFolderPath(SIMULATION_FOLDER_PATH,SIM_FOLDER_PREFIX_RESAMPLE);
        boost::filesystem::path file_path = resampleFolderPath;
        std::string filename = SIM_FILE_PREFIX;
        filename += SIM_FILE_EXTENSION;
        file_path /= filename;
        if(!m_pStateRecorderResampler->createSimFile(file_path)) {
            m_pSimulationLog->logMessage("---> Could not create sim file!");
            ASSERTMSG(false,"Could not create sim file!");
        };
        FileManager::getSingletonPtr()->copyFile(m_pSceneParser->getParsedSceneFile(),resampleFolderPath,true);
    } else {
        //Make Fileloader load only q!
        m_pFileLoader->setReadFullState(true); // This makes things faster, because of buffering i think, first thought "false" would give better results which does not!
    }

}


void PlaybackManager::cleanUpSimThread() {
    if(m_settingsSimThread.m_bSimFileDrop) {
        m_pStateRecorderResampler->closeAll();
    }
}



bool PlaybackManager::keyPressed(const OIS::KeyEvent &e) {

    switch (e.key) {
    case OIS::KC_M:
        addToTimeScale(0.1);
        break;
    case OIS::KC_N:
        addToTimeScale(-0.1);
        break;
    case OIS::KC_B:
        togglePauseSimulation();
        break;
    default:
        break;
    }
    return true;
}


bool PlaybackManager::keyReleased(const OIS::KeyEvent &e) {
    switch (e.key) {
        default:
            break;
    }
    return true;
}



void PlaybackManager::startPlaybackThread() {
    if(m_bSetupSuccessful) {
        if (!isSimThreadRunning()) {
            setThreadToBeStopped(false);
            //Start  Thread===========================================
            initBeforeThreads();
            m_pThread = new boost::thread( boost::bind(&PlaybackManager::threadRunSimulation, &*this) );

            m_pSimulationLog->logMessage("---> PlaybackManager:: Start Thread: PlaybackThread started ...");
        } else {
            m_pSimulationLog->logMessage("---> PlaybackManager:: A PlaybackThread is already running!");
        }
    } else {
        m_pSimulationLog->logMessage("---> PlaybackManager:: No scene is loaded! No thread can be started!");
    }

}


void PlaybackManager::stopPlaybackThread(bool force_stop) {
    if(m_bSetupSuccessful) {
        if (isSimThreadRunning()) {
            setThreadToBeStopped(true);
            cancelAllWaits();

            m_pThread->join();
            m_pSimulationLog->logMessage("---> PlaybackManager:: Stop Thread: PlaybackThread has been stopped!");
        } else {
            m_pSimulationLog->logMessage("---> PlaybackManager:: Stop Thread: PlaybackThread is not running!");
        }
    } else {
        m_pSimulationLog->logMessage("---> PlaybackManager:: No scene is loaded!, No thread running!");
    }
}



void PlaybackManager::cancelAllWaits() {
    m_pVideoDropper->cancelWait();
}


void PlaybackManager::enableInput(bool value) {
    if(value) {
        // add some key,mouse listener to change the input
        InputContext::getSingletonPtr()->addKeyListener(this,m_KeyListenerName);
    } else {
        // add some key,mouse listener to change the input
        InputContext::getSingletonPtr()->removeKeyListener(this);
    }
}

