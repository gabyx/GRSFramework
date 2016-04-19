// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/states/SimulationState.hpp"

#include "GRSF/common/ApplicationCLOptions.hpp"
#include "GRSF/common/ApplicationSignalHandler.hpp"

#include "GRSF/states/simulationManager/SimulationManagerGUI.hpp"
#include "GRSF/states/simulationManager/PlaybackManager.hpp"

#include "GRSF/singeltons/FileManager.hpp"
#include "GRSF/common/OgreSceneManagerDeleter.hpp"


using namespace Ogre;
using namespace OgreBites;
using namespace Eigen;

SimulationState::SimulationState() {
    m_lengthScale = 100;
    m_pBaseNode = nullptr;
    m_SceneDetailIndex = 0;

    m_eMouseMode = CAMERA;
    m_eSimulationActiveMode = REALTIME;


    m_pAppLog = RenderContext::getSingleton().m_pAppLog;

}

SimulationState::~SimulationState() {
    DESTRUCTOR_MESSAGE
    //exit();
}

void SimulationState::enter() {

    m_pAppLog->logMessage("Entering SimulationState...");


    m_pSceneMgr = std::shared_ptr<Ogre::SceneManager>( RenderContext::getSingleton().m_pRoot->createSceneManager(ST_GENERIC, "SimulationStateSceneMgr"), OgreSceneManagerDeleter());
    m_pAppLog->logMessage("---> Added new scene manager...");

    RenderContext::getSingleton().addOverlaySystem( m_pSceneMgr.get() );
    m_pAppLog->logMessage("---> Added overlay sytem...");

    setupScene();

    m_pAppLog->logMessage("Adding InputContext...");
    ::InputContext::getSingleton().addKeyListener(this,"SimulationState::KeyListener");

    m_pAppLog->logMessage("Adding OgreBites::SdkTrayManager...");
    m_pTrayMgr = std::shared_ptr< OgreBites::SdkTrayManager>(new OgreBites::SdkTrayManager("SimulationStateTray",
                 RenderContext::getSingleton().m_pRenderWnd,
                 ::InputContext::getSingleton().getInputContext(), this));

    m_pAppLog->logMessage("Adding MenuMouse...");
    m_pMenuMouse = std::shared_ptr< MenuMouse >(new MenuMouse(m_pTrayMgr,"MenuMouse"));
    m_pMenuMouse->setInactive();


    m_pTrayMgr->hideBackdrop();
    setupParamsPanel();

    setSimulationMode(1);
    setupActiveModeSelection();

    // Setup the Simulation Manager with the loaded system;
    m_pAppLog->logMessage("Adding SimulationManagerGUI...");
    m_pSimMgr  = std::shared_ptr<SimulationManagerGUI > (new SimulationManagerGUI(m_pSceneMgr));
    m_pSimMgr->setup(ApplicationCLOptions::getSingleton().getSceneFile());

    updateSceneFunction = boost::bind(&SimulationState::updateSceneRealtime,this,_1);

    m_pTimelineRendering = std::shared_ptr<Ogre::Timer>(new Ogre::Timer());


}


bool SimulationState::pause() {

    m_pAppLog->logMessage("Pausing SimulationState...");

    m_pOrbitCamera->disableInput();
    m_pMenuMouse->setInactive();
    m_pTrayMgr->hideAll();

    ::InputContext::getSingleton().removeKeyListener(this);
    m_pSimMgr->enableInput(false);
    return true;
}


void SimulationState::resume() {
    m_pAppLog->logMessage("Resuming SimulationState...");

    setMouseMode(false);
    m_pOrbitCamera->setActive();
    m_pTrayMgr->showAll();

    ::InputContext::getSingleton().addKeyListener(this,"SimulationState::KeyListener");
    m_pSimMgr->enableInput(true);
}


void SimulationState::exit() {
    //Delete Tray
    //m_pTrayMgr->destroyAllWidgets();
    m_PhysicsStatsParams.clear();
    m_pTrayMgr.reset();

    m_pAppLog->logMessage("Leaving SimulationState...");	// DEBUG	runtime error on quitting...

    //Delete Camera
    m_pOrbitCamera.reset();

    // Delete MenuMouse
    m_pMenuMouse.reset();

    //Delete SceneMgr
    m_pSceneMgr.reset();

    //Delete Sim Mgr
    m_pSimMgr.reset();


    //Delete Timer
    m_pTimelineRendering.reset();

    ::InputContext::getSingleton().removeKeyListener(this);
}


void SimulationState::setupParamsPanel() {
    m_pPhysicsStatsPanel = (m_pTrayMgr->createParamsPanel(OgreBites::TL_TOPLEFT, "Physics Stats", 300, 8));
    m_PhysicsStatsParams.push_back("Timeline Rendering");
    m_PhysicsStatsParams.push_back("Visualization Time");
    m_PhysicsStatsParams.push_back("Simulation Time");
    m_PhysicsStatsParams.push_back("Delay");
    m_PhysicsStatsParams.push_back("Average Iter. Time");
    m_PhysicsStatsParams.push_back("Max Iter. Time");
    m_PhysicsStatsParams.push_back("Time Scale");
    m_PhysicsStatsParams.push_back("# Contacts:");
    m_pPhysicsStatsPanel->setAllParamNames(m_PhysicsStatsParams);
}

void SimulationState::setupActiveModeSelection() {

    Ogre::StringVector items;
    items.push_back("Realtime");
    items.push_back("Record");
    m_pActiveModeSelectMenu = (m_pTrayMgr->createThickSelectMenu(TL_TOPRIGHT,"ActiveModeSelectionSimulation","Simulations Mode",200,3,items));
    // Set active in Menu
    m_pActiveModeSelectMenu->selectItem((int)m_eSimulationActiveMode);
}


void SimulationState::itemSelected(OgreBites::SelectMenu * menu) {

    if( menu == m_pActiveModeSelectMenu) {
        Ogre::DisplayString str = menu->getSelectedItem();
        if( str == Ogre::UTFString("Realtime") ) {
            setSimulationMode(0);
        } else if(str ==  Ogre::UTFString("Record") ) {
            setSimulationMode(1);
        }
    }
}

void SimulationState::setSimulationMode(int i) {
    switch(i) {
    case 0:
        m_pAppLog->logMessage("SimulationState:: Switched to REALTIME");
        m_eSimulationActiveMode = REALTIME;
        updateSceneFunction = boost::bind(&SimulationState::updateSceneRealtime,this,_1);
        break;
    case 1:
        m_pAppLog->logMessage("SimulationState:: Switched to RECORD");
        m_eSimulationActiveMode = RECORD;
        updateSceneFunction = boost::bind(&SimulationState::updateSceneRealtime,this,_1);
        break;
    }
}
void SimulationState::switchSimulationMode() {
    m_eSimulationActiveMode = static_cast<ActiveMode>(((int)m_eSimulationActiveMode+1)%m_pActiveModeSelectMenu->getNumItems()) ;
    // Set active in Menu
    m_pActiveModeSelectMenu->selectItem((int)m_eSimulationActiveMode);
}

void SimulationState::setupScene() {


    //World Axes
    Entity* ent = m_pSceneMgr->createEntity("WorldAxes", "axes.mesh");
    SceneNode* WorldAxes = m_pSceneMgr->getRootSceneNode()->createChildSceneNode("WorldAxes");
    WorldAxes->attachObject(ent);

    m_pOrbitCamera = std::shared_ptr<OrbitCamera>(new OrbitCamera(m_pSceneMgr.get(),"SimulationState::OrbitCam", 0.13, 50, 200, 0, M_PI/4));
    m_pOrbitCamera->enableInput();
    // Push attachable objects for Orbit camera to list
    m_pOrbitCamera->m_OrbitNodeList.push_back(WorldAxes);

    m_pBaseNode =  m_pSceneMgr->getRootSceneNode()->createChildSceneNode("BaseFrame");

    Ogre::Light* pLight = m_pSceneMgr->createLight("Light");
    pLight->setType(Ogre::Light::LT_DIRECTIONAL);
    pLight->setDirection(Ogre::Vector3(-1,-1,-1));
    pLight->setDiffuseColour((Ogre::Real)0.8,(Ogre::Real) 0.8,(Ogre::Real) 0.8);
    pLight->setPosition( 75, 75, 300 );
    //pLight->setAttenuation(5000000,0,0.1,0);
    pLight->setCastShadows(true);

    pLight = m_pSceneMgr->createLight("Light2");
    pLight->setType(Ogre::Light::LT_DIRECTIONAL);
    pLight->setDirection(Ogre::Vector3(1,1,-1));
    pLight->setDiffuseColour((Ogre::Real)0.8,(Ogre::Real) 0.8,(Ogre::Real) 0.8);
    pLight->setPosition( -75, -75, 300 );
    pLight->setCastShadows(false);

//    m_pSceneMgr->setAmbientLight(Ogre::ColourValue((Ogre::Real)0.2, (Ogre::Real)0.2, (Ogre::Real)0.2));


// pLight->setAttenuation(5000000,0,0.1,0);
    // Set Shadow Technique
    //m_pSceneMgr->setShadowTechnique(SHADOWTYPE_STENCIL_ADDITIVE);
//  m_pSceneMgr->setShadowTextureSelfShadow(true);
//  m_pSceneMgr->setShadowTextureSettings(512, 2);
//  m_pSceneMgr->setShadowTextureSize(512);
//  m_pSceneMgr->setShadowFarDistance(1000);
//
//  m_pSceneMgr->setShadowTextureCountPerLightType(Ogre::Light::LT_DIRECTIONAL, 1);
//
//m_pSceneMgr->setShadowTextureCount(1);
//m_pSceneMgr->setShadowTextureConfig(0, 1024, 1024, Ogre::PF_FLOAT32_R);
////    Mgr->setShadowTextureConfig(1, 512, 512, Ogre::PF_FLOAT32_R);
////    Mgr->setShadowTextureConfig(2, 256, 256, Ogre::PF_FLOAT32_R);
//m_pSceneMgr->setShadowTextureSelfShadow(true);
//m_pSceneMgr->setShadowCasterRenderBackFaces(true);


}


bool SimulationState::keyPressed(const OIS::KeyEvent &keyEventRef) {
    switch(keyEventRef.key) {
    case OIS::KC_2: {
        switchSimulationMode();
        break;
    }
    case OIS::KC_1: {
        setMouseMode(true);
        return false;
    }
    case OIS::KC_4: {
        toggleMPIVisualization();
        return false;
    }
    case OIS::KC_R: {
        switchSceneDetailIndex();
        break;
    }
    case OIS::KC_ESCAPE: {
        m_pSimMgr->stopSimThread(SimulationManagerBase::ALL, true);
        this->popAppState();
        return false;
    }
    case OIS::KC_F1:
    case OIS::KC_0: {
        switchToPlaybackState();
        return false;
    }
    case OIS::KC_L: {
        if(m_eSimulationActiveMode == REALTIME) {
            m_pSimMgr->startSimThread(SimulationManagerBase::REALTIME);
        } else if(m_eSimulationActiveMode == RECORD) {
            m_pSimMgr->startSimThread(SimulationManagerBase::RECORD);
        }
        break;
    }
    case OIS::KC_K: {
        if(m_eSimulationActiveMode == REALTIME) {
            m_pSimMgr->stopSimThread(SimulationManagerGUI::REALTIME,false);
        } else if(m_eSimulationActiveMode == RECORD) {
            m_pSimMgr->stopSimThread(SimulationManagerGUI::RECORD,false);
        }
        break;
    }
    case OIS::KC_U: {
        break;
    }
    case  OIS::KC_I: {

        break;
    }
    default:
        break;
    }

    return true;
}

void SimulationState::switchToPlaybackState() {
    std::shared_ptr<RenderAppState> appState = this->findAppStateByName("PlaybackState");
    if(appState) {
        this->pushAppState(appState);
    }
}

bool SimulationState::keyReleased(const OIS::KeyEvent &keyEventRef) {
    return true;
}

void SimulationState::setMouseMode(bool switchMode = false) {
    if(switchMode) {
        if(m_eMouseMode == MENU) {
            m_eMouseMode = CAMERA;
            std::cout << " Switched to Camera Mode"<<std::endl;
            m_pOrbitCamera->enableInput();
            m_pMenuMouse->setInactive();
        } else {
            m_eMouseMode = MENU;
            std::cout << " Switched to Mouse Mode"<<std::endl;
            m_pOrbitCamera->disableInput();
            m_pMenuMouse->setActive();
        }
    } else {
        if(m_eMouseMode == MENU) {
            m_pOrbitCamera->disableInput();
            m_pMenuMouse->setActive();

        } else {
            m_pOrbitCamera->enableInput();
            m_pMenuMouse->setInactive();
        }
    }
}

void SimulationState::switchSceneDetailIndex() {
    m_SceneDetailIndex = (m_SceneDetailIndex+1)%3 ;
    switch (m_SceneDetailIndex) {
    case 0 :
        m_pOrbitCamera->getCamera()->setPolygonMode(PM_SOLID);
        break;
    case 1 :
        m_pOrbitCamera->getCamera()->setPolygonMode(PM_WIREFRAME);
        break;
    case 2 :
        m_pOrbitCamera->getCamera()->setPolygonMode(PM_POINTS);
        break;
    }
}


void SimulationState::toggleMPIVisualization() {
    try{
        auto * p =m_pSceneMgr->getManualObject("MPITopoSubGridManual");

        if(p->isVisible()){
            p->setVisible(false);
        }else{
            p->setVisible(true);
        }
    }catch(...){}
}

void SimulationState::update(double timeSinceLastFrame) {

    static int counter = 0;
    if( counter++ % 100 == 0){
        ApplicationSignalHandler::getSingleton().handlePendingSignals();
    }


    // Update Camera!
    m_pOrbitCamera->update(timeSinceLastFrame);

    // HERE make a function wrapper which calls two different modes for realtime and playback
    updateSceneFunction(timeSinceLastFrame);
}

void SimulationState::updateSceneRealtime(double timeSinceLastFrame) {
    updateParamsPanelSimulation();
    m_pSimMgr->updateScene(timeSinceLastFrame);
}


void SimulationState::updateParamsPanelSimulation() {
    static std::stringstream GlobalTimeText, DynamicsThreadTimeText, SimTimeText, DelayText, TimeFactorText, AverageIterTimeText, MaxIterTimeText, NContactsText;
    static double Realtime, SimTime, AverageIterTime, MaxIterTime,NContacts; // These variables are not acces by another thread!! only for the function updateTimerText !

    static int iterations = 0;
    iterations++;

    if(iterations%50==0) {

        SimTime = m_pSimMgr->getSimulationTime();
        Realtime = m_pSimMgr->getTimelineSimulation();
        m_pSimMgr->getIterationTime(AverageIterTime,MaxIterTime);

        GlobalTimeText.str("");
        DynamicsThreadTimeText.str("");
        SimTimeText.str("");
        DelayText.str("");
        TimeFactorText.str("");
        AverageIterTimeText.str("");
        MaxIterTimeText.str("");
        NContactsText.str("");

        m_pPhysicsStatsValues.clear();
        GlobalTimeText << std::setw(10) <<std::fixed<< std::setprecision (4) << m_pTimelineRendering->getMilliseconds()*1e-3;
        DynamicsThreadTimeText << std::setprecision (4) << Realtime;
        SimTimeText <<std::fixed<< std::setprecision (4) << SimTime;
        if( m_pSimMgr->isSimulationPaused()){
            SimTimeText<<" (Paused)";
        }
        DelayText <<std::fixed<< std::setprecision (4) << Realtime-SimTime;
        AverageIterTimeText << std::fixed<< std::setprecision (4) <<AverageIterTime;
        MaxIterTimeText << std::fixed<< std::setprecision (4) << MaxIterTime;
        TimeFactorText << m_pSimMgr->getTimeScale();
        NContactsText << m_pSimMgr->getNumberOfContacts();

        m_pPhysicsStatsValues.push_back(GlobalTimeText.str());
        m_pPhysicsStatsValues.push_back(DynamicsThreadTimeText.str());
        m_pPhysicsStatsValues.push_back(SimTimeText.str());
        m_pPhysicsStatsValues.push_back(DelayText.str());
        m_pPhysicsStatsValues.push_back(AverageIterTimeText.str());
        m_pPhysicsStatsValues.push_back(MaxIterTimeText.str());
        m_pPhysicsStatsValues.push_back(TimeFactorText.str());
        m_pPhysicsStatsValues.push_back(NContactsText.str());
        m_pPhysicsStatsPanel->setAllParamValues(m_pPhysicsStatsValues);

    }
}


