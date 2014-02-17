#include "PlaybackState.hpp"

#include "SimulationManagerGUI.hpp"
#include "PlaybackManager.hpp"

#include "FileManager.hpp"
#include "OgreSceneManagerDeleter.hpp"
#include "SimulationManagerGUI.hpp"


using namespace Ogre;
using namespace OgreBites;
using namespace Eigen;

PlaybackState::PlaybackState() {
    m_lengthScale = 100;
    m_SceneDetailIndex = 0;

    m_eMouseMode = CAMERA;
    m_eSimulationActiveMode = PLAYBACK;

    m_pAppLog = RenderContext::getSingletonPtr()->m_pAppLog;


}

PlaybackState::~PlaybackState() {
    DECONSTRUCTOR_MESSAGE
    exit();
}

void PlaybackState::enter() {

    m_pAppLog->logMessage("Entering PlaybackState...");

    m_pSceneMgr = boost::shared_ptr<Ogre::SceneManager>( RenderContext::getSingletonPtr()->m_pRoot->createSceneManager(ST_GENERIC, "PlaybackStateSceneMgr"), OgreSceneManagerDeleter());
    setupScene();


    InputContext::getSingletonPtr()->addKeyListener(this,"PlaybackState::KeyListener");
    InputContext::getSingletonPtr()->addMouseListener(this,"PlaybackState::MouseListener");

    m_pTrayMgr = boost::shared_ptr< OgreBites::SdkTrayManager>(new OgreBites::SdkTrayManager("PlaybackStateTray",
                 RenderContext::getSingletonPtr()->m_pRenderWnd,
                 InputContext::getSingletonPtr()->getMouse(), this));
    m_pTrayMgr->hideBackdrop();

    m_pMenuMouse = boost::shared_ptr< MenuMouse >(new MenuMouse(m_pTrayMgr,"MenuMouse"));
    m_pMenuMouse->setInactive();




    // Setup the Playback Panel and the Playback Manager with the loaded system;
    setupGUI();

    // If no scene has been automatically loaded from the panel, and therefore no playback manager is instantiated, -> change the scene!
    if(!m_pPlaybackMgr) {
        changeScene();
    }

    m_pTimelineRendering = boost::shared_ptr<Ogre::Timer>(new Ogre::Timer());

    setSimulationMode(0);
}


bool PlaybackState::pause() {
    RenderContext::getSingletonPtr()->m_pAppLog->logMessage("Pausing PlaybackState...");

    m_pOrbitCamera->disableInput();
    m_pMenuMouse->setInactive();
    m_pTrayMgr->hideAll();

    InputContext::getSingletonPtr()->removeKeyListener(this);
    InputContext::getSingletonPtr()->removeMouseListener(this);
    m_pPlaybackMgr->enableInput(false);
    return true;
}


void PlaybackState::resume() {
    m_pAppLog->logMessage("Resuming PlaybackState...");

    setMouseMode(false);
    m_pTrayMgr->showAll();

    InputContext::getSingletonPtr()->addKeyListener(this,"PlaybackState::KeyListener");
    InputContext::getSingletonPtr()->addMouseListener(this,"PlaybackState::MouseListener");
    m_pPlaybackMgr->enableInput(true);
}


void PlaybackState::exit() {
    //Delete Tray
    //m_pTrayMgr->destroyAllWidgets();
    m_PhysicsStatsParams.clear();
    m_pTrayMgr.reset();

    m_pAppLog->logMessage("Leaving PlaybackState...");	// DEBUG	runtime error on quitting...

    //Delete Camera
    m_pOrbitCamera.reset();

    // Delete MenuMouse
    m_pMenuMouse.reset();

    //Delete SceneMgr
    m_pSceneMgr.reset();

    // Delete Playback Mgr
    m_pPlaybackMgr.reset();

    //Delete Timer
    m_pTimelineRendering.reset();

    InputContext::getSingletonPtr()->removeKeyListener(this);
    InputContext::getSingletonPtr()->removeMouseListener(this);
}


void PlaybackState::setupParamsPanel() {
    m_pPhysicsStatsPanel = (m_pTrayMgr->createParamsPanel(OgreBites::TL_TOPLEFT, "Playback Stats", 300, 8));
    m_PhysicsStatsParams.push_back("Global Time");
    m_PhysicsStatsParams.push_back("Visualization Time");
    m_PhysicsStatsParams.push_back("Simulation Time");
    m_PhysicsStatsParams.push_back("Delay");
    m_PhysicsStatsParams.push_back("Average Iter. Time");
    m_PhysicsStatsParams.push_back("Max Iter. Time");
    m_PhysicsStatsParams.push_back("Time Scale");
    m_PhysicsStatsParams.push_back("# Contacts:");
    m_pPhysicsStatsPanel->setAllParamNames(m_PhysicsStatsParams);
}

void PlaybackState::setupActiveModeSelection() {

    Ogre::StringVector items;
    items.push_back("Playback");
    m_pActiveModeSelectMenu = (m_pTrayMgr->createThickSelectMenu(TL_TOPRIGHT,"ActiveModeSelectionPlayback","Simulations Mode",200,3,items));

}

void PlaybackState::setupGUI() {

    setupParamsPanel();
    setupActiveModeSelection();


    m_pTrayMgr->hideBackdrop();
    m_pPlaybackFiles = (m_pTrayMgr->createThickSelectMenu(
                            TL_TOP,"PlaybackFiles","Playback Files",600,10,
                            Ogre::StringVector()
                        ));
    updatePlaybackPanel();

    // Setup reload button
    m_pPlaybackFilesReload = m_pTrayMgr->createButton(TL_TOP,"ButtonReloadFiles","Reload",100);
    m_pTrayMgr->setTrayWidgetAlignment(TL_TOP, GHA_RIGHT);

    // Setup Video GUI stuff
    m_pTrayMgr->setTrayWidgetAlignment(TL_BOTTOMRIGHT, GHA_RIGHT);
    m_pCheckBoxVideo = m_pTrayMgr->createCheckBox(TL_BOTTOMRIGHT,"CheckBoxVideo","Drop Video",200);
    m_pTrayMgr->createSeparator(TL_BOTTOMRIGHT,"Sep1",500);
    m_pCheckBoxSimFile = m_pTrayMgr->createCheckBox(TL_BOTTOMRIGHT,"CheckBoxSimFile","Drop resampled SimFile",300);
    m_pCheckBoxSimFileInterpolate = m_pTrayMgr->createCheckBox(TL_BOTTOMRIGHT,"CheckBoxSimFileInterpolate","Drop SimFile: Interpolate",300);
    m_pTrayMgr->createSeparator(TL_BOTTOMRIGHT,"Sep2",500);
    m_pSliderFPS = m_pTrayMgr->createLongSlider(TL_BOTTOMRIGHT,"SliderFPS","Drop FPS",500,70,1,1000,1000);
    m_pSliderStartTime = m_pTrayMgr->createLongSlider(TL_BOTTOMRIGHT,"SliderStart","Start Time",500,70,0,5,10000);
    m_pSliderEndTime = m_pTrayMgr->createLongSlider(TL_BOTTOMRIGHT,"SliderEnd","End Time",500,70,0,10,10000);

    //Set Default values!
    m_pCheckBoxVideo->setChecked(false,false);
    m_pCheckBoxSimFile->setChecked(false,false);
    m_pCheckBoxSimFileInterpolate->setChecked(false,false);
    m_pSliderFPS->setValue(50,false);
    m_pSliderStartTime->setValue(0,false);
    m_pSliderEndTime->setValue(10,false);

    if(m_pPlaybackFiles->getNumItems()!=0) {
        m_pPlaybackFiles->selectItem(0);
    }

}
void PlaybackState::updatePlaybackPanel() {
    FileManager::getSingletonPtr()->updateFileList(SIMULATION_FOLDER_PATH,true);
    auto stringMap = FileManager::getSingletonPtr()->getSimFileNameList();

    Ogre::StringVector vec;
    for(auto it=stringMap.begin(); it!=stringMap.end(); it++) {
        vec.push_back(it->string());
    }

    m_pPlaybackFiles->setItems(vec);
}

void PlaybackState::checkBoxToggled(OgreBites::CheckBox * box) {
    if(box == m_pCheckBoxVideo) {
        if(box->isChecked()) {
            m_pPlaybackMgr->m_VideoDropSettings.m_bVideoDrop = true;
        } else {
            m_pPlaybackMgr->m_VideoDropSettings.m_bVideoDrop = false;
        }
    } else if(box== m_pCheckBoxSimFile) {
        if(box->isChecked()) {
            m_pPlaybackMgr->m_SimFileDropSettings.m_bSimFileDrop = true;
        } else {
            m_pPlaybackMgr->m_SimFileDropSettings.m_bSimFileDrop = false;
        }
    } else if(box == m_pCheckBoxSimFileInterpolate) {
        if(box->isChecked()) {
            m_pPlaybackMgr->m_SimFileDropSettings.m_bSimFileDropInterpolate = true;
        } else {
            m_pPlaybackMgr->m_SimFileDropSettings.m_bSimFileDropInterpolate = false;
        }
    }
}

void PlaybackState::sliderMoved(OgreBites::Slider * slider) {
    if(slider == m_pSliderFPS) {
        double fps = m_pSliderFPS->getValue();
        m_pPlaybackMgr->m_VideoDropSettings.m_FPS = fps;
        m_pPlaybackMgr->m_SimFileDropSettings.m_FPS = fps;
    } else if(slider == m_pSliderStartTime) {
        if(slider->getValue() >= m_pSliderEndTime->getValue()) {
            slider->setValue(m_pSliderEndTime->getValue()-0.001,false);
        }
        m_pPlaybackMgr->m_SimFileDropSettings.m_startTime = slider->getValue();
    } else if(slider == m_pSliderEndTime) {
        if(slider->getValue() <= m_pSliderStartTime->getValue()) {
            slider->setValue(m_pSliderStartTime->getValue()+0.001,false);
        }
        m_pPlaybackMgr->m_SimFileDropSettings.m_endTime = slider->getValue();
    }
}

void PlaybackState::buttonHit(OgreBites::Button * button) {
    if(button == m_pPlaybackFilesReload) {
        updatePlaybackPanel();
    }
}

void PlaybackState::itemSelected(OgreBites::SelectMenu * menu) {
    if(menu == m_pPlaybackFiles) {
        Ogre::DisplayString str = menu->getSelectedItem();
        std::string str_utf8 = str.asUTF8();
        FileManager::getSingletonPtr()->setPathSelectedSimFile(str_utf8);
        changeScene();
    } else if( menu == m_pActiveModeSelectMenu) {
        Ogre::DisplayString str = menu->getSelectedItem();
        if(str ==  Ogre::UTFString("Playback") ) {
            setSimulationMode(0);
        }
    }
}

void PlaybackState::changeScene() {

    m_pBaseNode->removeAndDestroyAllChildren();
    m_pSceneMgr->destroyAllManualObjects();
    m_pSceneMgr->destroyAllEntities();

    m_pAppLog->logMessage("Loading new Scene...");
    //Load new scene
    m_pPlaybackMgr = boost::shared_ptr<PlaybackManager > (new PlaybackManager(m_pSceneMgr)); // overwrite the existing, which gets deleted!

    if(m_pPlaybackMgr->setup()) {
        m_pAppLog->logMessage("Loading new scene successful");
    } else {
        m_pAppLog->logMessage("Loading new scene failed");
    }

    // Set Guis stuff, release callbacks!
    m_pCheckBoxVideo->setChecked(m_pCheckBoxVideo->isChecked(),true);
    m_pCheckBoxSimFile->setChecked(m_pCheckBoxSimFile->isChecked(),true);
    m_pSliderFPS->setValue(m_pSliderFPS->getValue(),true);
    m_pSliderEndTime->setValue(m_pSliderEndTime->getValue(),true);
    m_pSliderStartTime->setValue(m_pSliderStartTime->getValue(),true);
}

void PlaybackState::setSimulationMode(int i) {
    switch(i) {
    case 0:
        m_pAppLog->logMessage("PlaybackState:: Switched to PLAYBACK");
        m_eSimulationActiveMode = PLAYBACK;
        // Show file panel
        m_pPlaybackFiles->show();
        updateSceneFunction = boost::bind(&PlaybackState::updateScenePlayback,this,_1);
        break;
    }
}
void PlaybackState::switchSimulationMode() {
    m_eSimulationActiveMode = static_cast<ActiveMode>(((int)m_eSimulationActiveMode+1)%m_pActiveModeSelectMenu->getNumItems()) ;
    // Set active in Menu
    m_pActiveModeSelectMenu->selectItem((int)m_eSimulationActiveMode);
}

void PlaybackState::setupScene() {

    //World Axes
    Entity* ent = m_pSceneMgr->createEntity("WorldAxes", "axes.mesh");
    SceneNode* WorldAxes = m_pSceneMgr->getRootSceneNode()->createChildSceneNode("WorldAxes");
    WorldAxes->attachObject(ent);

    m_pBaseNode =  m_pSceneMgr->getRootSceneNode()->createChildSceneNode("BaseFrame");

    m_pOrbitCamera = boost::shared_ptr<OrbitCamera>(new OrbitCamera(m_pSceneMgr.get(),"PlaybackState::OrbitCam", 0.13, 150, 200, 0, M_PI/4));
    m_pOrbitCamera->enableInput();
    // Push attachable objects for Orbit camera to list
    m_pOrbitCamera->m_OrbitNodeList.push_back(WorldAxes);


    Ogre::Light* pLight = m_pSceneMgr->createLight("Light");
    pLight->setType(Ogre::Light::LT_DIRECTIONAL);
    pLight->setDirection(Ogre::Vector3(-1,-1,-1));
    pLight->setDiffuseColour((Ogre::Real)0.8,(Ogre::Real) 0.8,(Ogre::Real) 0.8);
    pLight->setPosition( 75, 75, 300 );
    //pLight->setAttenuation(5000000,0,0.1,0);
    //pLight->setCastShadows(true);

    pLight = m_pSceneMgr->createLight("Light2");
    pLight->setType(Ogre::Light::LT_DIRECTIONAL);
    pLight->setDirection(Ogre::Vector3(1,1,-1));
    pLight->setDiffuseColour((Ogre::Real)0.8,(Ogre::Real) 0.8,(Ogre::Real) 0.8);
    pLight->setPosition( -75, -75, 300 );
    pLight->setCastShadows(false);

    // Set Shadow Technique
    m_pSceneMgr->setAmbientLight(Ogre::ColourValue(0.7, 0.7, 0.7));
    m_pSceneMgr->setShadowTechnique(SHADOWTYPE_STENCIL_ADDITIVE);




    // Add a fancy skybox :-)
    //m_pSceneMgr->setSkyBox(true, "NoCloudsSkyBox",5000,true,Ogre::Quaternion(Ogre::Radian(M_PI/2),Ogre::Vector3(1,0,0)),ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
}


bool PlaybackState::keyPressed(const OIS::KeyEvent &keyEventRef) {
    switch(keyEventRef.key) {
    case OIS::KC_3: {
        toggleGUI();
    }

    case OIS::KC_2: {
        switchSimulationMode();
        return false;
    }
    case OIS::KC_1: {
        setMouseMode(true);
        return false;
    }
    case OIS::KC_R: {
        switchSceneDetailIndex();
        break;
    }
    case OIS::KC_ESCAPE: {
        m_pPlaybackMgr->stopPlaybackThread(true);
        this->popAppState();
        return false;
    }
    case OIS::KC_F1:
    case OIS::KC_0 : {
        switchToSimulationState();
        return false;
    }
    case OIS::KC_L: {
        //m_pTrayMgr->hideAll();
        m_pPlaybackMgr->startPlaybackThread();
        break;
    }
    case OIS::KC_K: {
        m_pPlaybackMgr->stopPlaybackThread(false);
        break;
    }
    case OIS::KC_U: {
        break;
    }
    case OIS::KC_I: {

        break;
    }
    default:
        break;
    }

    return true;
}


bool PlaybackState::keyReleased(const OIS::KeyEvent &keyEventRef) {
    return true;
}

void PlaybackState::setMouseMode(bool switchMode = false) {
    if(switchMode) {
        if(m_eMouseMode == MENU) {
            m_eMouseMode = CAMERA;
            std::cout << " Switched to Camera Mode"<<std::endl;
            m_pOrbitCamera->setActive();
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
            m_pOrbitCamera->setActive();
            m_pOrbitCamera->disableInput();
            m_pMenuMouse->setActive();

        } else {
            m_pOrbitCamera->setActive();
            m_pOrbitCamera->enableInput();
            m_pMenuMouse->setInactive();
        }
    }
}

void PlaybackState::switchSceneDetailIndex() {
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

void PlaybackState::switchToSimulationState() {
    boost::shared_ptr<RenderAppState> appState = this->findAppStateByName("SimulationState");
    if(appState) {
        this->pushAppState(appState);
    }
}

void PlaybackState::toggleGUI() {
    if(m_pTrayMgr->areTraysVisible()) {
        m_pTrayMgr->hideAll();
    } else {
        m_pTrayMgr->showAll();
    }
}


void PlaybackState::update(double timeSinceLastFrame) {


    // Update Camera!
    m_pOrbitCamera->update(timeSinceLastFrame);

    // HERE make a function wrapper which calls two different modes for realtime and playback
    updateSceneFunction(timeSinceLastFrame);
}


void PlaybackState::updateScenePlayback(double timeSinceLastFrame) {
    updateParamsPanelPlayback();
    m_pPlaybackMgr->updateScene(timeSinceLastFrame);
}

void PlaybackState::updateParamsPanelPlayback() {
    static std::stringstream GlobalTimeText, DynamicsThreadTimeText, SimTimeText, DelayText, TimeFactorText, AverageIterTimeText, MaxIterTimeText;
    static double Realtime, SimTime; // These variables are not acces by another thread!! only for the function updateTimerText !

    static int iterations = 0;
    iterations++;

    if(iterations%50==0) {

        SimTime = m_pPlaybackMgr->getSimulationTime();
        Realtime = m_pPlaybackMgr->getTimelineSimulation();

        GlobalTimeText.str("");
        DynamicsThreadTimeText.str("");
        SimTimeText.str("");
        DelayText.str("");
        TimeFactorText.str("");
        AverageIterTimeText.str("");
        MaxIterTimeText.str("");

        m_pPhysicsStatsValues.clear();
        GlobalTimeText << std::setw(10) <<std::fixed<< std::setprecision (4) << m_pTimelineRendering->getMilliseconds()*1e-3;
        DynamicsThreadTimeText << std::setprecision (4) << Realtime;
        SimTimeText <<std::fixed<< std::setprecision (4) << SimTime;
        if( m_pPlaybackMgr->isSimulationPaused()) {
            SimTimeText<<" (Paused)";
        }
        DelayText <<std::fixed<< std::setprecision (4) << Realtime-SimTime;
        AverageIterTimeText << std::fixed<< std::setprecision (4) <<0;
        MaxIterTimeText << std::fixed<< std::setprecision (4) << 0;
        TimeFactorText << m_pPlaybackMgr->getTimeScale();

        m_pPhysicsStatsValues.push_back(GlobalTimeText.str());
        m_pPhysicsStatsValues.push_back(DynamicsThreadTimeText.str());
        m_pPhysicsStatsValues.push_back(SimTimeText.str());
        m_pPhysicsStatsValues.push_back(DelayText.str());
        m_pPhysicsStatsValues.push_back(AverageIterTimeText.str());
        m_pPhysicsStatsValues.push_back(MaxIterTimeText.str());
        m_pPhysicsStatsValues.push_back(TimeFactorText.str());

        m_pPhysicsStatsPanel->setAllParamValues(m_pPhysicsStatsValues);

    }
}
