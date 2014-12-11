#ifndef GRSF_States_PlaybackState_hpp
#define GRSF_States_PlaybackState_hpp

/*=========================================================
	implementation specific includes
_________________________________________________________*/
#include <boost/thread.hpp>
#include <memory>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <OGRE/OgreSubEntity.h>
#include <OGRE/OgreMaterialManager.h>
#include <OIS/OISEvents.h>
#include <OIS/OISKeyboard.h>
#include <OIS/OISJoyStick.h>
#include <OIS/OISMouse.h>
#include <SdkTrays.h>

#include "GRSF/States/AppState.hpp"
#include "GRSF/States/AppStateManager.hpp"
#include "GRSF/States/SimulationManager/SimulationManagerGUI.hpp"
#include "GRSF/States/SimulationManager/PlaybackManager.hpp"
#include "GRSF/Singeltons/Contexts/InputContext.hpp"
#include "GRSF/Singeltons/Contexts/GuiContext.hpp"
#include "GRSF/App/OrbitCamera.hpp"
#include "GRSF/App/MenuMouse.hpp"
//=========================================================

/*=========================================================
	Class PlaybackState
_________________________________________________________*/
/**
* @ingroup	States
* @brief	Simulation State.
**/

class PlaybackState : public AppState, OgreBites::SdkTrayListener {
public:
    PlaybackState();
    ~PlaybackState();

    DECLARE_APPSTATE_CLASS(PlaybackState);

    void enter();
    void setupScene();
    void exit();
    bool pause();
    void resume();

    void moveCamera();
    void getInput();

    void update(double timeSinceLastFrame);

    std::shared_ptr<PlaybackManager>	   m_pPlaybackMgr;

private:

    Ogre::Log * m_pAppLog;

    void setupParamsPanel();

    Ogre::SceneNode * m_pBaseNode;                                       ///<Everythin in the Scene, gets deleted when scene reloads!
    void changeScene();

    std::shared_ptr<OrbitCamera>	m_pOrbitCamera;
    std::shared_ptr<MenuMouse>	   m_pMenuMouse;

    void switchToSimulationState();
    enum MouseMode { CAMERA = 0, MENU } m_eMouseMode;
    void setMouseMode(bool switchMode);


    void setupGUI(); // Setups the playback file list...
    void updatePlaybackPanel();// Updates the panel
    enum ActiveMode {PLAYBACK} m_eSimulationActiveMode;
    void setupActiveModeSelection();

    void updateParamsPanelPlayback();

    unsigned int m_SceneDetailIndex;
    void switchSceneDetailIndex();
    void toggleGUI();


    std::shared_ptr<Ogre::Timer>	m_pTimelineRendering;

    double m_lengthScale;

    std::shared_ptr<OgreBites::SdkTrayManager>	m_pTrayMgr;
    OgreBites::ParamsPanel*		m_pPhysicsStatsPanel;
    Ogre::StringVector			m_PhysicsStatsParams;
    Ogre::StringVector			m_pPhysicsStatsValues;

    OgreBites::SelectMenu* m_pActiveModeSelectMenu;
    OgreBites::SelectMenu* m_pPlaybackFiles;
    OgreBites::Button* m_pPlaybackFilesReload;
    OgreBites::CheckBox * m_pCheckBoxVideo;
    OgreBites::CheckBox * m_pCheckBoxSimFile;
    OgreBites::CheckBox * m_pCheckBoxSimFileInterpolate;
    OgreBites::Slider * m_pSliderFPS;
    OgreBites::Slider * m_pSliderStartTime;
    OgreBites::Slider * m_pSliderEndTime;

    void itemSelected(OgreBites::SelectMenu * menu); // Virtual of SdkTrayListner
    void checkBoxToggled(OgreBites::CheckBox * box); // Virtual of SdkTrayListener
    void sliderMoved(OgreBites::Slider * slider); // Virtual of SdkTrayListener
    void buttonHit(OgreBites::Button * button); // Virtual of SdkTrayListener

    bool keyPressed(const OIS::KeyEvent &keyEventRef);
    bool keyReleased(const OIS::KeyEvent &keyEventRef);

    bool mouseMoved(const OIS::MouseEvent &evt) {
        return true;
    };
    bool mousePressed(const OIS::MouseEvent &evt, OIS::MouseButtonID id) {
        return true;
    };
    bool mouseReleased(const OIS::MouseEvent &evt, OIS::MouseButtonID id) {
        return true;
    };

    void switchSimulationMode();
    void toggleMPIVisualization();
    void setSimulationMode(int i);
};
//=========================================================

#endif
