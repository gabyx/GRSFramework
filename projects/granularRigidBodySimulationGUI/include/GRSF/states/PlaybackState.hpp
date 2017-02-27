// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_states_PlaybackState_hpp
#define GRSF_states_PlaybackState_hpp

/*=========================================================
    implementation specific includes
_________________________________________________________*/
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <memory>

#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSubEntity.h>
#include <OIS/OISEvents.h>
#include <OIS/OISJoyStick.h>
#include <OIS/OISKeyboard.h>
#include <OIS/OISMouse.h>
#include <SdkTrays.h>

#include "GRSF/app/MenuMouse.hpp"
#include "GRSF/app/OrbitCamera.hpp"
#include "GRSF/singeltons/contexts/GuiContext.hpp"
#include "GRSF/singeltons/contexts/InputContext.hpp"
#include "GRSF/states/AppState.hpp"
#include "GRSF/states/AppStateManager.hpp"
#include "GRSF/states/simulationManager/PlaybackManager.hpp"
#include "GRSF/states/simulationManager/SimulationManagerGUI.hpp"
//=========================================================

/*=========================================================
    Class PlaybackState
_________________________________________________________*/
/**
* @ingroup	States
* @brief	Simulation State.
**/

class PlaybackState : public AppState, OgreBites::SdkTrayListener
{
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

    std::shared_ptr<PlaybackManager> m_pPlaybackMgr;

    private:
    Ogre::Log* m_pAppLog;

    void setupParamsPanel();

    Ogre::SceneNode* m_pBaseNode;  ///<Everythin in the Scene, gets deleted when scene reloads!
    void             changeScene();

    std::shared_ptr<OrbitCamera> m_pOrbitCamera;
    std::shared_ptr<MenuMouse>   m_pMenuMouse;

    void switchToSimulationState();
    enum MouseMode
    {
        CAMERA = 0,
        MENU
    } m_eMouseMode;
    void setMouseMode(bool switchMode);

    void setupGUI();             // Setups the playback file list...
    void updatePlaybackPanel();  // Updates the panel
    enum ActiveMode
    {
        PLAYBACK
    } m_eSimulationActiveMode;
    void setupActiveModeSelection();

    void updateParamsPanelPlayback();

    unsigned int m_SceneDetailIndex;
    void         switchSceneDetailIndex();
    void         toggleGUI();

    std::shared_ptr<Ogre::Timer> m_pTimelineRendering;

    double m_lengthScale;

    std::shared_ptr<OgreBites::SdkTrayManager> m_pTrayMgr;
    OgreBites::ParamsPanel*                    m_pPhysicsStatsPanel;
    Ogre::StringVector                         m_PhysicsStatsParams;
    Ogre::StringVector                         m_pPhysicsStatsValues;

    OgreBites::SelectMenu* m_pActiveModeSelectMenu;
    OgreBites::SelectMenu* m_pPlaybackFiles;
    OgreBites::Button*     m_pPlaybackFilesReload;
    OgreBites::CheckBox*   m_pCheckBoxVideo;
    OgreBites::CheckBox*   m_pCheckBoxSimFile;
    OgreBites::CheckBox*   m_pCheckBoxSimFileInterpolate;
    OgreBites::Slider*     m_pSliderFPS;
    OgreBites::Slider*     m_pSliderStartTime;
    OgreBites::Slider*     m_pSliderEndTime;

    void itemSelected(OgreBites::SelectMenu* menu);  // Virtual of SdkTrayListner
    void checkBoxToggled(OgreBites::CheckBox* box);  // Virtual of SdkTrayListener
    void sliderMoved(OgreBites::Slider* slider);     // Virtual of SdkTrayListener
    void buttonHit(OgreBites::Button* button);       // Virtual of SdkTrayListener

    bool keyPressed(const OIS::KeyEvent& keyEventRef);
    bool keyReleased(const OIS::KeyEvent& keyEventRef);

    bool mouseMoved(const OIS::MouseEvent& evt)
    {
        return true;
    };
    bool mousePressed(const OIS::MouseEvent& evt, OIS::MouseButtonID id)
    {
        return true;
    };
    bool mouseReleased(const OIS::MouseEvent& evt, OIS::MouseButtonID id)
    {
        return true;
    };

    void switchSimulationMode();
    void toggleMPIVisualization();
    void setSimulationMode(int i);
};
//=========================================================

#endif
