// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_states_SimulationState_hpp
#define GRSF_states_SimulationState_hpp

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
#include "GRSF/states/simulationManager/SimulationManagerGUI.hpp"
//=========================================================

/*=========================================================
    Class SimulationState
_________________________________________________________*/
/**
* @ingroup	States
* @brief	Simulation State.
**/

class SimulationState : public AppState, OgreBites::SdkTrayListener
{
public:
    SimulationState();
    ~SimulationState();

    DECLARE_APPSTATE_CLASS(SimulationState);

    void enter();
    void setupScene();
    void exit();
    bool pause();
    void resume();

    void moveCamera();
    void getInput();

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

    void buttonHit(OgreBites::Button* button){};

    void update(double timeSinceLastFrame);

    boost::function<void(double)>
         updateSceneFunction;  // this function binds to updateScenePlayback() or  updateSceneRealtime()
    void updateSceneRealtime(double timeSinceLastFrame);

    std::shared_ptr<SimulationManagerGUI> m_pSimMgr;

private:
    Ogre::Log* m_pAppLog;

    void setupParamsPanel();

    Ogre::SceneNode* m_pBaseNode;

    std::shared_ptr<OrbitCamera> m_pOrbitCamera;
    std::shared_ptr<MenuMouse>   m_pMenuMouse;

    void switchToPlaybackState();

    enum MouseMode
    {
        CAMERA = 0,
        MENU
    } m_eMouseMode;
    void setMouseMode(bool switchMode);
    enum ActiveMode
    {
        REALTIME = 0,
        RECORD
    } m_eSimulationActiveMode;
    void setupActiveModeSelection();
    // In each mode you can start a thread, realtime thread, record thread
    // Realtime and Recorder Thread start only one thread!
    // You cannot start Realtime thread, and a record thread at the same time

    void updateParamsPanelSimulation();

    unsigned int m_SceneDetailIndex;
    void         switchSceneDetailIndex();
    void         toggleMPIVisualization();

    std::shared_ptr<Ogre::Timer> m_pTimelineRendering;

    double m_lengthScale;

    std::shared_ptr<OgreBites::SdkTrayManager> m_pTrayMgr;
    OgreBites::ParamsPanel*                    m_pPhysicsStatsPanel;
    Ogre::StringVector                         m_PhysicsStatsParams;
    Ogre::StringVector                         m_pPhysicsStatsValues;

    OgreBites::SelectMenu* m_pActiveModeSelectMenu;
    void itemSelected(OgreBites::SelectMenu* menu);  // Virtual of SdkTrayListner
    void switchSimulationMode();
    void setSimulationMode(int i);
};
//=========================================================

#endif
