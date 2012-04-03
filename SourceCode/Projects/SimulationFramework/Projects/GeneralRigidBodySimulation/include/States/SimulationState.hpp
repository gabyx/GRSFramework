#ifndef SimulationState_hpp
#define SimulationState_hpp

/*=========================================================
	implementation specific includes
_________________________________________________________*/
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <OGRE/OgreSubEntity.h>
#include <OGRE/OgreMaterialManager.h>
#include <OIS/OISEvents.h>
#include <OIS/OISKeyboard.h>
#include <OIS/OISJoyStick.h>
#include <OIS/OISMouse.h>
#include <SdkTrays.h>

#include "AppState.hpp"
#include "AppStateManager.hpp"
#include "SimulationManager.hpp"
#include "InputContext.hpp"
#include "GuiContext.hpp"
#include "OrbitCamera.hpp"
#include "MenuMouse.hpp"
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

	bool keyPressed(const OIS::KeyEvent &keyEventRef);
	bool keyReleased(const OIS::KeyEvent &keyEventRef);

	bool mouseMoved(const OIS::MouseEvent &evt){return true;};
	bool mousePressed(const OIS::MouseEvent &evt, OIS::MouseButtonID id){return true;};
	bool mouseReleased(const OIS::MouseEvent &evt, OIS::MouseButtonID id){return true;};

	void buttonHit(OgreBites::Button* button){};

	void update(double timeSinceLastFrame);

  boost::function<void (double)> updateSceneFunction; // this function binds to updateScenePlayback() or  updateSceneRealtime()
  void updateSceneRealtime(double timeSinceLastFrame);

   boost::shared_ptr<SimulationManager<GeneralConfig> >	m_pSimMgr;

private:

  Ogre::Log * m_pAppLog;

	void setupParamsPanel();

	Ogre::SceneNode * m_pBaseNode;

	boost::shared_ptr<OrbitCamera>	m_pOrbitCamera;
	boost::shared_ptr<MenuMouse>	m_pMenuMouse;

   void switchToPlaybackState();

  enum MouseMode{ CAMERA = 0, MENU } m_eMouseMode;
  void setMouseMode(bool switchMode);
  enum ActiveMode{ REALTIME = 0 , RECORD} m_eSimulationActiveMode;
  void setupActiveModeSelection();
  // In each mode you can start a thread, realtime thread, record thread
  // Realtime and Recorder Thread start only one thread!
  // You cannot start Realtime thread, and a record thread at the same time

  void updateParamsPanelSimulation();

  unsigned int m_SceneDetailIndex;
  void switchSceneDetailIndex();

	boost::shared_ptr<Ogre::Timer>	m_pTimelineRendering;

	double m_lengthScale;

	boost::shared_ptr<OgreBites::SdkTrayManager>	m_pTrayMgr;
	OgreBites::ParamsPanel*		m_pPhysicsStatsPanel;
	Ogre::StringVector			m_PhysicsStatsParams;
	Ogre::StringVector			m_pPhysicsStatsValues;

  OgreBites::SelectMenu* m_pActiveModeSelectMenu;
  void itemSelected(OgreBites::SelectMenu * menu); // Virtual of SdkTrayListner
  void switchSimulationMode();
  void setSimulationMode(int i);
};
//=========================================================

#endif
