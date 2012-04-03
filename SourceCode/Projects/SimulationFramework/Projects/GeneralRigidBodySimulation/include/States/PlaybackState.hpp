﻿#ifndef PlaybackState_hpp
#define PlaybackState_hpp

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
#include "PlaybackManager.hpp"
#include "InputContext.hpp"
#include "GuiContext.hpp"
#include "OrbitCamera.hpp"
#include "MenuMouse.hpp"
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

  boost::function<void (double)> updateSceneFunction; // this function binds to updateScenePlayback() or  updateSceneRealtime()
  void updateScenePlayback(double timeSinceLastFrame);

   boost::shared_ptr<PlaybackManager<GeneralConfig> >	   m_pPlaybackMgr;

private:

  Ogre::Log * m_pAppLog;

	void setupParamsPanel();

	Ogre::SceneNode * m_pBaseNode;                                       ///<Everythin in the Scene, gets deleted when scene reloads!
   void changeScene();

	boost::shared_ptr<OrbitCamera>	m_pOrbitCamera;
	boost::shared_ptr<MenuMouse>	   m_pMenuMouse;

  void switchToSimulationState();
  enum MouseMode{ CAMERA = 0, MENU } m_eMouseMode;
  void setMouseMode(bool switchMode);

  
  void setupGUI(); // Setups the playback file list...
  void updatePlaybackPanel();// Updates the panel
  enum ActiveMode{PLAYBACK} m_eSimulationActiveMode;
  void setupActiveModeSelection();

  void updateParamsPanelPlayback();

  unsigned int m_SceneDetailIndex;
  void switchSceneDetailIndex();
  void toggleGUI();


	boost::shared_ptr<Ogre::Timer>	m_pTimelineRendering;

	double m_lengthScale;

	boost::shared_ptr<OgreBites::SdkTrayManager>	m_pTrayMgr;
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

	bool mouseMoved(const OIS::MouseEvent &evt){return true;};
	bool mousePressed(const OIS::MouseEvent &evt, OIS::MouseButtonID id){return true;};
	bool mouseReleased(const OIS::MouseEvent &evt, OIS::MouseButtonID id){return true;};

  void switchSimulationMode();
  void setSimulationMode(int i);
};
//=========================================================

#endif
