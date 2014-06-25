﻿#ifndef PLAYBACK_MANAGER_HPP
#define PLAYBACK_MANAGER_HPP

#include <memory>
#include <OIS/OISEvents.h>
#include <OIS/OISKeyboard.h>
#include <OIS/OISJoyStick.h>
#include <OIS/OISMouse.h>

#include "TypeDefs.hpp"

#include "SceneParserOgre.hpp"

#include "PlaybackLoader.hpp"
#include "PlaybackManagerBase.hpp"
#include "InputContext.hpp"
#include "DynamicsState.hpp"

#include "VideoDropper.hpp"
#include "StateRecorderResampler.hpp"

class StateRingPoolVisBackFront;
template< typename TStatePool> class PlaybackLoader;
class SharedBufferPlayback;


class PlaybackManager : public PlaybackManagerBase , public OIS::KeyListener
{
public:

  DEFINE_CONFIG_TYPES

	PlaybackManager(std::shared_ptr<Ogre::SceneManager> pSceneMgr);
	~PlaybackManager();

  std::shared_ptr<SharedBufferPlayback >	m_pSharedBuffer;

  std::shared_ptr< SceneParserOgre > m_pSceneParser;

  std::shared_ptr< VideoDropper > m_pVideoDropper;
  struct VideoDropSettings{ bool m_bVideoDrop; double m_FPS;} m_VideoDropSettings;

  std::shared_ptr< StateRecorderResampler > m_pStateRecorderResampler;
  struct SimFileDropSettings{ bool m_bSimFileDrop; double m_FPS; bool m_bSimFileDropInterpolate; double m_startTime; double m_endTime;} m_SimFileDropSettings;

  bool setup();

  // Accessed by grafic thread  ===============
   void updateScene(double timeSinceLastFrame);
	double getSimulationTime(); // used to access the current simulation state time, from the AppState

	std::vector<Ogre::SceneNode*>	m_SceneNodeSimBodies;
   std::vector<Ogre::SceneNode*>	m_SceneNodeBodies;

   void startPlaybackThread();
   void stopPlaybackThread(bool force_stop);

	// Key Mouse Listener
	bool keyPressed(const OIS::KeyEvent &keyEventRef);
	bool keyReleased(const OIS::KeyEvent &keyEventRef);
    void enableInput(bool value);

private:

   Logging::Log*	m_pSimulationLog;
   Logging::Log*	m_pThreadLog;

   std::shared_ptr<Ogre::SceneManager>	m_pSceneMgr;

  std::string m_KeyListenerName;

  bool parseScene();
  bool m_bSetupSuccessful;

  std::shared_ptr<const DynamicsState > m_pVisBuffer;

  //Accessed only by Graphic thread ==========
   void updateSimBodies();
   bool writeInitialState();
   void initBeforeThreads();
   void cancelAllWaits();
   struct SettingsVisThread{
         bool m_bFirstPass;
         bool m_bVideoDrop;
   } m_SettingsVisThread;

  // =========================================


  //Accessed only by Sim thread ==============
  	void threadRunSimulation();
   void initSimThread();
   void cleanUpSimThread();
   struct SettingsSimThread{
         bool m_bSimFileDrop;
         bool m_bSimFileDropInterpolate;
         bool m_bVideoDrop;
   } m_SettingsSimThread;
   // =========================================

  // Accessed only by Loader Thread
  std::shared_ptr< PlaybackLoader<StateRingPoolVisBackFront > > m_pFileLoader;


  const unsigned int m_nDofuBody, m_nDofqBody; // These are the dimensions for one Obj
  unsigned int m_nSimBodies;
  double m_lengthScale;
  Ogre::SceneNode * m_pBaseNode;

};


#endif // SIMULATIONMANAGERMAZE_HPP
