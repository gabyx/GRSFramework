﻿#ifndef PLAYBACK_MANAGER_HPP
#define PLAYBACK_MANAGER_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <OIS/OISEvents.h>
#include <OIS/OISKeyboard.h>
#include <OIS/OISJoyStick.h>
#include <OIS/OISMouse.h>

#include "TypeDefs.hpp"
#include "PlaybackLoader.hpp"
#include "PlaybackManagerBase.hpp"
#include "InputContext.hpp"
#include "DynamicsState.hpp"

#include "VideoDropper.hpp"
#include "StateRecorderResampler.hpp"

template< typename TLayoutConfig > class StateRingPoolVisBackFront;
template< typename TLayoutConfig , typename TStatePool> class PlaybackLoader;
template< typename TLayoutConfig> class SharedBufferPlayback;

template<typename TConfig>
class PlaybackManager : public PlaybackManagerBase
{
public:
  DEFINE_CONFIG_TYPES_OF(TConfig);

	PlaybackManager(boost::shared_ptr<Ogre::SceneManager> pSceneMgr);
	~PlaybackManager();

  boost::shared_ptr<SharedBufferPlayback<TLayoutConfig> >	m_pSharedBuffer;

  boost::shared_ptr< SceneParser<TConfig> > m_pSceneParser;

  boost::shared_ptr< VideoDropper > m_pVideoDropper;
  struct VideoDropSettings{ bool m_bVideoDrop; double m_FPS;} m_VideoDropSettings;

  boost::shared_ptr< StateRecorderResampler<TLayoutConfig> > m_pStateRecorderResampler;
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

private:

  
  bool parseScene();
  bool m_bSetupSuccessful;

  boost::shared_ptr<const DynamicsState<TLayoutConfig> > m_pVisBuffer;

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
  boost::shared_ptr< PlaybackLoader<TLayoutConfig, StateRingPoolVisBackFront<TLayoutConfig> > > m_pFileLoader;


  const unsigned int m_nDofuObj, m_nDofqObj; // These are the dimensions for one Obj
  unsigned int m_nSimBodies;
  double m_lengthScale;
  Ogre::SceneNode * m_pBaseNode;

 
};


//Implementation
#include "PlaybackManager.icc"


#endif // SIMULATIONMANAGERMAZE_HPP
