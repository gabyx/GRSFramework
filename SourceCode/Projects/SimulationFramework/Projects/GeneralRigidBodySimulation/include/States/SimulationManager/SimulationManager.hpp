#ifndef SimulationManager_hpp
#define SimulationManager_hpp

#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include <Eigen/Dense>
#include <OIS/OISEvents.h>
#include <OIS/OISKeyboard.h>
#include <OIS/OISJoyStick.h>
#include <OIS/OISMouse.h>



#include "TypeDefs.hpp"

#include "SimulationManagerBase.hpp"
#include "InputContext.hpp"

#include "SceneParser.hpp"

template <typename TLayoutConfig> class DynamicsState;
template <typename TLayoutConfig> class StateRecorder;
template< typename TLayoutConfig> class SharedBufferDynSys;

template<typename TConfig>
class SimulationManager : public SimulationManagerBase
{
public:

   DEFINE_CONFIG_TYPES_OF(TConfig)

    SimulationManager(boost::shared_ptr<Ogre::SceneManager> pSceneMgr);
   ~SimulationManager();


   boost::shared_ptr<SharedBufferDynSys<TLayoutConfig> >	    m_pSharedBuffer;
   boost::shared_ptr<StateRecorder<TLayoutConfig> >		    m_pStateRecorder;

   std::vector<Ogre::SceneNode*>	m_SceneNodeSimBodies;
   std::vector<Ogre::SceneNode*>	m_SceneNodeBodies;
   
   void setup();

   boost::shared_ptr< SceneParser<TConfig> > m_pSceneParser;


   void updateScene(double timeSinceLastFrame);
   void updateSimBodies();
   void initBeforeThreads();

   double getSimulationTime(); // used to access the current simulation state time, from the AppState

   void startSimThread(Threads threadToStop);
   void stopSimThread(Threads threadToStop, bool force_stop);

   // Key Mouse Listener
   bool keyPressed(const OIS::KeyEvent &keyEventRef);
   bool keyReleased(const OIS::KeyEvent &keyEventRef);

private:

   void setShadowTechniques();

   boost::shared_ptr<const DynamicsState<TLayoutConfig> > m_pVisBuffer;

   // Accessed only by thread ===================
   void threadRunSimulation();
   void initSimThread();
   void cleanUpSimThread();

   void threadRunRecord();
   void initRecordThread();
   void cleanUpRecordThread();
   struct SettingsSimThread{
         double m_EndTime;
   } m_SettingsSimThread;

   void readSharedBuffer();
   void writeSharedBuffer();


   Ogre::Log* m_pSolverLog;

   boost::shared_ptr< TTimeStepper >	m_pTimestepper;

   boost::shared_ptr< TSystem >		   m_pDynSys;
   // ===========================================

   int m_nSimBodies;
   double m_lengthScale;
   Ogre::SceneNode * m_pBaseNode;

   bool writeInitialState();

   // Timming Variables for updateScene =====
   bool m_bFirstPass;

   double m_timeFrameToFrameAvgNoWorkload;
   unsigned int m_counterNoWorkload;

   double m_lastTimeWhenChanged;
   double m_newTimeWhenChanged;
   double m_deltaTimeWhenChanged;

   double m_stopTimeAfterUpdate;

   double m_diffUpdateAndChangedTime;
   int m_passCounter;
   // =========================================


   // File Paths for one Simulation, always reset ==============================
   boost::filesystem::path m_SimFolderPath;
   boost::filesystem::path m_SimFilePath;
};


// Implementation
#include "SimulationManager.icc"


#endif // SIMULATIONMANAGERMAZE_HPP
