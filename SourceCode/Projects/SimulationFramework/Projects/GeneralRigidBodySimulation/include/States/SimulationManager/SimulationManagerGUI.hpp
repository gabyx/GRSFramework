#ifndef SimulationManagerGUI_hpp
#define SimulationManagerGUI_hpp

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

#include "SceneParserOgre.hpp"

template <typename TLayoutConfig> class DynamicsState;
template <typename TLayoutConfig> class StateRecorder;
template< typename TLayoutConfig> class SharedBufferDynSys;

template<typename TConfig>
class SimulationManagerGUI : public SimulationManagerBase , public OIS::KeyListener
{
public:

   DEFINE_CONFIG_TYPES_OF(TConfig)

    SimulationManagerGUI(boost::shared_ptr<Ogre::SceneManager> pSceneMgr);
   ~SimulationManagerGUI();


   boost::shared_ptr<SharedBufferDynSys<LayoutConfigType> >	    m_pSharedBuffer;
   boost::shared_ptr<StateRecorder<LayoutConfigType> >		    m_pStateRecorder;

   std::vector<Ogre::SceneNode*>	m_SceneNodeSimBodies;
   std::vector<Ogre::SceneNode*>	m_SceneNodeBodies;

   void setup();

   boost::shared_ptr< SceneParserOgre<TConfig> > m_pSceneParser;


   void updateScene(double timeSinceLastFrame);
   void updateSimBodies();
   void initBeforeThreads();

   double getSimulationTime(); // used to access the current simulation state time, from the AppState

   void startSimThread(Threads threadToStop);
   void stopSimThread(Threads threadToStop, bool force_stop);

   // Key Mouse Listener

   bool keyPressed(const OIS::KeyEvent &keyEventRef);
   bool keyReleased(const OIS::KeyEvent &keyEventRef);
   void enableInput(bool value);

private:

   boost::shared_ptr<Ogre::SceneManager>	m_pSceneMgr;

   void setShadowTechniques();

   boost::shared_ptr<const DynamicsState<LayoutConfigType> > m_pVisBuffer;

   // Accessed only by thread ===================
   void threadRunSimulation();
   void initSimThread();
   void cleanUpSimThread();

   void threadRunRecord();
   bool initRecordThread();
   void cleanUpRecordThread();
   struct SettingsSimThread{
         double m_EndTime;
   } m_SettingsSimThread;

   void readSharedBuffer();
   void writeSharedBuffer();

   std::string m_KeyListenerName;

   Logging::Log *  m_pSimulationLog;

   boost::shared_ptr< TimeStepperType >	m_pTimestepper;

   boost::shared_ptr< DynamicsSystemType >		   m_pDynSys;
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
#include "SimulationManagerGUI.icc"


#endif // SIMULATIONMANAGERMAZE_HPP
