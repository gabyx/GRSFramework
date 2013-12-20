#ifndef SimulationManager_hpp
#define SimulationManager_hpp

#include <boost/shared_ptr.hpp>
#include <boost/timer/timer.hpp>
#include <boost/filesystem.hpp>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "SceneParser.hpp"


class DynamicsState;
class StateRecorder;
class SharedBufferDynSys;


class SimulationManager
{
public:

   DEFINE_CONFIG_TYPES

    SimulationManager();
   ~SimulationManager();

   boost::shared_ptr<SharedBufferDynSys >	    m_pSharedBuffer;
   boost::shared_ptr<StateRecorder >		    m_pStateRecorder;

   void setup();
   void setup(boost::filesystem::path sceneFilePath);

   boost::shared_ptr< SceneParser > m_pSceneParser;

   void startSim();

private:

   boost::timer::cpu_timer m_global_time;

   void writeAllOutput();
   RecorderSettings m_RecorderSettings;

   // Accessed only by thread ===================
   void threadRunRecord();
   bool initRecordThread();
   void cleanUpRecordThread();

   struct SettingsSimThread{
         double m_EndTime;
   } m_SettingsSimThread;


   Logging::Log *  m_pSimulationLog;

   boost::shared_ptr< TimeStepperType >	m_pTimestepper;

   boost::shared_ptr< DynamicsSystemType > m_pDynSys;
   // ===========================================

   int m_nSimBodies;

   // File Paths for one Simulation, always reset ==============================
   boost::filesystem::path m_SimFolderPath;
   boost::filesystem::path m_SimFilePath;
};


#endif // SIMULATIONMANAGERMAZE_HPP
