#ifndef SimulationManager_hpp
#define SimulationManager_hpp

#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>

#include "LogDefines.hpp"
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

    SimulationManager();
   ~SimulationManager();

   boost::shared_ptr<SharedBufferDynSys<LayoutConfigType> >	    m_pSharedBuffer;
   boost::shared_ptr<StateRecorder<LayoutConfigType> >		    m_pStateRecorder;

   void setup();
   void setup(boost::filesystem::path sceneFilePath);

   boost::shared_ptr< SceneParser<TConfig> > m_pSceneParser;

   void startSimThread();
   void waitForSimThread();

   void stopSimThread();

private:

   // Accessed only by thread ===================
   void threadRunRecord();
   void initRecordThread();
   void cleanUpRecordThread();

   struct SettingsSimThread{
         double m_EndTime;
   } m_SettingsSimThread;

   void readSharedBuffer();
   void writeSharedBuffer();

   std::string m_KeyListenerName;

   Logging::Log *  m_pSimulationLog;

   boost::shared_ptr< TimeStepperType >	m_pTimestepper;

   boost::shared_ptr< DynamicsSystemType > m_pDynSys;
   // ===========================================

   int m_nSimBodies;

   // File Paths for one Simulation, always reset ==============================
   boost::filesystem::path m_SimFolderPath;
   boost::filesystem::path m_SimFilePath;
};


// Implementation
#include "SimulationManager.icc"


#endif // SIMULATIONMANAGERMAZE_HPP
