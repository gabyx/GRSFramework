#ifndef SimulationManagerMPI_hpp
#define SimulationManagerMPI_hpp

#include <mpi.h>

#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>

#include "LogDefines.hpp"
#include "TypeDefs.hpp"

#include "SceneParser.hpp"

template <typename TLayoutConfig> class DynamicsState;
template <typename TLayoutConfig> class StateRecorder;
template< typename TLayoutConfig> class SharedBufferDynSys;



template<typename TConfig>
class SimulationManagerMPI
{
public:

   DEFINE_CONFIG_TYPES_OF(TConfig)

    SimulationManagerMPI();
   ~SimulationManagerMPI();

   boost::shared_ptr<SharedBufferDynSys<LayoutConfigType> >	    m_pSharedBuffer;
   boost::shared_ptr<StateRecorder<LayoutConfigType> >		    m_pStateRecorder;

   void setup();
   void setup(boost::filesystem::path sceneFilePath);

   boost::shared_ptr< SceneParser<TConfig> > m_pSceneParser;

private:

    unsigned int m_nSimBodies;

   // Accessed only by thread ===================

   struct SettingsSimThread{
         double m_EndTime;
   } m_SettingsSimThread;


   Logging::Log *  m_pSimulationLog;

   boost::shared_ptr< TimeStepperType >	m_pTimestepper;

   boost::shared_ptr< DynamicsSystemType > m_pDynSys;
   // ===========================================

   // File Paths for one Simulation, always reset ==============================
   boost::filesystem::path m_SimFolderPath;
   boost::filesystem::path m_SimFilePath;
};


// Implementation
#include "SimulationManagerMPI.icc"


#endif // SimulationManagerMPIMAZE_HPP
