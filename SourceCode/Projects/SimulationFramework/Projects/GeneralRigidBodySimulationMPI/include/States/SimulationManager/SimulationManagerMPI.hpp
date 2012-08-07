#ifndef SimulationManagerMPI_hpp
#define SimulationManagerMPI_hpp

#include <mpi.h>

#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>

#include "LogDefines.hpp"
#include "TypeDefs.hpp"

#include "StateRecorderBody.hpp"
#include "SceneParserMPI.hpp"

#include "MPIInformation.hpp"


template <typename TLayoutConfig> class StateRecorder;


template<typename TConfig>
class SimulationManagerMPI {
public:

    DEFINE_CONFIG_TYPES_OF(TConfig)

    SimulationManagerMPI();
    ~SimulationManagerMPI();

    boost::shared_ptr<StateRecorderBody<DynamicsSystemType> >		    m_pStateRecorder;

    void setup();
    void setup(boost::filesystem::path sceneFilePath);


    void startSim();

    boost::shared_ptr< SceneParser<TConfig> > m_pSceneParser;

private:

    void initSim();

    unsigned int m_nSimBodies, m_nGlobalSimBodies;

    // Accessed only by thread ===================

    struct SettingsSimThread {
        double m_EndTime;
    } m_SettingsSimThread;


    Logging::Log *  m_pSimulationLog;

    boost::shared_ptr< TimeStepperType >	m_pTimestepper;

    boost::shared_ptr< DynamicsSystemType > m_pDynSys;
    // ===========================================

    // File Paths for one Simulation, always reset ==============================
    boost::filesystem::path m_SimFolderPath;

    typedef MPILayer::ProcessInformation<LayoutConfigType> ProcessInfoType;
    ProcessInfoType m_MPIProcInfo;


    bool checkNumberOfBodiesInProcess();

};


// Implementation
#include "SimulationManagerMPI.icc"


#endif // SimulationManagerMPI_HPP
