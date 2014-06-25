#ifndef SimulationManagerMPI_hpp
#define SimulationManagerMPI_hpp

#include <mpi.h>

#include <memory>
#include <boost/filesystem.hpp>

#include "LogDefines.hpp"
#include "TypeDefs.hpp"

#include "SceneParserMPI.hpp"

#include "MPIInformation.hpp"
#include "MPICommunication.hpp"
#include "MPITopologyBuilder.hpp"
#include "BodyCommunicator.hpp"

#include "CPUTimer.hpp"

class StateRecorder;
class StateRecorderBody;
class StateRecorderProcess;
class StateRecorderMPI;


class SimulationManagerMPI {
public:

    DEFINE_CONFIG_TYPES

    SimulationManagerMPI();
    ~SimulationManagerMPI();

    void setup();
    void setup(boost::filesystem::path sceneFilePath);


    void startSim();

private:

    CPUTimer m_globalTimer;

    void initSim();

    unsigned int m_nSimBodies, m_nGlobalSimBodies;

    RecorderSettings m_RecorderSettings;

    // File Paths for one Simulation, always reset ==============================
    boost::filesystem::path m_SimFolderPath;

    struct SettingsSimThread {
        double m_EndTime;
    } m_SettingsSimThread;


    Logging::Log *  m_pSimulationLog;

    //typedef StateRecorderBody<DynamicsSystemType> StateRecorderType;
    typedef StateRecorderMPI StateRecorderType;
    // ===============================================

    std::shared_ptr< StateRecorderType >  m_pStateRecorder;

    std::shared_ptr< SceneParserMPI >     m_pSceneParser;

    std::shared_ptr< TimeStepperType >	m_pTimestepper;
    std::shared_ptr< DynamicsSystemType > m_pDynSys;
    std::shared_ptr< BodyCommunicator >   m_pBodyCommunicator;


    typedef typename MPILayer::ProcessCommunicator ProcessCommunicatorType;
    typedef typename ProcessCommunicatorType::ProcessInfoType ProcessInfoType;
    std::shared_ptr< ProcessCommunicatorType > m_pProcCommunicator;

    typedef typename MPILayer::TopologyBuilder  TopologyBuilderType;
    std::shared_ptr< TopologyBuilderType >    m_pTopologyBuilder;

    void writeAllOutput();

    void checkNumberOfBodiesInProcess();
    void getMaxRuntime(PREC runtime);
};


// Implementation
#include "SimulationManagerMPI.icc"


#endif // SimulationManagerMPI_HPP
