#ifndef SimulationManagerMPI_hpp
#define SimulationManagerMPI_hpp

#include <mpi.h>

#include <boost/timer/timer.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>

#include "LogDefines.hpp"
#include "TypeDefs.hpp"

#include "SceneParserMPI.hpp"

#include "MPIInformation.hpp"
#include "MPICommunication.hpp"
#include "MPITopologyBuilder.hpp"
#include "BodyCommunicator.hpp"

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

    boost::timer::cpu_timer m_globalTimer;

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

    boost::shared_ptr< StateRecorderType >  m_pStateRecorder;

    boost::shared_ptr< SceneParserMPI >     m_pSceneParser;

    boost::shared_ptr< TimeStepperType >	m_pTimestepper;
    boost::shared_ptr< DynamicsSystemType > m_pDynSys;
    boost::shared_ptr< BodyCommunicator >   m_pBodyCommunicator;


    typedef typename MPILayer::ProcessCommunicator ProcessCommunicatorType;
    typedef typename ProcessCommunicatorType::ProcessInfoType ProcessInfoType;
    boost::shared_ptr< ProcessCommunicatorType > m_pProcCommunicator;

    typedef typename MPILayer::TopologyBuilder  TopologyBuilderType;
    boost::shared_ptr< TopologyBuilderType >    m_pTopologyBuilder;

    void writeAllOutput();

    void checkNumberOfBodiesInProcess();
    void getMaxRuntime(PREC runtime);
};


// Implementation
#include "SimulationManagerMPI.icc"


#endif // SimulationManagerMPI_HPP
