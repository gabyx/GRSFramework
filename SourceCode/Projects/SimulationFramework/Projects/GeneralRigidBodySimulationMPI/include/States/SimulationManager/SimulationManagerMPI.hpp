#ifndef SimulationManagerMPI_hpp
#define SimulationManagerMPI_hpp

#include <mpi.h>

#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>

#include "LogDefines.hpp"
#include "TypeDefs.hpp"

#include "SceneParserMPI.hpp"

#include "MPIInformation.hpp"
#include "MPICommunication.hpp"

#include "NeighbourCommunicator.hpp"

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

    boost::shared_ptr< StateRecorderType >   m_pStateRecorder;

    boost::shared_ptr< SceneParserMPI > m_pSceneParser;

    boost::shared_ptr< TimeStepperType >	m_pTimestepper;
    boost::shared_ptr< DynamicsSystemType > m_pDynSys;
    boost::shared_ptr< NeighbourCommunicator > m_pNbCommunicator;

    typedef typename MPILayer::ProcessCommunicator::ProcessInfoType ProcessInfoType;
    boost::shared_ptr< MPILayer::ProcessCommunicator > m_pProcCommunicator;

    void writeAllOutput();

    void checkNumberOfBodiesInProcess();
    void getMaxRuntime(PREC runtime);
};


// Implementation
#include "SimulationManagerMPI.icc"


#endif // SimulationManagerMPI_HPP
