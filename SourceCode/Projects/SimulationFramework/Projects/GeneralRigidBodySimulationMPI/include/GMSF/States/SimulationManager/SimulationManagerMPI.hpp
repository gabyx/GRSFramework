﻿#ifndef GMSF_States_SimulationManager_SimulationManagerMPI_hpp
#define GMSF_States_SimulationManager_SimulationManagerMPI_hpp

#include <mpi.h>

#include <memory>
#include <boost/filesystem.hpp>

#include "GMSF/Common/LogDefines.hpp"
#include "GMSF/Common/TypeDefs.hpp"

#include "GMSF/Dynamics/General/MPIInformation.hpp"
#include "GMSF/Dynamics/General/MPICommunication.hpp"
#include "GMSF/Dynamics/General/MPITopologyBuilder.hpp"
#include "GMSF/Dynamics/General/BodyCommunicator.hpp"

#include "GMSF/Common/CPUTimer.hpp"

class StateRecorder;
class StateRecorderBody;
class StateRecorderProcess;
class StateRecorderMPI;


class SimulationManagerMPI {
public:

    DEFINE_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    SimulationManagerMPI();
    ~SimulationManagerMPI();

    void setup();
    void setup(boost::filesystem::path sceneFilePath);


    void startSim();

private:

    CPUTimer m_globalTimer;

    void initSim();
    void reloadScene();

    unsigned int m_nGlobalSimBodies;

    RecorderSettings m_RecorderSettings;

    // File Paths for one Simulation, always reset ==============================
    boost::filesystem::path m_SimFolderPath;

    struct SettingsSimThread {
        double m_EndTime;
    } m_settingsSimThread;


    Logging::Log *  m_pSimulationLog;

    //using StateRecorderType = StateRecorderBody<DynamicsSystemType>;
    using StateRecorderType = StateRecorderMPI;
    // ===============================================

    std::shared_ptr< StateRecorderType >  m_pStateRecorder;

    std::shared_ptr< TimeStepperType >	m_pTimestepper;
    std::shared_ptr< DynamicsSystemType > m_pDynSys;
    std::shared_ptr< BodyCommunicator >   m_pBodyCommunicator;


    using ProcessCommunicatorType = MPILayer::ProcessCommunicator;
    using ProcessInfoType = typename ProcessCommunicatorType::ProcessInfoType;
    std::shared_ptr< ProcessCommunicatorType > m_pProcCommunicator;


    using TopologyBuilderType = MPILayer::TopologyBuilder ;
    std::shared_ptr< TopologyBuilderType >    m_pTopologyBuilder;

    void writeAllOutput();

    void checkNumberOfBodiesInProcess();
    void checkOwnerOfBodiesInProcess();
    void getMaxRuntime(PREC runtime);

    boost::filesystem::path m_sceneFileParsed;
};


// Implementation
#include "GMSF/States/SimulationManager/SimulationManagerMPI.icc"


#endif // SimulationManagerMPI_HPP
