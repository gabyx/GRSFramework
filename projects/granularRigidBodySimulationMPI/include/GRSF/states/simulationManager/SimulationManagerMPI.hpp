// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_states_simulationManager_SimulationManagerMPI_hpp
#define GRSF_states_simulationManager_SimulationManagerMPI_hpp

#include <mpi.h>

#include <boost/filesystem.hpp>
#include <memory>

#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/dynamics/general/BodyCommunicator.hpp"
#include "GRSF/dynamics/general/MPICommunication.hpp"
#include "GRSF/dynamics/general/MPIInformation.hpp"
#include "GRSF/dynamics/general/MPITopologyBuilder.hpp"

#include "GRSF/common/CPUTimer.hpp"

class StateRecorder;
class StateRecorderBody;
class StateRecorderProcess;
class StateRecorderMPI;

class SimulationManagerMPI
{
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

    struct SettingsSimThread
    {
        double m_EndTime;
    } m_settingsSimThread;

    Logging::Log* m_pSimulationLog;

    // using StateRecorderType = StateRecorderBody<DynamicsSystemType>;
    using StateRecorderType = StateRecorderMPI;
    // ===============================================

    std::shared_ptr<StateRecorderType> m_pStateRecorder;
    void gracefullyExit(int signal);

    std::shared_ptr<TimeStepperType>    m_pTimestepper;
    std::shared_ptr<DynamicsSystemType> m_pDynSys;
    std::shared_ptr<BodyCommunicator>   m_pBodyCommunicator;

    using ProcessCommunicatorType = MPILayer::ProcessCommunicator;
    using ProcessInfoType         = typename ProcessCommunicatorType::ProcessInfoType;
    std::shared_ptr<ProcessCommunicatorType> m_pProcCommunicator;

    using TopologyBuilderType = MPILayer::TopologyBuilder<ProcessCommunicatorType>;
    std::shared_ptr<TopologyBuilderType> m_pTopologyBuilder;

    void writeAllOutput();

    void checkNumberOfBodiesInProcess();
    void checkOwnerOfBodiesInProcess();
    void getMaxRuntime(PREC runtime);

    boost::filesystem::path m_sceneFileParsed;
};

#endif  // SimulationManagerMPI_HPP
