// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_states_simulationManager_SimulationManager_hpp
#define GRSF_states_simulationManager_SimulationManager_hpp

#include <boost/filesystem.hpp>
#include <memory>

#include "GRSF/common/ApplicationSignalHandler.hpp"
#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include TimeStepper_INCLUDE_FILE
#include DynamicsSystem_INCLUDE_FILE
#include "GRSF/systems/SceneParser.hpp"

#include "GRSF/common/CPUTimer.hpp"

class DynamicsState;
class StateRecorder;
class SharedBufferDynSys;

class SimulationManager
{
    public:
    DEFINE_CONFIG_TYPES

    SimulationManager();
    ~SimulationManager();

    std::shared_ptr<SharedBufferDynSys> m_pSharedBuffer;
    std::shared_ptr<StateRecorder>      m_pStateRecorder;

    void setup();
    void setup(boost::filesystem::path sceneFilePath);

    using SceneParserType =
        SceneParser<DynamicsSystemType, DynamicsSystemType::ParserModulesCreator::SceneParserTraits>;
    std::shared_ptr<SceneParserType> m_pSceneParser;

    void startSim();

    private:
    CPUTimer m_global_time;

    void             writeAllOutput();
    RecorderSettings m_RecorderSettings;

    // Accessed only by thread ===================

    template <bool handleSignals = false>
    void           threadRunRecord()
    {
        m_pSimulationLog->logMessage("---> SimulationManager: Simulation entering...");
        if (initRecordThread())
        {
            // wait for vis thread! (which does some loops before)
            m_global_time.start();

            while (1)
            {
                if (handleSignals)
                {
                    ApplicationSignalHandler::getSingleton().handlePendingSignals();
                }
                // Do one iteration
                m_pTimestepper->doTimeStep();
                writeAllOutput();
                // Check if simulation can be aborted!
                if (m_pTimestepper->finished())
                {
                    m_pSimulationLog->logMessage("---> SimulationManager: Timestepper finished, exit...");
                    break;
                }
            }
            cleanUpRecordThread();
        }
        m_pSimulationLog->logMessage("---> SimulationManager: Simulation leaving...");
    }

    bool initRecordThread();
    void cleanUpRecordThread();

    struct SettingsSimThread
    {
        double m_EndTime;
    } m_settingsSimThread;

    Logging::Log* m_pSimulationLog;

    std::shared_ptr<TimeStepperType> m_pTimestepper;

    std::shared_ptr<DynamicsSystemType> m_pDynSys;
    // ===========================================

    unsigned int m_nSimBodies;

    // File Paths for one Simulation, always reset ==============================
    boost::filesystem::path m_SimFolderPath;
    boost::filesystem::path m_SimFilePath;
};

#endif  // SIMULATIONMANAGERMAZE_HPP
