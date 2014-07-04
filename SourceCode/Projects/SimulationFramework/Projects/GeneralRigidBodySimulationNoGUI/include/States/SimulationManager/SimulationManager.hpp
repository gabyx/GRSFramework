#ifndef SimulationManager_hpp
#define SimulationManager_hpp

#include <memory>
#include <boost/filesystem.hpp>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "SceneParser.hpp"

#include "CPUTimer.hpp"

class DynamicsState;
class StateRecorder;
class SharedBufferDynSys;


class SimulationManager {
public:

    DEFINE_CONFIG_TYPES

    SimulationManager();
    ~SimulationManager();

    std::shared_ptr<SharedBufferDynSys >	    m_pSharedBuffer;
    std::shared_ptr<StateRecorder >		    m_pStateRecorder;

    void setup();
    void setup(boost::filesystem::path sceneFilePath);

    using SceneParserType = SceneParser<DynamicsSystemType>;
    std::shared_ptr< SceneParserType > m_pSceneParser;

    void startSim();

private:

    CPUTimer m_global_time;

    void writeAllOutput();
    RecorderSettings m_RecorderSettings;

    // Accessed only by thread ===================
    void threadRunRecord();
    bool initRecordThread();
    void cleanUpRecordThread();

    struct SettingsSimThread {
        double m_EndTime;
    } m_SettingsSimThread;


    Logging::Log *  m_pSimulationLog;

    std::shared_ptr< TimeStepperType >	m_pTimestepper;

    std::shared_ptr< DynamicsSystemType > m_pDynSys;
    // ===========================================

    int m_nSimBodies;

    // File Paths for one Simulation, always reset ==============================
    boost::filesystem::path m_SimFolderPath;
    boost::filesystem::path m_SimFilePath;
};


#endif // SIMULATIONMANAGERMAZE_HPP
