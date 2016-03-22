#ifndef GRSF_States_SimulationManager_SimulationManagerGUI_hpp
#define GRSF_States_SimulationManager_SimulationManagerGUI_hpp

#include <memory>
#include <boost/filesystem.hpp>

#include <Ogre.h>

#include <OIS/OISEvents.h>
#include <OIS/OISKeyboard.h>
#include <OIS/OISJoyStick.h>
#include <OIS/OISMouse.h>


#include "GRSF/Common/TypeDefs.hpp"

#include DynamicsSystem_INCLUDE_FILE

#include "GRSF/States/SimulationManager/SimulationManagerBase.hpp"
#include "GRSF/Singeltons/Contexts/InputContext.hpp"

#include "GRSF/Systems/SceneParserGUI.hpp"

#include "GRSF/Common/DynamicCoordinateFrames.hpp"

class DynamicsState;
class StateRecorder;
class SharedBufferDynSys;

class SimulationManagerGUI : public SimulationManagerBase , public OIS::KeyListener {
public:

    DEFINE_CONFIG_TYPES

    SimulationManagerGUI(std::shared_ptr<Ogre::SceneManager> pSceneMgr);
    ~SimulationManagerGUI();


    std::shared_ptr<SharedBufferDynSys >	    m_pSharedBuffer;
    std::shared_ptr<StateRecorder >		m_pStateRecorder;

    void setup(boost::filesystem::path sceneFilePath);
    using SceneParserType = SceneParserGUI< DynamicsSystemType,
                                            DynamicsSystemType::ParserModulesCreator::SceneParserTraits >;
    std::shared_ptr< SceneParserType > m_pSceneParser;

    void updateScene(double timeSinceLastFrame);
    inline void updateSimBodies();
    void initBeforeThreads();

    double getSimulationTime(); // used to access the current simulation state time, from the AppState


    void startSimThread(Threads threadToStop);
    void stopSimThread(Threads threadToStop, bool force_stop);

    // Key Mouse Listener

    bool keyPressed(const OIS::KeyEvent &keyEventRef);
    bool keyReleased(const OIS::KeyEvent &keyEventRef);
    void enableInput(bool value);

private:

    std::shared_ptr<Ogre::SceneManager>	m_pSceneMgr;
    Ogre::SceneNode * m_pBaseNode;
    void setShadowTechniques();

    const DynamicsState * m_pVisBuffer;

    // Accessed only by thread ===================
    void threadRunSimulation();
    void initSimThread();
    void cleanUpSimThread();

    CPUTimer m_global_time;

    void writeAllOutput();
    RecorderSettings m_RecorderSettings;

    void threadRunRecord();
    bool initRecordThread();
    void cleanUpRecordThread();
    struct SettingsSimThread {
        double m_EndTime;
    } m_settingsSimThread;

    void readSharedBuffer();
    void writeSharedBuffer();

    std::string m_KeyListenerName;

    Logging::Log *  m_pSimulationLog;

    std::shared_ptr< TimeStepperType >	m_pTimestepper;

    std::shared_ptr< DynamicsSystemType >		   m_pDynSys;
    // ===========================================

    bool writeInitialState();

    // Visualization of ContactFrame
    bool m_bShowContactFrames;
    boost::mutex m_mutexShowContactFrames;
    void toggleShowContactFrames();
    bool showContactFramesEnabled();

    DynamicCoordinateFrames m_dynCoordFrame;
    inline void updateContactFrameVisualization();
    // ==============================

    // Timming Variables for updateScene =====
    bool m_bFirstPass;

    double m_timeFrameToFrameAvgNoWorkload;
    unsigned int m_counterNoWorkload;

    double m_lastTimeWhenChanged;
    double m_newTimeWhenChanged;
    double m_deltaTimeWhenChanged;

    double m_stopTimeAfterUpdate;

    double m_diffUpdateAndChangedTime;
    int m_passCounter;
    // =========================================



    // File Paths for one Simulation, always reset ==============================
    boost::filesystem::path m_SimFolderPath;
    boost::filesystem::path m_SimFilePath;
};



#endif // SIMULATIONMANAGERMAZE_HPP
