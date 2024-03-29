// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_states_simulationManager_SimulationManagerGUI_hpp
#define GRSF_states_simulationManager_SimulationManagerGUI_hpp

#include <boost/filesystem.hpp>
#include <memory>

#include <Ogre.h>

#include <OIS/OISEvents.h>
#include <OIS/OISJoyStick.h>
#include <OIS/OISKeyboard.h>
#include <OIS/OISMouse.h>

#include "GRSF/common/TypeDefs.hpp"

#include DynamicsSystem_INCLUDE_FILE

#include "GRSF/singeltons/contexts/InputContext.hpp"
#include "GRSF/states/simulationManager/SimulationManagerBase.hpp"

#include "GRSF/systems/SceneParserGUI.hpp"

#include "GRSF/common/DynamicCoordinateFrames.hpp"

class DynamicsState;
class StateRecorder;
class SharedBufferDynSys;

class SimulationManagerGUI : public SimulationManagerBase, public OIS::KeyListener
{
public:
    DEFINE_CONFIG_TYPES

    SimulationManagerGUI(std::shared_ptr<Ogre::SceneManager> pSceneMgr);
    ~SimulationManagerGUI();

    std::shared_ptr<SharedBufferDynSys> m_pSharedBuffer;
    std::shared_ptr<StateRecorder> m_pStateRecorder;

    void setup(boost::filesystem::path sceneFilePath);
    using SceneParserType =
        SceneParserGUI<DynamicsSystemType, DynamicsSystemType::ParserModulesCreator::SceneParserTraits>;
    std::shared_ptr<SceneParserType> m_pSceneParser;

    void updateScene(double timeSinceLastFrame);
    inline void updateSimBodies();
    void initBeforeThreads();

    double getSimulationTime();  // used to access the current simulation state time, from the AppState

    void startSimThread(Threads threadToStop);
    void stopSimThread(Threads threadToStop, bool force_stop);

    // Key Mouse Listener

    bool keyPressed(const OIS::KeyEvent& keyEventRef);
    bool keyReleased(const OIS::KeyEvent& keyEventRef);
    void enableInput(bool value);

private:
    std::shared_ptr<Ogre::SceneManager> m_pSceneMgr;
    Ogre::SceneNode* m_pBaseNode;
    void setShadowTechniques();

    const DynamicsState* m_pVisBuffer;

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
    struct SettingsSimThread
    {
        double m_EndTime;
    } m_settingsSimThread;

    void readSharedBuffer();
    void writeSharedBuffer();

    std::string m_KeyListenerName;

    Logging::Log* m_pSimulationLog;

    std::shared_ptr<TimeStepperType> m_pTimestepper;

    std::shared_ptr<DynamicsSystemType> m_pDynSys;
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

#endif  // SIMULATIONMANAGERMAZE_HPP
