// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_states_simulationManager_PlaybackManager_hpp
#define GRSF_states_simulationManager_PlaybackManager_hpp

#include <OIS/OISEvents.h>
#include <OIS/OISJoyStick.h>
#include <OIS/OISKeyboard.h>
#include <OIS/OISMouse.h>
#include <memory>

#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/systems/SceneParserGUI.hpp"

#include "GRSF/dynamics/buffers/DynamicsState.hpp"
#include "GRSF/singeltons/contexts/InputContext.hpp"
#include "GRSF/states/simulationManager/PlaybackLoader.hpp"
#include "GRSF/states/simulationManager/PlaybackManagerBase.hpp"

#include "GRSF/dynamics/buffers/StateRecorderResampler.hpp"
#include "GRSF/states/VideoDropper.hpp"

class StateRingPoolVisBackFront;
template <typename TStatePool>
class PlaybackLoader;
class SharedBufferPlayback;

class PlaybackManager : public PlaybackManagerBase, public OIS::KeyListener
{
public:
    DEFINE_CONFIG_TYPES

    PlaybackManager(std::shared_ptr<Ogre::SceneManager> pSceneMgr);
    ~PlaybackManager();

    std::shared_ptr<SharedBufferPlayback> m_pSharedBuffer;

    std::shared_ptr<DynamicsSystemPlayback> m_pDynSys;

    using SceneParserType =
        SceneParserGUI<DynamicsSystemPlayback, DynamicsSystemPlayback::ParserModulesCreator::SceneParserTraits>;
    std::shared_ptr<SceneParserType> m_pSceneParser;

    std::shared_ptr<VideoDropper> m_pVideoDropper;
    struct VideoDropSettings
    {
        bool   m_bVideoDrop;
        double m_FPS;
    } m_VideoDropSettings;

    std::shared_ptr<StateRecorderResampler> m_pStateRecorderResampler;
    struct SimFileDropSettings
    {
        bool   m_bSimFileDrop;
        double m_FPS;
        bool   m_bSimFileDropInterpolate;
        double m_startTime;
        double m_endTime;
    } m_SimFileDropSettings;

    bool setup();

    // Accessed by grafic thread  ===============
    void updateScene(double timeSinceLastFrame);
    double getSimulationTime();  // used to access the current simulation state time, from the AppState

    void startPlaybackThread();
    void stopPlaybackThread(bool force_stop);

    // Key Mouse Listener
    bool keyPressed(const OIS::KeyEvent& keyEventRef);
    bool keyReleased(const OIS::KeyEvent& keyEventRef);
    void enableInput(bool value);

private:
    Logging::Log* m_pSimulationLog;
    Logging::Log* m_pThreadLog;

    std::shared_ptr<Ogre::SceneManager> m_pSceneMgr;
    Ogre::SceneNode*                    m_pBaseNode;

    std::string m_KeyListenerName;

    bool parseScene();
    bool m_bSetupSuccessful;

    const DynamicsState* m_pVisBuffer;

    // Accessed only by Graphic thread ==========
    void updateSimBodies();
    bool writeInitialState();
    void initBeforeThreads();
    void cancelAllWaits();
    struct SettingsVisThread
    {
        bool m_bFirstPass;
        bool m_bVideoDrop;
    } m_settingsVisThread;

    // =========================================

    // Accessed only by Sim thread ==============
    void threadRunSimulation();
    void initSimThread();
    void cleanUpSimThread();
    struct SettingsSimThread
    {
        bool m_bSimFileDrop;
        bool m_bSimFileDropInterpolate;
        bool m_bVideoDrop;
    } m_settingsSimThread;
    // =========================================

    // Accessed only by Loader Thread
    std::shared_ptr<PlaybackLoader<StateRingPoolVisBackFront>> m_pFileLoader;
};

#endif  // SIMULATIONMANAGERMAZE_HPP
