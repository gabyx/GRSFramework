#ifndef GRSF_States_SimulationManager_PlaybackManager_hpp
#define GRSF_States_SimulationManager_PlaybackManager_hpp

#include <memory>
#include <OIS/OISEvents.h>
#include <OIS/OISKeyboard.h>
#include <OIS/OISJoyStick.h>
#include <OIS/OISMouse.h>

#include "GRSF/Common/TypeDefs.hpp"

#include "GRSF/Systems/SceneParserGUI.hpp"

#include "GRSF/States/SimulationManager/PlaybackLoader.hpp"
#include "GRSF/States/SimulationManager/PlaybackManagerBase.hpp"
#include "GRSF/Singeltons/Contexts/InputContext.hpp"
#include "GRSF/Dynamics/Buffers/DynamicsState.hpp"

#include "GRSF/States/VideoDropper.hpp"
#include "GRSF/Dynamics/Buffers/StateRecorderResampler.hpp"

class StateRingPoolVisBackFront;
template< typename TStatePool> class PlaybackLoader;
class SharedBufferPlayback;


class PlaybackManager : public PlaybackManagerBase , public OIS::KeyListener {
public:

    DEFINE_CONFIG_TYPES

    PlaybackManager(std::shared_ptr<Ogre::SceneManager> pSceneMgr);
    ~PlaybackManager();

    std::shared_ptr<SharedBufferPlayback >	m_pSharedBuffer;

    std::shared_ptr< DynamicsSystemPlayback > m_pDynSys;

    using SceneParserType =  SceneParserGUI<DynamicsSystemPlayback, DynamicsSystemPlayback::ParserModulesCreator::SceneParserTraits>;
    std::shared_ptr< SceneParserType> m_pSceneParser;

    std::shared_ptr< VideoDropper > m_pVideoDropper;
    struct VideoDropSettings {
        bool m_bVideoDrop;
        double m_FPS;
    } m_VideoDropSettings;

    std::shared_ptr< StateRecorderResampler > m_pStateRecorderResampler;
    struct SimFileDropSettings {
        bool m_bSimFileDrop;
        double m_FPS;
        bool m_bSimFileDropInterpolate;
        double m_startTime;
        double m_endTime;
    } m_SimFileDropSettings;

    bool setup();

    // Accessed by grafic thread  ===============
    void updateScene(double timeSinceLastFrame);
    double getSimulationTime(); // used to access the current simulation state time, from the AppState

    void startPlaybackThread();
    void stopPlaybackThread(bool force_stop);

    // Key Mouse Listener
    bool keyPressed(const OIS::KeyEvent &keyEventRef);
    bool keyReleased(const OIS::KeyEvent &keyEventRef);
    void enableInput(bool value);

private:

    Logging::Log*	m_pSimulationLog;
    Logging::Log*	m_pThreadLog;

    std::shared_ptr<Ogre::SceneManager>	m_pSceneMgr;
    Ogre::SceneNode * m_pBaseNode;

    std::string m_KeyListenerName;

    bool parseScene();
    bool m_bSetupSuccessful;

    const DynamicsState * m_pVisBuffer;

    //Accessed only by Graphic thread ==========
    void updateSimBodies();
    bool writeInitialState();
    void initBeforeThreads();
    void cancelAllWaits();
    struct SettingsVisThread {
        bool m_bFirstPass;
        bool m_bVideoDrop;
    } m_settingsVisThread;

    // =========================================


    //Accessed only by Sim thread ==============
    void threadRunSimulation();
    void initSimThread();
    void cleanUpSimThread();
    struct SettingsSimThread {
        bool m_bSimFileDrop;
        bool m_bSimFileDropInterpolate;
        bool m_bVideoDrop;
    } m_settingsSimThread;
    // =========================================

    // Accessed only by Loader Thread
    std::shared_ptr< PlaybackLoader<StateRingPoolVisBackFront > > m_pFileLoader;


};


#endif // SIMULATIONMANAGERMAZE_HPP
