#include "GRSF/App/App.hpp"

#include <boost/pointer_cast.hpp>

#include "GRSF/Common/ApplicationCLOptions.hpp"

App::App() {
}


App::~App() {
    DECONSTRUCTOR_MESSAGE
}


void App::startApp() {

    std::stringstream processFolder;
    processFolder << PROCESS_FOLDER_PREFIX << 0;
    boost::filesystem::path localDirPath;

    localDirPath = ApplicationCLOptions::getSingleton().getLocalDirs()[0];
    localDirPath /= processFolder.str();

    INSTANCIATE_UNIQUE_SINGELTON_CTOR( FileManager , fileManager , (ApplicationCLOptions::getSingleton().getGlobalDir(), localDirPath) )

    INSTANCIATE_UNIQUE_SINGELTON(Logging::LogManager, logManager)

    INSTANCIATE_UNIQUE_SINGELTON(RenderContext, renderContext)

    if(!RenderContext::getSingleton().initOgre("RigidBodySimulation v1.0"))
        return;
    RenderContext::getSingleton().m_pAppLog->logMessage("RenderContext initialized!");

    INSTANCIATE_UNIQUE_SINGELTON(InputContext, inputContext )

    if(!::InputContext::getSingleton().initialise())
        return;

    RenderContext::getSingleton().m_pAppLog->logMessage("InputContext initialized!");

    INSTANCIATE_UNIQUE_SINGELTON( GuiContext, guiContext)

    if(!GuiContext::getSingleton().initBitesTray())
        return;
    RenderContext::getSingleton().m_pAppLog->logMessage("GuiContext initialized!");

    std::shared_ptr<AppStateManager> pAppStateManager = std::shared_ptr<AppStateManager>( new AppStateManager());
    SimulationState::create(pAppStateManager, "SimulationState");
    PlaybackState::create(pAppStateManager, "PlaybackState");

    std::shared_ptr<AppState> appSim = std::dynamic_pointer_cast<AppState>(pAppStateManager->findAppStateByName("SimulationState"));
    //std::shared_ptr<AppState> appPlayback = boost::dynamic_pointer_cast<AppState>(pAppStateManager->findAppStateByName("PlaybackState"));

    pAppStateManager->pushAppState(appSim);
    //pAppStateManager->pushAppState(appPlayback);

    pAppStateManager->start(appSim);

    pAppStateManager.reset();

}
