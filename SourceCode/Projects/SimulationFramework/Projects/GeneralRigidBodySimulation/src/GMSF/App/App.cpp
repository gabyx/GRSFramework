#include "GMSF/App/App.hpp"

#include <boost/pointer_cast.hpp>

#include "GMSF/Common/ApplicationCLOptions.hpp"

App::App()
{
	m_bShutdown			= false;



}


App::~App()
{
  DECONSTRUCTOR_MESSAGE
}


void App::startApp()
{

    std::stringstream processFolder;
    processFolder << PROCESS_FOLDER_PREFIX << 0;
    boost::filesystem::path localDirPath;

    localDirPath = ApplicationCLOptions::getSingleton().m_localDirs[0];
    localDirPath /= processFolder.str();

    FileManager fileManager(ApplicationCLOptions::getSingleton().m_globalDir, localDirPath); //Creates path if it does not exist

    Logging::LogManager logManager;

	RenderContext renderContext;

	if(!RenderContext::getSingleton().initOgre("RigidBodySimulation v1.0"))
		return;
	RenderContext::getSingleton().m_pAppLog->logMessage("RenderContext initialized!");

	m_bShutdown = false;

	m_pAppStateManager = std::shared_ptr<AppStateManager>( new AppStateManager());


	InputContext inputContext;
	if(!InputContext::getSingleton().initialise())
		return;
	RenderContext::getSingleton().m_pAppLog->logMessage("InputContext initialized!");


	GuiContext guiContext;
	if(!GuiContext::getSingleton().initBitesTray())
		return;
	RenderContext::getSingleton().m_pAppLog->logMessage("GuiContext initialized!");



   SimulationState::create(m_pAppStateManager, "SimulationState");
   PlaybackState::create(m_pAppStateManager, "PlaybackState");

   std::shared_ptr<AppState> appSim = std::dynamic_pointer_cast<AppState>(m_pAppStateManager->findAppStateByName("SimulationState"));
   //std::shared_ptr<AppState> appPlayback = boost::dynamic_pointer_cast<AppState>(m_pAppStateManager->findAppStateByName("PlaybackState"));

   m_pAppStateManager->pushAppState(appSim);
   //m_pAppStateManager->pushAppState(appPlayback);

   m_pAppStateManager->start(appSim);

   m_pAppStateManager.reset();

}
