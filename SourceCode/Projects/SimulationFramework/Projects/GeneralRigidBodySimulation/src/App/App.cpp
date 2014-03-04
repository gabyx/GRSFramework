#include "App.hpp"

#include <boost/pointer_cast.hpp>

#include "ApplicationCLOptions.hpp"

App::App()
{
	m_bShutdown			= false;



}


App::~App()
{
  DECONSTRUCTOR_MESSAGE

    m_pAppStateManager.reset();

    delete FileManager::getSingletonPtr();
    delete Logging::LogManager::getSingletonPtr();

	delete GuiContext::getSingletonPtr();
	delete InputContext::getSingletonPtr();
	delete RenderContext::getSingletonPtr();
}


void App::startApp()
{

    std::stringstream processFolder;
    processFolder << PROCESS_FOLDER_PREFIX << 0;
    boost::filesystem::path localDirPath;

    localDirPath = ApplicationCLOptions::getSingletonPtr()->m_localDirs[0];
    localDirPath /= processFolder.str();
    new FileManager(ApplicationCLOptions::getSingletonPtr()->m_globalDir, localDirPath); //Creates path if it does not exist

    new Logging::LogManager;

	new RenderContext;

	if(!RenderContext::getSingletonPtr()->initOgre("RigidBodySimulation v1.0"))
		return;
	RenderContext::getSingletonPtr()->m_pAppLog->logMessage("RenderContext initialized!");

	m_bShutdown = false;

	m_pAppStateManager = boost::shared_ptr<AppStateManager>( new AppStateManager());


	new InputContext;
	if(!InputContext::getSingletonPtr()->initialise())
		return;
	RenderContext::getSingletonPtr()->m_pAppLog->logMessage("InputContext initialized!");


	new GuiContext;
	if(!GuiContext::getSingletonPtr()->initBitesTray())
		return;
	RenderContext::getSingletonPtr()->m_pAppLog->logMessage("GuiContext initialized!");



   SimulationState::create(m_pAppStateManager, "SimulationState");
   PlaybackState::create(m_pAppStateManager, "PlaybackState");

   boost::shared_ptr<AppState> appSim = boost::dynamic_pointer_cast<AppState>(m_pAppStateManager->findAppStateByName("SimulationState"));
   //boost::shared_ptr<AppState> appPlayback = boost::dynamic_pointer_cast<AppState>(m_pAppStateManager->findAppStateByName("PlaybackState"));

   m_pAppStateManager->pushAppState(appSim);
   //m_pAppStateManager->pushAppState(appPlayback);

   m_pAppStateManager->start(appSim);

}
