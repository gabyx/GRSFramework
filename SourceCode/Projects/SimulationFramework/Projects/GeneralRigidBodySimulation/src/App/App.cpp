#include "App.hpp"

#include <boost/pointer_cast.hpp>


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
      std::cout <<"Pointer: "<< FileManager::getSingletonPtr() << std::endl;

      new FileManager();
      new Logging::LogManager();

	new RenderContext();
	  std::cout <<"Pointer: "<< FileManager::getSingletonPtr() << std::endl;
	if(!RenderContext::getSingletonPtr()->initOgre("RigidBodySimulation v1.0"))
		return;
	RenderContext::getSingletonPtr()->m_pAppLog->logMessage("RigidBodySimulation(RenderContext) initialized!");

	m_bShutdown = false;

	m_pAppStateManager = boost::shared_ptr<AppStateManager>( new AppStateManager());


	new InputContext();
	if(!InputContext::getSingletonPtr()->initialise())
		return;
	RenderContext::getSingletonPtr()->m_pAppLog->logMessage("InputContext initialized!");


	new GuiContext();
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
