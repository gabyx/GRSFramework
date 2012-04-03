#include "States/AppStateManager.hpp"

#include "LogDefines.hpp"
#include "PlatformDefines.hpp"

/* template<> AppStateManager* Ogre::Singleton<AppStateManager>::ms_Singleton = 0; */

AppStateManager::AppStateManager()
{
	m_bShutdown = false;
}


AppStateManager::~AppStateManager()
{
  DECONSTRUCTOR_MESSAGE

	while(!m_ActiveStateStack.empty())
	{
		m_ActiveStateStack.back()->exit();
		m_ActiveStateStack.pop_back();
	}

	while(!m_States.empty())
	{
		m_States.pop_back();
	}
}


void AppStateManager::manageAppState(Ogre::String stateName, boost::shared_ptr<RenderAppState> state)
{
	try
	{
		state_info new_state_info;
		new_state_info.name = stateName;
		new_state_info.state = state;
		m_States.push_back(new_state_info);
	}
	catch(std::exception& e)
	{
		//delete state;
		throw Ogre::Exception(Ogre::Exception::ERR_INTERNAL_ERROR, "Error while trying to manage a new AppState\n" + Ogre::String(e.what()), "AppStateManager.cpp (39)");
	}
}


boost::shared_ptr<RenderAppState> AppStateManager::findAppStateByName(Ogre::String stateName)
{
	std::vector<state_info>::iterator itr;

	for(itr=m_States.begin();itr!=m_States.end();itr++)
	{
		if(itr->name==stateName)
			return itr->state;
	}

	return boost::shared_ptr<RenderAppState>();
}


void AppStateManager::start(boost::shared_ptr<RenderAppState> state)
{
	pushAppState(state);

#ifdef _DEBUG
	RenderContext::getSingletonPtr()->m_pAppLog->logMessage("Start main loop...");
#endif

//! @todo	is unsigned long (integer) the right type for "timeSinceLastFrame"?
	double timeSinceLastFrame = 1;
	double startTime = 0;

	RenderContext::getSingletonPtr()->m_pRenderWnd->resetStatistics();

	while(!m_bShutdown && !RenderContext::getSingletonPtr()->isOgreToBeShutDown())
	{
		if(RenderContext::getSingletonPtr()->m_pRenderWnd->isClosed())m_bShutdown = true;

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
		Ogre::WindowEventUtilities::messagePump();
#endif
		if(RenderContext::getSingletonPtr()->m_pRenderWnd->isActive())
		{
      startTime = (double)RenderContext::getSingletonPtr()->m_pTimer->getMicrosecondsCPU() * 1.0e-6;

			m_ActiveStateStack.back()->update(timeSinceLastFrame);

			RenderContext::getSingletonPtr()->updateOgre(timeSinceLastFrame);
			RenderContext::getSingletonPtr()->m_pRoot->renderOneFrame();

			timeSinceLastFrame = (double)RenderContext::getSingletonPtr()->m_pTimer->getMicrosecondsCPU()*1.0e-6 - startTime;
		}
		else
		{
			//sleep(1000);
		}
	}

#ifdef _DEBUG
	RenderContext::getSingletonPtr()->m_pAppLog->logMessage("Main loop quit");
	RenderContext::getSingletonPtr()->m_pAppLog->logMessage("Shutdown OGRE...");
#endif
}



void AppStateManager::start(boost::shared_ptr<AppState> state)
{
	pushAppState(state);


	RenderContext::getSingletonPtr()->m_pAppLog->logMessage("AppStateManager::start(AppState* state) .. Start render loop");


//! @todo	is unsigned long (integer) the right type for "timeSinceLastFrame"?
  double timeSinceLastFrame = 1;
  double startTime = 0;

	RenderContext::getSingletonPtr()->m_pRenderWnd->resetStatistics();

	while(!m_bShutdown && !RenderContext::getSingletonPtr()->isOgreToBeShutDown())
	{
		if(RenderContext::getSingletonPtr()->m_pRenderWnd->isClosed())
			m_bShutdown = true;

//#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
		Ogre::WindowEventUtilities::messagePump();
//#endif


		if(RenderContext::getSingletonPtr()->m_pRenderWnd->isActive())
		{
			startTime = (double)RenderContext::getSingletonPtr()->m_pTimer->getMicrosecondsCPU() * 1.0e-6;

         InputContext::getSingletonPtr()->capture(); //Possible that it kills all app states!

         if(!m_ActiveStateStack.empty()){
			   m_ActiveStateStack.back()->update(timeSinceLastFrame);
         }
         

			RenderContext::getSingletonPtr()->updateOgre(timeSinceLastFrame);

			if (GuiContext::getSingletonPtr())
				GuiContext::getSingletonPtr()->updateGuiContext(timeSinceLastFrame);

			RenderContext::getSingletonPtr()->m_pRoot->renderOneFrame();

			timeSinceLastFrame = (double)RenderContext::getSingletonPtr()->m_pTimer->getMicrosecondsCPU()*1.0e-6 - startTime;
		}
		else
		{
		    //std::cout << "not active"<<std::endl;
			//sleep(1);
		}
	}


	RenderContext::getSingletonPtr()->m_pAppLog->logMessage("AppStateManager::start(AppState* state) .. Quit render loop / Shutdown OGRE");

}


//void AppStateManager::changeAppState(boost::shared_ptr<RenderAppState> state)
//{
//	if(!m_ActiveStateStack.empty())
//	{
//		m_ActiveStateStack.back()->pause();
//		m_ActiveStateStack.pop_back();
//	}
//
//	m_ActiveStateStack.push_back(state);
//	init(state);
//	m_ActiveStateStack.back()->enter();
//}


bool AppStateManager::pushAppState(boost::shared_ptr<RenderAppState> state)
{
	if(!m_ActiveStateStack.empty())
	{
      if(m_ActiveStateStack.back() == state){
         return true;
      }else{
		   if(!m_ActiveStateStack.back()->pause())
			   return false;
      }
	}
   // Check if state is already in active state stack, if so, we need to move this state to the front and do resume instead of enter!!
   std::list<boost::shared_ptr<RenderAppState> >::iterator it;

   bool isInActiveStack = false;
   for(it = m_ActiveStateStack.begin(); it != m_ActiveStateStack.end(); it++){
      if( (*it) == state ){ // Compares the raw pointers!!
         m_ActiveStateStack.erase(it);
         isInActiveStack = true;
         break;
      }
   }

   m_ActiveStateStack.push_back(state);
   init(state);

   if(isInActiveStack){
       m_ActiveStateStack.back()->resume();
   }else{
	   m_ActiveStateStack.back()->enter();
   }

	return true;
}


void AppStateManager::popAppState(void)
{
	if(!m_ActiveStateStack.empty())
	{
		m_ActiveStateStack.back()->exit();
		m_ActiveStateStack.pop_back();
	}

	if(!m_ActiveStateStack.empty())
	{
		init(m_ActiveStateStack.back());
		m_ActiveStateStack.back()->resume();
	}
    else
		shutdown();
}


void AppStateManager::shutdown()
{
	m_bShutdown=true;
}


void AppStateManager::init(boost::shared_ptr<RenderAppState> state)
{
	RenderContext::getSingletonPtr()->m_pRenderWnd->resetStatistics();
}


void AppStateManager::init(boost::shared_ptr<AppState> state)
{
	InputContext::getSingletonPtr()->addKeyListener(state.get(),"AppStateManager::KeyListener");
	InputContext::getSingletonPtr()->addMouseListener(state.get(),"AppStateManager::MouseListener");

	RenderContext::getSingletonPtr()->m_pRenderWnd->resetStatistics();
}
