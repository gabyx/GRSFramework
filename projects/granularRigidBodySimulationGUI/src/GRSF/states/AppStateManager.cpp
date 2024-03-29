// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/states/AppStateManager.hpp"

#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/PlatformDefines.hpp"

/* template<> AppStateManager* Ogre::Singleton<AppStateManager>::ms_Singleton = 0; */

AppStateManager::AppStateManager()
{
    m_bShutdown = false;
}

AppStateManager::~AppStateManager()
{
    DESTRUCTOR_MESSAGE

    while (!m_ActiveStateStack.empty())
    {
        m_ActiveStateStack.back()->exit();
        m_ActiveStateStack.pop_back();
    }

    while (!m_States.empty())
    {
        m_States.pop_back();
    }
}

void AppStateManager::manageAppState(Ogre::String stateName, std::shared_ptr<RenderAppState> state)
{
    try
    {
        state_info new_state_info;
        new_state_info.name  = stateName;
        new_state_info.state = state;
        m_States.push_back(new_state_info);
    }
    catch (std::exception& e)
    {
        // delete state;
        throw Ogre::Exception(Ogre::Exception::ERR_INTERNAL_ERROR,
                              "Error while trying to manage a new AppState\n" + Ogre::String(e.what()),
                              "AppStateManager.cpp (39)");
    }
}

std::shared_ptr<RenderAppState> AppStateManager::findAppStateByName(Ogre::String stateName)
{
    std::vector<state_info>::iterator itr;

    for (itr = m_States.begin(); itr != m_States.end(); itr++)
    {
        if (itr->name == stateName)
            return itr->state;
    }

    return std::shared_ptr<RenderAppState>();
}

void AppStateManager::start(std::shared_ptr<RenderAppState> state)
{
    pushAppState(state);

#ifdef _DEBUG
    RenderContext::getSingleton().m_pAppLog->logMessage("Start main loop...");
#endif

    //! @todo	is unsigned long (integer) the right type for "timeSinceLastFrame"?
    double timeSinceLastFrame = 1;
    double startTime          = 0;

    RenderContext::getSingleton().m_pRenderWnd->resetStatistics();

    while (!m_bShutdown && !RenderContext::getSingleton().isOgreToBeShutDown())
    {
        if (RenderContext::getSingleton().m_pRenderWnd->isClosed())
            m_bShutdown = true;

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
        Ogre::WindowEventUtilities::messagePump();
#endif
        if (RenderContext::getSingleton().m_pRenderWnd->isActive())
        {
            startTime = (double)RenderContext::getSingleton().m_pTimer->getMicrosecondsCPU() * 1.0e-6;

            m_ActiveStateStack.back()->update(timeSinceLastFrame);

            RenderContext::getSingleton().updateOgre(timeSinceLastFrame);
            RenderContext::getSingleton().m_pRoot->renderOneFrame();

            timeSinceLastFrame =
                (double)RenderContext::getSingleton().m_pTimer->getMicrosecondsCPU() * 1.0e-6 - startTime;
        }
        else
        {
            // sleep(1000);
        }
    }

#ifdef _DEBUG
    RenderContext::getSingleton().m_pAppLog->logMessage("Main loop quit");
    RenderContext::getSingleton().m_pAppLog->logMessage("Shutdown OGRE...");
#endif
}

void AppStateManager::start(std::shared_ptr<AppState> state)
{
    pushAppState(state);

    RenderContext::getSingleton().m_pAppLog->logMessage("AppStateManager::start(AppState* state) .. Start render loop");

    //! @todo	is unsigned long (integer) the right type for "timeSinceLastFrame"?
    double timeSinceLastFrame = 1;
    double startTime          = 0;

    RenderContext::getSingleton().m_pRenderWnd->resetStatistics();

    while (!m_bShutdown && !RenderContext::getSingleton().isOgreToBeShutDown())
    {
        if (RenderContext::getSingleton().m_pRenderWnd->isClosed())
            m_bShutdown = true;

        //#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
        Ogre::WindowEventUtilities::messagePump();
        //#endif

        if (RenderContext::getSingleton().m_pRenderWnd->isActive())
        {
            startTime = (double)RenderContext::getSingleton().m_pTimer->getMicrosecondsCPU() * 1.0e-6;

            ::InputContext::getSingleton().capture();  // Possible that it kills all app states!

            if (!m_ActiveStateStack.empty())
            {
                m_ActiveStateStack.back()->update(timeSinceLastFrame);
            }

            RenderContext::getSingleton().updateOgre(timeSinceLastFrame);

            if (GuiContext::getSingletonPtr())
                GuiContext::getSingleton().updateGuiContext(timeSinceLastFrame);

            RenderContext::getSingleton().m_pRoot->renderOneFrame();

            timeSinceLastFrame =
                (double)RenderContext::getSingleton().m_pTimer->getMicrosecondsCPU() * 1.0e-6 - startTime;
        }
        else
        {
            // std::cout << "not active"<<std::endl;
            // sleep(1);
        }
    }

    RenderContext::getSingleton().m_pAppLog->logMessage(
        "AppStateManager::start(AppState* state) .. Quit render loop / Shutdown OGRE");
}

// void AppStateManager::changeAppState(std::shared_ptr<RenderAppState> state)
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

bool AppStateManager::pushAppState(std::shared_ptr<RenderAppState> state)
{
    if (!m_ActiveStateStack.empty())
    {
        if (m_ActiveStateStack.back() == state)
        {
            return true;
        }
        else
        {
            if (!m_ActiveStateStack.back()->pause())
                return false;
        }
    }
    // Check if state is already in active state stack, if so, we need to move this state to the front and do resume
    // instead of enter!!
    std::list<std::shared_ptr<RenderAppState>>::iterator it;

    bool isInActiveStack = false;
    for (it = m_ActiveStateStack.begin(); it != m_ActiveStateStack.end(); it++)
    {
        if ((*it) == state)
        {  // Compares the raw pointers!!
            m_ActiveStateStack.erase(it);
            isInActiveStack = true;
            break;
        }
    }

    m_ActiveStateStack.push_back(state);
    init(state);

    if (isInActiveStack)
    {
        m_ActiveStateStack.back()->resume();
    }
    else
    {
        m_ActiveStateStack.back()->enter();
    }

    return true;
}

void AppStateManager::popAppState(void)
{
    if (!m_ActiveStateStack.empty())
    {
        m_ActiveStateStack.back()->exit();
        m_ActiveStateStack.pop_back();
    }

    if (!m_ActiveStateStack.empty())
    {
        init(m_ActiveStateStack.back());
        m_ActiveStateStack.back()->resume();
    }
    else
        shutdown();
}

void AppStateManager::shutdown()
{
    m_bShutdown = true;
}

void AppStateManager::init(std::shared_ptr<RenderAppState> state)
{
    RenderContext::getSingleton().m_pRenderWnd->resetStatistics();
}

void AppStateManager::init(std::shared_ptr<AppState> state)
{
    //	::InputContext::getSingleton().addKeyListener(state.get(),"AppStateManager::KeyListener");
    //	::InputContext::getSingleton().addMouseListener(state.get(),"AppStateManager::MouseListener");

    RenderContext::getSingleton().m_pRenderWnd->resetStatistics();
}
