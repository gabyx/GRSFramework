#include "Contexts/RenderContext.hpp"

#include "LogDefines.hpp"

//=========================================================

using namespace Ogre;

//=========================================================

template<> RenderContext* Ogre::Singleton<RenderContext>::ms_Singleton = 0;

RenderContext::RenderContext()
{
    m_bShutDownOgre		= false;
    m_iNumScreenShots	= 0;

}


bool RenderContext::initOgre(Ogre::String wndTitle)
{
    Ogre::LogManager* logMgr = new Ogre::LogManager();

// OGRE LOG
#if LogToFileOgre == 1
    m_pOgreLog = Ogre::LogManager::getSingleton().createLog("OgreLogfile.log", true, true, false);
#else
    m_pOgreLog = Ogre::LogManager::getSingleton().createLog("OgreLogfile.log", true, true, true);
#endif
#if LogToConsoleOgre == 1
    m_pOgreLog->setDebugOutputEnabled(true);
#else
	m_pOgreLog->setDebugOutputEnabled(false);
#endif


// APP LOG
#if LogToFileApp == 1
  m_pAppLog = Ogre::LogManager::getSingleton().createLog("AppLogfile.log",false,true,false);
#else
  m_pAppLog = Ogre::LogManager::getSingleton().createLog("AppLogfile.log",false,true,true);
#endif
#if LogToConsoleApp == 1
  m_pAppLog->setDebugOutputEnabled(true);
#else
  m_pAppLog->setDebugOutputEnabled(false);
#endif
  m_pAppLog->setTimeStampEnabled(false);


	m_pAppLog->logMessage("RenderContext::Render Window initializing...");
	// Render Window ==========================================================================
	m_pRoot = boost::shared_ptr<Ogre::Root>(new Ogre::Root());

	if(m_pRoot->restoreConfig())
	{
	}
	else if(!m_pRoot->showConfigDialog())
	{
		return false;
	};

	m_pRenderWnd = m_pRoot->initialise(true, wndTitle);

	m_pViewport = m_pRenderWnd->addViewport(0);
	m_pAppLog->logMessage("RenderContext::Render Window initialized!");


	m_pAppLog->logMessage("RenderContext::Resources initializing...");
	// Resources initializing ==================================================================
    Ogre::String secName, typeName, archName;
    Ogre::ConfigFile cf;
    cf.load("resources.cfg");

    Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();
    while (seci.hasMoreElements())
    {
        secName = seci.peekNextKey();
        Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
        Ogre::ConfigFile::SettingsMultiMap::iterator i;
        for (i = settings->begin(); i != settings->end(); ++i)
        {
            typeName = i->first;
            archName = i->second;
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(archName, typeName, secName);
        }
    }
    Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
	m_pAppLog->logMessage("RenderContext::Resources initialized...");

	// Timer =====================================================================================
	m_pTimer = boost::shared_ptr<Ogre::Timer>(new Ogre::Timer());
    m_pTimer->reset();


    m_pRenderWnd->setActive(true);

    return true;
}


RenderContext::~RenderContext()
{
  DECONSTRUCTOR_MESSAGE
}


void RenderContext::updateOgre(double timeSinceLastFrame)
{
}
