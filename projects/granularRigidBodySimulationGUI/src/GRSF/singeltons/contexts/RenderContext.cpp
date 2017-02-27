// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/singeltons/contexts/RenderContext.hpp"

#include <boost/filesystem.hpp>

#include "GRSF/common/LogDefines.hpp"
#include "GRSF/singeltons/FileManager.hpp"

//=========================================================

using namespace Ogre;

//=========================================================

template <>
RenderContext* Ogre::Singleton<RenderContext>::msSingleton = 0;

RenderContext::RenderContext()
{
    m_bShutDownOgre   = false;
    m_iNumScreenShots = 0;
}

bool RenderContext::initOgre(Ogre::String wndTitle)
{
    m_pRoot          = std::shared_ptr<Ogre::Root>(new Ogre::Root());
    m_pOverlaySystem = std::shared_ptr<Ogre::OverlaySystem>(new Ogre::OverlaySystem());

    // OGRE LOG
    boost::filesystem::path filePath = FileManager::getSingleton().getLocalDirectoryPath();
    filePath /= GLOBAL_LOG_FOLDER_DIRECTORY;
    if (!boost::filesystem::exists(filePath))
    {
        boost::filesystem::create_directories(filePath);
    }
    filePath /= "OgreLogfile.log";

#if OGRELOG_TOFILE == 1
    m_pOgreLog = Ogre::LogManager::getSingleton().createLog(filePath.string(), true, true, false);
#else
    m_pOgreLog = Ogre::LogManager::getSingleton().createLog(filePath.string(), true, true, true);
#endif
#if OGRELOG_TOCONSOLE == 1
    m_pOgreLog->setDebugOutputEnabled(true);
#else
    m_pOgreLog->setDebugOutputEnabled(false);
#endif

    // APP LOG
    filePath = FileManager::getSingleton().getLocalDirectoryPath();
    filePath /= GLOBAL_LOG_FOLDER_DIRECTORY;
    filePath /= "AppLogfile.log";
#if APPLOG_TOFILE == 1
    m_pAppLog = Ogre::LogManager::getSingleton().createLog(filePath.string(), false, true, false);
#else
    m_pAppLog = Ogre::LogManager::getSingleton().createLog(filePath.string(), false, true, true);
#endif
#if APPLOG_TOCONSOLE == 1
    m_pAppLog->setDebugOutputEnabled(true);
#else
    m_pAppLog->setDebugOutputEnabled(false);
#endif
    m_pAppLog->setTimeStampEnabled(false);

    m_pAppLog->logMessage("RenderContext::Render Window initializing...");
    // Render Window ==========================================================================

    if (m_pRoot->restoreConfig())
    {
    }
    else if (!m_pRoot->showConfigDialog())
    {
        return false;
    };

    m_pRenderWnd = m_pRoot->initialise(true, wndTitle);

    m_pViewport = m_pRenderWnd->addViewport(0);
    m_pAppLog->logMessage("RenderContext::Render Window initialized!");

    m_pAppLog->logMessage("RenderContext::Resources initializing...");
    // Resources initializing ==================================================================
    Ogre::String     secName, typeName, archName;
    Ogre::ConfigFile cf;
    cf.load("resources.cfg");

    Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();
    while (seci.hasMoreElements())
    {
        secName                                               = seci.peekNextKey();
        Ogre::ConfigFile::SettingsMultiMap*          settings = seci.getNext();
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
    m_pTimer = std::shared_ptr<Ogre::Timer>(new Ogre::Timer());
    m_pTimer->reset();

    m_pRenderWnd->setActive(true);

    return true;
}

RenderContext::~RenderContext()
{
    DESTRUCTOR_MESSAGE
}

void RenderContext::updateOgre(double timeSinceLastFrame)
{
}
