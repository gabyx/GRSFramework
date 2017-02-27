// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/app/App.hpp"

#include <boost/pointer_cast.hpp>

#include "GRSF/common/ApplicationCLOptions.hpp"

App::App()
{
}

App::~App()
{
    DESTRUCTOR_MESSAGE
}

void App::startApp()
{
    std::stringstream processFolder;
    processFolder << PROCESS_FOLDER_PREFIX << 0;
    boost::filesystem::path localDirPath;

    localDirPath = ApplicationCLOptions::getSingleton().getLocalDirs()[0];
    localDirPath /= processFolder.str();

    INSTANCIATE_UNIQUE_SINGELTON_CTOR(
        FileManager, fileManager, (ApplicationCLOptions::getSingleton().getGlobalDir(), localDirPath))

    INSTANCIATE_UNIQUE_SINGELTON(Logging::LogManager, logManager)

    INSTANCIATE_UNIQUE_SINGELTON(RenderContext, renderContext)

    std::string title = "GRSFramework SimGUI: version: " GRSF_VERSION_STRING
#ifndef NDEBUG
                        " | config: debug";
#else
                        " | config: release";
#endif

    if (!RenderContext::getSingleton().initOgre(title))
        return;
    RenderContext::getSingleton().m_pAppLog->logMessage("RenderContext initialized!");

    INSTANCIATE_UNIQUE_SINGELTON(InputContext, inputContext)

    if (!::InputContext::getSingleton().initialise())
        return;

    RenderContext::getSingleton().m_pAppLog->logMessage("InputContext initialized!");

    INSTANCIATE_UNIQUE_SINGELTON(GuiContext, guiContext)

    if (!GuiContext::getSingleton().initBitesTray())
        return;
    RenderContext::getSingleton().m_pAppLog->logMessage("GuiContext initialized!");

    std::shared_ptr<AppStateManager> pAppStateManager = std::shared_ptr<AppStateManager>(new AppStateManager());
    SimulationState::create(pAppStateManager, "SimulationState");
    PlaybackState::create(pAppStateManager, "PlaybackState");

    std::shared_ptr<AppState> appSim =
        std::dynamic_pointer_cast<AppState>(pAppStateManager->findAppStateByName("SimulationState"));
    // std::shared_ptr<AppState> appPlayback =
    // boost::dynamic_pointer_cast<AppState>(pAppStateManager->findAppStateByName("PlaybackState"));

    pAppStateManager->pushAppState(appSim);
    // pAppStateManager->pushAppState(appPlayback);

    pAppStateManager->start(appSim);

    pAppStateManager.reset();
}
