/*
 *  App.hpp
 *
 *  Created by Gabriel Nützi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */


#ifndef GMSF_App_App_hpp
#define GMSF_App_App_hpp

// Includes =================================
#include <memory>

#include "SimulationState.hpp"
#include "PlaybackState.hpp"
#include "FileManager.hpp"
#include "Contexts/InputContext.hpp"
#include "Contexts/GuiContext.hpp"
#include "Contexts/RenderContext.hpp"

#include "AppStateManager.hpp"

#include "LogDefines.hpp"
// ===========================================


/**
* @ingroup	testdummyapp
* @brief	This class is the Application class.
*
* An application is created & started in main.cpp
*
* @todo		startApp <-> stopApp{clean up all the stuff done in startApp, on shutdown call stopApp first}
* @wish		video exporter: store SysState at desired framerate of a simulation; rerender frames after & store them; automatic .avi generation out of these frames
* @wish		surrounding "Browser" allows handling several Apps: startApp stopApp required (create - destroy; pause - resume); + 1 layer of handling (loading + running)
* @todo		OgreSingleton implementation is used everywhere -> use an own one (or boost::singleton)
* @todo		take flag switching (TEST_GAME, APP_WITH_INPUT, etc.) into separate header "defs.hpp"; work with values ("define APP_WITH_INPUT 1")
**/

class App
{
public:
	App();
	~App();

	void startApp(); ///< Starts the Application.

private:

	std::shared_ptr<AppStateManager>	m_pAppStateManager; ///< The AppStateManager which handles all AppStates in a queue.

	bool						m_bShutdown; 				///< Bool to indicate that the App should be shutdown.
};
//=========================================================

#endif	// APP_HPP
