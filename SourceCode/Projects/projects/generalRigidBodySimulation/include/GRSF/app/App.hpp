/*
 *  GRSF/App/App.hpp
 *
 *  Created by Gabriel Nützi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */


#ifndef GRSF_App_App_hpp
#define GRSF_App_App_hpp

// Includes =================================
#include <memory>

#include "GRSF/States/SimulationState.hpp"
#include "GRSF/States/PlaybackState.hpp"
#include "GRSF/Singeltons/FileManager.hpp"
#include "GRSF/Singeltons/Contexts/InputContext.hpp"
#include "GRSF/Singeltons/Contexts/GuiContext.hpp"
#include "GRSF/Singeltons/Contexts/RenderContext.hpp"

#include "GRSF/States/AppStateManager.hpp"

#include "GRSF/Common/LogDefines.hpp"
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
};
//=========================================================

#endif	// APP_HPP
