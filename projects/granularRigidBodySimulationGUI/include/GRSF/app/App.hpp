// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_app_App_hpp
#define GRSF_app_App_hpp

// Includes =================================
#include <memory>

#include "GRSF/states/SimulationState.hpp"
#include "GRSF/states/PlaybackState.hpp"
#include "GRSF/singeltons/FileManager.hpp"
#include "GRSF/singeltons/contexts/InputContext.hpp"
#include "GRSF/singeltons/contexts/GuiContext.hpp"
#include "GRSF/singeltons/contexts/RenderContext.hpp"

#include "GRSF/states/AppStateManager.hpp"

#include "GRSF/common/LogDefines.hpp"
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
