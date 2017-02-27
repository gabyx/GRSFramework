// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_states_AppStateManager_hpp
#define GRSF_states_AppStateManager_hpp

/*=========================================================
    implemenation specific includes
_________________________________________________________*/

#include <OgreWindowEventUtilities.h>
#include "GRSF/singeltons/contexts/GuiContext.hpp"
#include "GRSF/singeltons/contexts/InputContext.hpp"
#include "GRSF/states/AppState.hpp"
//=========================================================

/*=========================================================
    class AppStateManager
_________________________________________________________*/
/**
* @ingroup 	statestuff
* @brief	the AppStateManager is the most important part of an
*		application.
*
* It manages all application states occuring in an application.
*
* @todo		start"AppState" <-> "stopAppState" ?; shutdown?; pause, resume?
**/
class AppStateManager : public AppStateListener
{
public:
    typedef struct
    {
        Ogre::String                    name;
        std::shared_ptr<RenderAppState> state;
    } state_info;

    AppStateManager();
    ~AppStateManager();

    // AppStateListener functions:
    void manageAppState(Ogre::String stateName, std::shared_ptr<RenderAppState> state);

    std::shared_ptr<RenderAppState> findAppStateByName(Ogre::String stateName);
    /*void changeAppState(std::shared_ptr<RenderAppState> state);*/
    bool pushAppState(std::shared_ptr<RenderAppState> state);
    void popAppState(void);
    void shutdown(void);
    // -----

    void start(std::shared_ptr<AppState> state);
    void start(std::shared_ptr<RenderAppState> state);

protected:
    void init(std::shared_ptr<RenderAppState> state);
    void init(std::shared_ptr<AppState> state);

    std::list<std::shared_ptr<RenderAppState>> m_ActiveStateStack;
    std::vector<state_info>                    m_States;
    bool                                       m_bShutdown;
};
//=========================================================

#endif  // APPSTATEMANAGER_HPP
