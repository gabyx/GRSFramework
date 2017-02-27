// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_app_MenuMouse_hpp
#define GRSF_app_MenuMouse_hpp

// Includes =================================
#define _USE_MATH_DEFINES
#include <OGRE/Ogre.h>
#include <OIS/OISEvents.h>
#include <OIS/OISJoyStick.h>
#include <OIS/OISKeyboard.h>
#include <OIS/OISMouse.h>
#include <SdkTrays.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "GRSF/singeltons/contexts/InputContext.hpp"
// ==========================================

//
// using namespace Ogre;
// using namespace std;

/**
* 	@brief This is the MenuMouse class which manages the GUI mouse in the given SdkTrayManager. It enables and disables
* the mouse.
*/
class MenuMouse : public OIS::KeyListener, OIS::MouseListener
{
    public:
    /** @brief Constructs an orbiting Camera in the given Scene Manager pSceneMgr.
    * @param trayManager Pointer to en existing SdkTrayManager of Ogre.
    * @param name The name of the mouse.
    */
    MenuMouse(std::shared_ptr<OgreBites::SdkTrayManager> trayManager, Ogre::String name);
    ~MenuMouse();

    /** \name Disabling/Enabling Event Inputs */
    /* @{ */
    void setActive();    ///< Enables the input.
    void setInactive();  ///< Disables the input, which basically only removes this class in the InputManager and makes
                         /// the GUI mouse invisible.
    /* @} */
    protected:
    Ogre::String m_Name;  ///< The mouse name.

    std::shared_ptr<OgreBites::SdkTrayManager> m_pTrayMgr;  ///< The existing SdkTrayManager from Ogre.

    /** \name KeyListener for keyboard inputs */
    /* @{ */
    bool keyPressed(const OIS::KeyEvent& arg)
    {
        return true;
    };
    bool keyReleased(const OIS::KeyEvent& arg)
    {
        return true;
    };
    /* @} */

    /** \name MouseListener for mouse inputs */
    bool mouseMoved(const OIS::MouseEvent& arg);
    bool mousePressed(const OIS::MouseEvent& arg, OIS::MouseButtonID id);
    bool mouseReleased(const OIS::MouseEvent& arg, OIS::MouseButtonID id);
    /* @} */
};

#endif
