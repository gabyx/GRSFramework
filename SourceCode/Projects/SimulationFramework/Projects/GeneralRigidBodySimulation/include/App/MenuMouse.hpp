/*
 *  MenuMouse.hpp
 *
 *  Created by Gabriel Nützi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */


#ifndef MenuMouse_hpp
#define MenuMouse_hpp

// Includes =================================
#define _USE_MATH_DEFINES
#include <cstdlib>
#include <iostream>
#include <vector>
#include <cmath>
#include <OGRE/Ogre.h>
#include <OIS/OISEvents.h>
#include <OIS/OISKeyboard.h>
#include <OIS/OISJoyStick.h>
#include <OIS/OISMouse.h>
#include <SdkTrays.h>

#include <Contexts/InputContext.hpp>
// ==========================================

//
//using namespace Ogre;
//using namespace std;



/**
* 	@brief This is the MenuMouse class which manages the GUI mouse in the given SdkTrayManager. It enables and disables the mouse.
*/
class MenuMouse : public OIS::KeyListener, OIS::MouseListener{

 public:
	/** @brief Constructs an orbiting Camera in the given Scene Manager pSceneMgr.
	* @param trayManager Pointer to en existing SdkTrayManager of Ogre.
	* @param name The name of the mouse.
	*/
	MenuMouse(boost::shared_ptr<OgreBites::SdkTrayManager> trayManager, Ogre::String name);
	~MenuMouse();

	/** \name Disabling/Enabling Event Inputs */
    /* @{ */
	void setActive(); 	///< Enables the input.
	void setInactive();	///< Disables the input, which basically only removes this class in the InputManager and makes the GUI mouse invisible.
	/* @} */
 protected:
	Ogre::String m_Name; ///< The mouse name.

	boost::shared_ptr<OgreBites::SdkTrayManager>	m_pTrayMgr; ///< The existing SdkTrayManager from Ogre.

    /** \name KeyListener for keyboard inputs */
	 /* @{ */
	bool keyPressed(const OIS::KeyEvent &arg){return true;};
	bool keyReleased(const OIS::KeyEvent &arg){return true;};
	/* @} */

    /** \name MouseListener for mouse inputs */
    bool mouseMoved(const OIS::MouseEvent &arg);
    bool mousePressed(const OIS::MouseEvent &arg, OIS::MouseButtonID id);
    bool mouseReleased(const OIS::MouseEvent &arg, OIS::MouseButtonID id);
	/* @} */
};




#endif
