// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_states_AppState_hpp
#define GRSF_states_AppState_hpp

/*=========================================================
	implementation specific includes & predefinitions
_________________________________________________________*/
#include <memory>
#include "GRSF/singeltons/contexts/RenderContext.hpp"


class RenderAppState;
//=========================================================

/*=========================================================
	class AppStateListener
_________________________________________________________*/
/**
* @ingroup	statestuff
* @brief	AppStateListener is a base class for application
*		state managers.
*
* By deriving from AppStateListener the interface for handling application
* states is provided.
**/
class AppStateListener
{
public:
	AppStateListener(void){};
	virtual ~AppStateListener(void){};

	virtual void manageAppState(Ogre::String stateName, std::shared_ptr<RenderAppState> state) = 0;

	virtual std::shared_ptr<RenderAppState>	findAppStateByName(Ogre::String stateName) = 0;
	/*virtual void			changeAppState(std::shared_ptr<RenderAppState> state) = 0;*/
	virtual bool			pushAppState(std::shared_ptr<RenderAppState> state) = 0;
	virtual void			popAppState() = 0;
	virtual void			shutdown() = 0;
};
//=========================================================

/*=========================================================
	class RenderAppState:
_________________________________________________________*/
/**
* @ingroup	myFramework
* @brief	RenderAppState is an application state with render only
*		functionality.
*
* RenderAppState can be handled by an AppStateListener.
**/
class RenderAppState
{
public:
	//! @todo	<debug> should be protected, but RenderContext needs Access over m_pCurrentState

	static void create(std::shared_ptr<AppStateListener> parent, const Ogre::String name){};

	virtual void enter(void) = 0;
	virtual void exit(void) = 0;
	virtual bool pause(void){return false;}
	virtual void resume(void){};
	virtual void update(double timeSinceLastFrame) = 0;

	std::shared_ptr<Ogre::SceneManager>		m_pSceneMgr;

	virtual ~RenderAppState(){};
protected:

   bool m_bEntered;

	RenderAppState(void){};

	std::shared_ptr<RenderAppState>	findAppStateByName(Ogre::String stateName){return m_pParent->findAppStateByName(stateName);}
	/*void			changeAppState(std::shared_ptr<RenderAppState> state){m_pParent->changeAppState(state);}*/
	bool			pushAppState(std::shared_ptr<RenderAppState> state){return m_pParent->pushAppState(state);}
	void			popAppState(void){m_pParent->popAppState();}
	void			shutdown(void){m_pParent->shutdown();}

	AppStateListener*  m_pParent;
};
//=========================================================


/*=========================================================
	AppState implementation specific includes
_________________________________________________________*/
#include <OISEvents.h>										// <debug> all needed?
#include <OISInputManager.h>
#include <OISKeyboard.h>
#include <OISMouse.h>
//=========================================================

/*=========================================================
	Class AppState
_________________________________________________________*/
/**
* @ingroup myFramework
* @brief AppState is an application state with render &	input functionality.
*
* It can be handled by an AppStateListener. Actually it consists only of the
* RenderAppState & the OIS::...Listener functions.
* If a GUI should be supported, derive your application state from AppState AND
* OgreBites::SdkTrayListener & add an SdkTrayManager to your application state.
* (Of course only if the GUI is implemented with OgreBites Trays). See the
* Dummy...States for more details & examples.
**/
class AppState : public RenderAppState, public OIS::KeyListener, public OIS::MouseListener
{
};
//=========================================================

/*=========================================================
	Macro definition to automatically create a
	(Render)AppState's create(...) function.
_________________________________________________________*/
#define DECLARE_APPSTATE_CLASS(T)										\
	static void create(std::shared_ptr<AppStateListener> parent, const Ogre::String name)	\
{																		\
	std::shared_ptr<T> myAppState = std::shared_ptr<T>(new T());											\
	myAppState->m_pParent = parent.get();										\
	parent->manageAppState(name, myAppState);							\
}
//=========================================================

#endif	// APPSTATE_HPP
