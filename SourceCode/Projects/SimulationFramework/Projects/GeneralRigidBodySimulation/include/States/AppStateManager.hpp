#ifndef APPSTATEMANAGER_HPP
#define APPSTATEMANAGER_HPP

/*=========================================================
	implemenation specific includes
_________________________________________________________*/

#include "States/AppState.hpp"
#include "Contexts/InputContext.hpp"
#include "Contexts/GuiContext.hpp"
#include <OgreWindowEventUtilities.h>
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
		Ogre::String name;
		boost::shared_ptr<RenderAppState> state;
	} state_info;

	AppStateManager();
	~AppStateManager();

	// AppStateListener functions:
	void manageAppState(Ogre::String stateName, boost::shared_ptr<RenderAppState> state);

	boost::shared_ptr<RenderAppState> findAppStateByName(Ogre::String stateName);
	/*void changeAppState(boost::shared_ptr<RenderAppState> state);*/
	bool pushAppState(boost::shared_ptr<RenderAppState> state);
	void popAppState(void);
	void shutdown(void);
	// -----

	void start(boost::shared_ptr<AppState> state);
	void start(boost::shared_ptr<RenderAppState> state);

protected:
	void init(boost::shared_ptr<RenderAppState> state);
	void init(boost::shared_ptr<AppState> state);

	std::list<boost::shared_ptr<RenderAppState> >	m_ActiveStateStack;
	std::vector<state_info>			m_States;
	bool							m_bShutdown;
};
//=========================================================

#endif	// APPSTATEMANAGER_HPP
