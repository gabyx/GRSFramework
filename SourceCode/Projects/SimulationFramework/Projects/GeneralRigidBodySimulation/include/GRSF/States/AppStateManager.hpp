#ifndef GRSF_States_AppStateManager_hpp
#define GRSF_States_AppStateManager_hpp

/*=========================================================
	implemenation specific includes
_________________________________________________________*/

#include "GRSF/States/AppState.hpp"
#include "GRSF/Singeltons/Contexts/InputContext.hpp"
#include "GRSF/Singeltons/Contexts/GuiContext.hpp"
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

	std::list<std::shared_ptr<RenderAppState> >	m_ActiveStateStack;
	std::vector<state_info>			m_States;
	bool							m_bShutdown;
};
//=========================================================

#endif	// APPSTATEMANAGER_HPP
