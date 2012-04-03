#ifndef RENDERCONTEXT_HPP
#define RENDERCONTEXT_HPP


#include <OGRE/OgreCamera.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreLogManager.h>
#include <OGRE/OgreOverlay.h>
#include <OGRE/OgreOverlayElement.h>
#include <OGRE/OgreOverlayManager.h>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreConfigFile.h>

/**
* @ingroup	Contexts
* @brief	application-wide context for rendering issues.
*
* RenderContext initializes Ogre3D and contains pointers to the window, 
* viewport, Ogre::root, etc.
**/


class RenderContext : 
	public Ogre::Singleton<RenderContext>
{
public:
	RenderContext();
	~RenderContext();

//! @decision Contexts are library specific! -> initOgre
	bool initOgre(Ogre::String wndTitle);

	void updateOgre(double timeSinceLastFrame);

	bool isOgreToBeShutDown()const{return m_bShutDownOgre;}  

	boost::shared_ptr<Ogre::Root>					m_pRoot;
	Ogre::RenderWindow*								m_pRenderWnd; //no shared pointer because class has no deconstructor!
	Ogre::Viewport*									m_pViewport;
	Ogre::Log*										m_pOgreLog;
	Ogre::Log*										m_pAppLog;
	boost::shared_ptr<Ogre::Timer>					m_pTimer;

	bool						m_bShutDownOgre;	// <debug>	better private, but InputContext needs access

private:
	RenderContext(const RenderContext&);
	RenderContext& operator= (const RenderContext&);

	int							m_iNumScreenShots;
};
//=========================================================

#endif	// RENDERCONTEXT_HPP
