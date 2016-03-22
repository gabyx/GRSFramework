#ifndef GRSF_Singeltons_Contexts_RenderContext_hpp
#define GRSF_Singeltons_Contexts_RenderContext_hpp


#include <OGRE/OgreCamera.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreLogManager.h>
#include <OGRE/Overlay/OgreOverlaySystem.h>
#include <OGRE/Overlay/OgreOverlay.h>
#include <OGRE/Overlay/OgreOverlayElement.h>
#include <OGRE/Overlay/OgreOverlayManager.h>
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


class RenderContext : public Ogre::Singleton<RenderContext>
{
public:
	RenderContext();
	~RenderContext();

//! @decision Contexts are library specific! -> initOgre
	bool initOgre(Ogre::String wndTitle);

	void updateOgre(double timeSinceLastFrame);

	bool isOgreToBeShutDown()const{return m_bShutDownOgre;}

	std::shared_ptr<Ogre::Root>					      m_pRoot;
	std::shared_ptr<Ogre::OverlaySystem>			m_pOverlaySystem;
	Ogre::RenderWindow*								m_pRenderWnd = nullptr; //no shared pointer because class has no deconstructor!
	Ogre::Viewport*									m_pViewport = nullptr;
	Ogre::Log*										m_pOgreLog = nullptr;
	Ogre::Log*										m_pAppLog = nullptr;
	std::shared_ptr<Ogre::Timer>					m_pTimer;

	bool						m_bShutDownOgre;	// <debug>	better private, but InputContext needs access

	inline void addOverlaySystem(Ogre::SceneManager * mgr){
//				if(mgr && m_overlaySystem){
						mgr->addRenderQueueListener(m_pOverlaySystem.get());
//				}
	}
	inline void removeOverlaySystem(Ogre::SceneManager * mgr){
//				if(mgr && m_overlaySystem){
						mgr->removeRenderQueueListener(m_pOverlaySystem.get());
//				}
	}


private:
	RenderContext(const RenderContext&);
	RenderContext& operator= (const RenderContext&);

	int							m_iNumScreenShots;
};
//=========================================================

#endif	// RENDERCONTEXT_HPP
