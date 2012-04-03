#ifndef GUICONTEXT_HPP
#define GUICONTEXT_HPP


#include <boost/shared_ptr.hpp>
#include <OISMouse.h>
#include <SdkTrays.h>
#include "Contexts/RenderContext.hpp"
#include "Contexts/InputContext.hpp"


/**
* @ingroup	Contexts
* @brief	Specifying the general GuiContext of an application.
*
* This implementation uses "OgreBites Trays". Other implementations could
* use other GUI libraries.
**/
class GuiContext : public Ogre::Singleton<GuiContext>, OgreBites::SdkTrayListener
{
public:
	GuiContext();
	~GuiContext();

//! @decision Contexts are library specific! -> initBitesTray
	bool initBitesTray();
	void updateGuiContext(double timeSinceLastFrame);

private:
	boost::shared_ptr<OgreBites::SdkTrayManager>m_pTrayMgr;
	Ogre::FrameEvent            m_FrameEvent;
};


#endif	
