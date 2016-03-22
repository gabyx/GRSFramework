#ifndef GRSF_Singeltons_Contexts_GuiContext_hpp
#define GRSF_Singeltons_Contexts_GuiContext_hpp


#include <memory>
#include <OISMouse.h>
#include <SdkTrays.h>
#include "GRSF/Singeltons/Contexts/RenderContext.hpp"
#include "GRSF/Singeltons/Contexts/InputContext.hpp"


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
	std::shared_ptr<OgreBites::SdkTrayManager>m_pTrayMgr;
	Ogre::FrameEvent            m_FrameEvent;
	Ogre::OverlaySystem * m_overlaySystem = nullptr;
};


#endif
