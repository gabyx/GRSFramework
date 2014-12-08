#include "GMSF/Singeltons/Contexts/GuiContext.hpp"

#include "GMSF/Common/LogDefines.hpp"

//=========================================================

template<> GuiContext* Ogre::Singleton<GuiContext>::msSingleton = 0;

//=========================================================

GuiContext::GuiContext()
{

}

GuiContext::~GuiContext()
{
  DECONSTRUCTOR_MESSAGE
}

bool GuiContext::initBitesTray()
{
	m_pTrayMgr = std::shared_ptr<OgreBites::SdkTrayManager>( new OgreBites::SdkTrayManager("GuiContextTrayMgr",
		RenderContext::getSingleton().m_pRenderWnd,
		InputContext::getSingleton().getMouse(), this) );

    m_pTrayMgr->showFrameStats(OgreBites::TL_BOTTOMLEFT);
/**
* @decision	do not show the OGRE Logo! commented out:
*    m_pTrayMgr->showLogo(OgreBites::TL_BOTTOMRIGHT);
**/
    m_pTrayMgr->hideCursor();

	return true;
}

void GuiContext::updateGuiContext(double timeSinceLastFrame)
{
	m_FrameEvent.timeSinceLastFrame = timeSinceLastFrame;
    m_pTrayMgr->frameRenderingQueued(m_FrameEvent);
}
