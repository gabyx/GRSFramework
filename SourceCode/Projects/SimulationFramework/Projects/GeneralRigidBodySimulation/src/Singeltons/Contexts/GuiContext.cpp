#include "Contexts/GuiContext.hpp"

#include "LogDefines.hpp"

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
	m_pTrayMgr = boost::shared_ptr<OgreBites::SdkTrayManager>( new OgreBites::SdkTrayManager("GuiContextTrayMgr",
		RenderContext::getSingletonPtr()->m_pRenderWnd,
		InputContext::getSingletonPtr()->getMouse(), this) );

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
