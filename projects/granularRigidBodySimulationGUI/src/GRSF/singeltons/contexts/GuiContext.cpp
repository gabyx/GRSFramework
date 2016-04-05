// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/singeltons/contexts/GuiContext.hpp"

#include "GRSF/common/LogDefines.hpp"

//=========================================================

template<> GuiContext* Ogre::Singleton<GuiContext>::msSingleton = 0;

//=========================================================

GuiContext::GuiContext()
{
}

GuiContext::~GuiContext()
{
  DESTRUCTOR_MESSAGE

}

bool GuiContext::initBitesTray()
{

	m_pTrayMgr = std::shared_ptr<OgreBites::SdkTrayManager>( new OgreBites::SdkTrayManager("GuiContextTrayMgr",
		RenderContext::getSingleton().m_pRenderWnd,
		::InputContext::getSingleton().getInputContext(), this) );

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
