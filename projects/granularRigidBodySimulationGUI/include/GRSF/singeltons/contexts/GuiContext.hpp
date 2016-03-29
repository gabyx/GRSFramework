// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_singeltons_contexts_GuiContext_hpp
#define GRSF_singeltons_contexts_GuiContext_hpp


#include <memory>
#include <OISMouse.h>
#include <SdkTrays.h>
#include "GRSF/singeltons/contexts/RenderContext.hpp"
#include "GRSF/singeltons/contexts/InputContext.hpp"


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
