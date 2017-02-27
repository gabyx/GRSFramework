// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/app/MenuMouse.hpp"

MenuMouse::MenuMouse(std::shared_ptr<OgreBites::SdkTrayManager> trayManager, Ogre::String name)
{
    m_pTrayMgr = trayManager;
    m_Name     = name;
}

MenuMouse::~MenuMouse()
{
    setInactive();
}

bool MenuMouse::mouseMoved(const OIS::MouseEvent& evt)
{
    if (m_pTrayMgr->injectMouseMove(evt))
        return true;

    return true;
}

bool MenuMouse::mousePressed(const OIS::MouseEvent& evt, OIS::MouseButtonID id)
{
    if (m_pTrayMgr->injectMouseDown(evt, id))
        return true;

    return true;
}

bool MenuMouse::mouseReleased(const OIS::MouseEvent& evt, OIS::MouseButtonID id)
{
    if (m_pTrayMgr->injectMouseUp(evt, id))
        return true;

    return true;
}

void MenuMouse::setActive()
{
    ::InputContext::getSingleton().addKeyListener(this, m_Name);
    ::InputContext::getSingleton().addMouseListener(this, m_Name);
}
void MenuMouse::setInactive()
{
    ::InputContext::getSingleton().removeKeyListener(this);
    ::InputContext::getSingleton().removeMouseListener(this);
}
