#include "MenuMouse.hpp"

MenuMouse::MenuMouse(boost::shared_ptr<OgreBites::SdkTrayManager> trayManager, Ogre::String name){

	m_pTrayMgr = trayManager;
	m_Name = name;
}

MenuMouse::~MenuMouse(){
   setInactive();
}

bool MenuMouse::mouseMoved(const OIS::MouseEvent &evt)
{
	if (m_pTrayMgr->injectMouseMove(evt)) return true;

	
	return true;
}


bool MenuMouse::mousePressed(const OIS::MouseEvent &evt, OIS::MouseButtonID id)
{
	if (m_pTrayMgr->injectMouseDown(evt, id)) return true;

	return true;
}


bool MenuMouse::mouseReleased(const OIS::MouseEvent &evt, OIS::MouseButtonID id)
{
	if (m_pTrayMgr->injectMouseUp(evt, id)) return true;

	return true;
}



void MenuMouse::setActive(){
	InputContext::getSingletonPtr()->addKeyListener(this, m_Name);
	InputContext::getSingletonPtr()->addMouseListener(this, m_Name);

}
void MenuMouse::setInactive(){
	InputContext::getSingletonPtr()->removeKeyListener(this);
	InputContext::getSingletonPtr()->removeMouseListener(this);

}