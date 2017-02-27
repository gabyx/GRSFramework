// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_singeltons_contexts_InputContext_hpp
#define GRSF_singeltons_contexts_InputContext_hpp

#include <unordered_map>

#include <OIS/OISEvents.h>
#include <OIS/OISInputManager.h>
#include <OIS/OISJoyStick.h>
#include <OIS/OISKeyboard.h>
#include <OIS/OISMouse.h>
#include <memory>

#include <OGRE/InputContext.h>
#include <OGRE/OgreSingleton.h>
#include "GRSF/singeltons/contexts/RenderContext.hpp"

/**
* @ingroup	Contexts
* @brief	InputContext contains application-wide input behaviour.
**/
class InputContext : public Ogre::Singleton<InputContext>, OIS::KeyListener, OIS::MouseListener, OIS::JoyStickListener
{
public:
    InputContext(void);
    ~InputContext();

    bool initialise();
    void capture(void);

    void addKeyListener(OIS::KeyListener* keyListener, const std::string& instanceName);
    void addMouseListener(OIS::MouseListener* mouseListener, const std::string& instanceName);
    void addJoystickListener(OIS::JoyStickListener* joystickListener, const std::string& instanceName);

    void removeKeyListener(const std::string& instanceName);
    void removeMouseListener(const std::string& instanceName);
    void removeJoystickListener(const std::string& instanceName);

    void removeKeyListener(OIS::KeyListener* keyListener);
    void removeMouseListener(OIS::MouseListener* mouseListener);
    void removeJoystickListener(OIS::JoyStickListener* joystickListener);

    void removeAllListeners(void);
    void removeAllKeyListeners(void);
    void removeAllMouseListeners(void);
    void removeAllJoystickListeners(void);

    void setWindowExtents(int width, int height);

    OIS::Mouse*    getMouse(void);
    OIS::Keyboard* getKeyboard(void);
    OIS::JoyStick* getJoystick(unsigned int index);

    std::size_t getNumOfJoysticks(void);

    OgreBites::InputContext getInputContext();

private:
    OIS::Mouse*                 m_pMouse    = nullptr;
    OIS::Keyboard*              m_pKeyboard = nullptr;
    std::vector<OIS::JoyStick*> m_Joysticks;

    OIS::InputManager* mInputSystem = nullptr;

    InputContext(const InputContext&)
    {
    }
    InputContext& operator=(const InputContext&);

    bool keyPressed(const OIS::KeyEvent& e);
    bool keyReleased(const OIS::KeyEvent& e);

    bool mouseMoved(const OIS::MouseEvent& e);
    bool mousePressed(const OIS::MouseEvent& e, OIS::MouseButtonID id);
    bool mouseReleased(const OIS::MouseEvent& e, OIS::MouseButtonID id);

    bool povMoved(const OIS::JoyStickEvent& e, int pov);
    bool axisMoved(const OIS::JoyStickEvent& e, int axis);
    bool sliderMoved(const OIS::JoyStickEvent& e, int sliderID);
    bool buttonPressed(const OIS::JoyStickEvent& e, int button);
    bool buttonReleased(const OIS::JoyStickEvent& e, int button);

    std::vector<OIS::JoyStick*>::iterator itJoystick;
    std::vector<OIS::JoyStick*>::iterator itJoystickEnd;

    std::unordered_map<std::string, OIS::KeyListener*>      mKeyListeners;
    std::unordered_map<std::string, OIS::MouseListener*>    mMouseListeners;
    std::unordered_map<std::string, OIS::JoyStickListener*> mJoystickListeners;

    std::unordered_map<std::string, OIS::KeyListener*>::iterator      itKeyListener;
    std::unordered_map<std::string, OIS::MouseListener*>::iterator    itMouseListener;
    std::unordered_map<std::string, OIS::JoyStickListener*>::iterator itJoystickListener;

    std::unordered_map<std::string, OIS::KeyListener*>::iterator      itKeyListenerEnd;
    std::unordered_map<std::string, OIS::MouseListener*>::iterator    itMouseListenerEnd;
    std::unordered_map<std::string, OIS::JoyStickListener*>::iterator itJoystickListenerEnd;
};

//=========================================================

#endif  // INPUTCONTEXT_HPP
