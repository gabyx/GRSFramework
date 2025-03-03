// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/singeltons/contexts/InputContext.hpp"

#include "GRSF/common/Asserts.hpp"
#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/foreach_macro.hpp"

//=========================================================

//=========================================================

template <>
InputContext* Ogre::Singleton<InputContext>::msSingleton = 0;

//=========================================================

InputContext::InputContext(void) : m_pMouse(0), m_pKeyboard(0), mInputSystem(0)
{
}

InputContext::~InputContext(void)
{
    DESTRUCTOR_MESSAGE

    if (mInputSystem)
    {
        if (m_pMouse)
        {
            mInputSystem->destroyInputObject(m_pMouse);
            m_pMouse = 0;
        }

        if (m_pKeyboard)
        {
            mInputSystem->destroyInputObject(m_pKeyboard);
            m_pKeyboard = 0;
        }

        if (m_Joysticks.size() > 0)
        {
            itJoystick    = m_Joysticks.begin();
            itJoystickEnd = m_Joysticks.end();
            for (; itJoystick != itJoystickEnd; ++itJoystick)
            {
                mInputSystem->destroyInputObject(*itJoystick);
            }

            m_Joysticks.clear();
        }

        // If you use OIS1.0RC1 or above, uncomment this line
        // and comment the line below it
        mInputSystem->destroyInputSystem(mInputSystem);
        // mInputSystem->destroyInputSystem();
        mInputSystem = 0;

        // Clear Listeners
        mKeyListeners.clear();
        mMouseListeners.clear();
        mJoystickListeners.clear();
    }
}

bool InputContext::initialise()
{
    using namespace std;
    if (!mInputSystem)
    {
        // Setup basic variables
        OIS::ParamList paramList;
        std::size_t windowHnd = 0;
        std::ostringstream windowHndStr;

        // Get window handle
        RenderContext::getSingleton().m_pRenderWnd->getCustomAttribute("WINDOW", &windowHnd);

        // Fill parameter list
        windowHndStr << (unsigned int)windowHnd;
        paramList.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));

// Set  the mouse/keyboard exclusivity
#if defined __unix__ || defined __unix
        // mouse
        paramList.insert(std::make_pair("x11_mouse_grab", "false"));
        // keyboard
        paramList.insert(std::make_pair("x11_keyboard_grab", "false"));
#elif defined _WIN32 || defined _WIN64
// mouse
//            paramList.insert( make_pair("w32_mouse", "DISCL_FOREGROUND") );
//            paramList.insert( make_pair("w32_mouse", "DISCL_NONEXCLUSIVE") );
//
//            //keyboard
//            paramList.insert( make_pair("w32_keyboard", "DISCL_FOREGROUND") );
//            paramList.insert( make_pair("w32_keyboard", "DISCL_NONEXCLUSIVE") );
//            paramList.insert( make_pair("w32_keyboard", "DISCL_NOWINKEY") );
//
//            //joystick/gamepad
//            paramList.insert( make_pair("w32_joystick", "DISCL_FOREGROUND") );
//            paramList.insert( make_pair("w32_joystick", "DISCL_NONEXCLUSIVE") );
#endif

        // Create inputsystem
        mInputSystem = OIS::InputManager::createInputSystem(paramList);

        // If possible create a buffered keyboard
        // (note: if below line doesn't compile, try:  if (mInputSystem->getNumberOfDevices(OIS::OISKeyboard) > 0) {
        // if( mInputSystem->numKeyboards() > 0 ) {
        if (mInputSystem->getNumberOfDevices(OIS::OISKeyboard) > 0)
        {
            cout << "InputContext:: Found " << mInputSystem->getNumberOfDevices(OIS::OISKeyboard) << " Keyboard"
                 << endl;
            m_pKeyboard = static_cast<OIS::Keyboard*>(mInputSystem->createInputObject(OIS::OISKeyboard, true));
            m_pKeyboard->setEventCallback(this);
        }

        // If possible create a buffered mouse
        // (note: if below line doesn't compile, try:  if (mInputSystem->getNumberOfDevices(OIS::OISMouse) > 0) {
        // if( mInputSystem->numMice() > 0 ) {
        if (mInputSystem->getNumberOfDevices(OIS::OISMouse) > 0)
        {
            cout << "InputContext:: Found " << mInputSystem->getNumberOfDevices(OIS::OISMouse) << " Mouse" << endl;
            m_pMouse = static_cast<OIS::Mouse*>(mInputSystem->createInputObject(OIS::OISMouse, true));
            m_pMouse->setEventCallback(this);

            // Get window size
            unsigned int width, height, depth;
            int left, top;
            RenderContext::getSingleton().m_pRenderWnd->getMetrics(width, height, depth, left, top);

            // Set mouse region
            this->setWindowExtents(width, height);
        }

        // If possible create all joysticks in buffered mode
        // (note: if below line doesn't compile, try:  if (mInputSystem->getNumberOfDevices(OIS::OISJoyStick) > 0) {
        // if( mInputSystem->numJoySticks() > 0 ) {
        if (mInputSystem->getNumberOfDevices(OIS::OISJoyStick) > 0)
        {
            // m_Joysticks.resize( mInputSystem->numJoySticks() );
            cout << "InputContext:: Found " << mInputSystem->getNumberOfDevices(OIS::OISJoyStick) << " Joystick"
                 << endl;
            m_Joysticks.resize(mInputSystem->getNumberOfDevices(OIS::OISJoyStick));

            itJoystick    = m_Joysticks.begin();
            itJoystickEnd = m_Joysticks.end();
            for (; itJoystick != itJoystickEnd; ++itJoystick)
            {
                (*itJoystick) = static_cast<OIS::JoyStick*>(mInputSystem->createInputObject(OIS::OISJoyStick, true));
                (*itJoystick)->setEventCallback(this);
            }
        }
        return true;
    }
    return false;
}

void InputContext::capture(void)
{
    // Need to capture / update each device every frame
    if (m_pMouse)
    {
        m_pMouse->capture();
    }

    if (m_pKeyboard)
    {
        m_pKeyboard->capture();
    }

    if (m_Joysticks.size() > 0)
    {
        itJoystick    = m_Joysticks.begin();
        itJoystickEnd = m_Joysticks.end();
        for (; itJoystick != itJoystickEnd; ++itJoystick)
        {
            (*itJoystick)->capture();
        }
    }
}

void InputContext::addKeyListener(OIS::KeyListener* keyListener, const std::string& instanceName)
{
    if (m_pKeyboard)
    {
        // Check for duplicate items
        itKeyListener = mKeyListeners.find(instanceName);
        if (itKeyListener == mKeyListeners.end())
        {
            mKeyListeners[instanceName] = keyListener;
        }
        else
        {
            // Duplicate Item
            if (itKeyListener->second != keyListener)
            {
                GRSF_ASSERTMSG(
                    false,
                    "InputContext::addKeyListener dublicate name: " << instanceName
                                                                    << " is already taken by another key listener!");
            }
        }
    }
}

void InputContext::addMouseListener(OIS::MouseListener* mouseListener, const std::string& instanceName)
{
    if (m_pMouse)
    {
        // Check for duplicate items
        itMouseListener = mMouseListeners.find(instanceName);
        if (itMouseListener == mMouseListeners.end())
        {
            mMouseListeners[instanceName] = mouseListener;
        }
        else
        {
            // Duplicate Item
            if (itMouseListener->second != mouseListener)
            {
                GRSF_ASSERTMSG(false,
                               "InputContext::addMouseListener dublicate name: "
                                   << instanceName
                                   << " is already taken by another mouse listener!");
            }
        }
    }
}

void InputContext::addJoystickListener(OIS::JoyStickListener* joystickListener, const std::string& instanceName)
{
    if (m_Joysticks.size() > 0)
    {
        // Check for duplicate items
        itJoystickListener = mJoystickListeners.find(instanceName);
        if (itJoystickListener == mJoystickListeners.end())
        {
            mJoystickListeners[instanceName] = joystickListener;
        }
        else
        {
            // Duplicate Item
            if (itJoystickListener->second != joystickListener)
            {
                GRSF_ASSERTMSG(false,
                               "InputContext::addJoystickListener dublicate name: "
                                   << instanceName
                                   << " is already taken by another joystick listener!");
            }
        }
    }
}

void InputContext::removeKeyListener(const std::string& instanceName)
{
    // Check if item exists
    itKeyListener = mKeyListeners.find(instanceName);
    if (itKeyListener != mKeyListeners.end())
    {
        mKeyListeners.erase(itKeyListener);
    }
    else
    {
        // Doesn't Exist
    }
}

void InputContext::removeMouseListener(const std::string& instanceName)
{
    // Check if item exists
    itMouseListener = mMouseListeners.find(instanceName);
    if (itMouseListener != mMouseListeners.end())
    {
        mMouseListeners.erase(itMouseListener);
    }
    else
    {
        // Doesn't Exist
    }
}

void InputContext::removeJoystickListener(const std::string& instanceName)
{
    // Check if item exists
    itJoystickListener = mJoystickListeners.find(instanceName);
    if (itJoystickListener != mJoystickListeners.end())
    {
        mJoystickListeners.erase(itJoystickListener);
    }
    else
    {
        // Doesn't Exist
    }
}

void InputContext::removeKeyListener(OIS::KeyListener* keyListener)
{
    itKeyListener    = mKeyListeners.begin();
    itKeyListenerEnd = mKeyListeners.end();
    for (; itKeyListener != itKeyListenerEnd; ++itKeyListener)
    {
        if (itKeyListener->second == keyListener)
        {
            mKeyListeners.erase(itKeyListener);
            break;
        }
    }
}

void InputContext::removeMouseListener(OIS::MouseListener* mouseListener)
{
    itMouseListener    = mMouseListeners.begin();
    itMouseListenerEnd = mMouseListeners.end();
    for (; itMouseListener != itMouseListenerEnd; ++itMouseListener)
    {
        if (itMouseListener->second == mouseListener)
        {
            mMouseListeners.erase(itMouseListener);
            break;
        }
    }
}

void InputContext::removeJoystickListener(OIS::JoyStickListener* joystickListener)
{
    itJoystickListener    = mJoystickListeners.begin();
    itJoystickListenerEnd = mJoystickListeners.end();
    for (; itJoystickListener != itJoystickListenerEnd; ++itJoystickListener)
    {
        if (itJoystickListener->second == joystickListener)
        {
            mJoystickListeners.erase(itJoystickListener);
            break;
        }
    }
}

void InputContext::removeAllListeners(void)
{
    mKeyListeners.clear();
    mMouseListeners.clear();
    mJoystickListeners.clear();
}

void InputContext::removeAllKeyListeners(void)
{
    mKeyListeners.clear();
}

void InputContext::removeAllMouseListeners(void)
{
    mMouseListeners.clear();
}

void InputContext::removeAllJoystickListeners(void)
{
    mJoystickListeners.clear();
}

void InputContext::setWindowExtents(int width, int height)
{
    // Set mouse region (if window resizes, we should alter this to reflect as well)
    const OIS::MouseState& mouseState = m_pMouse->getMouseState();
    mouseState.width                  = width;
    mouseState.height                 = height;
}

OIS::Mouse* InputContext::getMouse(void)
{
    return m_pMouse;
}

OIS::Keyboard* InputContext::getKeyboard(void)
{
    return m_pKeyboard;
}

OIS::JoyStick* InputContext::getJoystick(unsigned int index)
{
    // Make sure it's a valid index
    if (index < m_Joysticks.size())
    {
        return m_Joysticks[index];
    }

    return 0;
}

OgreBites::InputContext InputContext::getInputContext()
{
    OgreBites::InputContext ic;
    ic.mMouse    = getMouse();
    ic.mKeyboard = getKeyboard();
    return ic;
}

std::size_t InputContext::getNumOfJoysticks(void)
{
    return m_Joysticks.size();
}

bool InputContext::keyPressed(const OIS::KeyEvent& e)
{
    // this loop needs to be special because, the function keyPressed can add/remove elements which invalidate
    // iterators!!
    // But the ->keyPressed but for actions like adding and removing in the keyPressed function should return false!!
    // that the loop breaks!

    for (auto itKeyListener = mKeyListeners.begin(), it_next = itKeyListener, it_end = mKeyListeners.end();
         itKeyListener != it_end;
         itKeyListener = it_next)
    {
        ++it_next;  // because we have a std::map iterators are valid if something is erased!

        if (!itKeyListener->second->keyPressed(e))
            break;
    }

    return true;
}

bool InputContext::keyReleased(const OIS::KeyEvent& e)
{
    for (auto itKeyListener = mKeyListeners.begin(), it_next = itKeyListener, it_end = mKeyListeners.end();
         itKeyListener != it_end;
         itKeyListener = it_next)
    {
        ++it_next;

        if (!itKeyListener->second->keyReleased(e))
            break;
    }

    return true;
}

bool InputContext::mouseMoved(const OIS::MouseEvent& e)
{
    itMouseListener    = mMouseListeners.begin();
    itMouseListenerEnd = mMouseListeners.end();
    for (; itMouseListener != itMouseListenerEnd; ++itMouseListener)
    {
        if (!itMouseListener->second->mouseMoved(e))
            break;
    }

    return true;
}

bool InputContext::mousePressed(const OIS::MouseEvent& e, OIS::MouseButtonID id)
{
    for (auto itMouseListener = mMouseListeners.begin(), it_next = itMouseListener, it_end = mMouseListeners.end();
         itMouseListener != it_end;
         itMouseListener = it_next)
    {
        ++it_next;

        if (!itMouseListener->second->mousePressed(e, id))
            break;
    }

    return true;
}

bool InputContext::mouseReleased(const OIS::MouseEvent& e, OIS::MouseButtonID id)
{
    for (auto itMouseListener = mMouseListeners.begin(), it_next = itMouseListener, it_end = mMouseListeners.end();
         itMouseListener != it_end;
         itMouseListener = it_next)
    {
        ++it_next;

        if (!itMouseListener->second->mouseReleased(e, id))
            break;
    }

    return true;
}

bool InputContext::povMoved(const OIS::JoyStickEvent& e, int pov)
{
    itJoystickListener    = mJoystickListeners.begin();
    itJoystickListenerEnd = mJoystickListeners.end();
    for (; itJoystickListener != itJoystickListenerEnd; ++itJoystickListener)
    {
        if (!itJoystickListener->second->povMoved(e, pov))
            break;
    }

    return true;
}

bool InputContext::axisMoved(const OIS::JoyStickEvent& e, int axis)
{
    itJoystickListener    = mJoystickListeners.begin();
    itJoystickListenerEnd = mJoystickListeners.end();
    for (; itJoystickListener != itJoystickListenerEnd; ++itJoystickListener)
    {
        if (!itJoystickListener->second->axisMoved(e, axis))
            break;
    }

    return true;
}

bool InputContext::sliderMoved(const OIS::JoyStickEvent& e, int sliderID)
{
    itJoystickListener    = mJoystickListeners.begin();
    itJoystickListenerEnd = mJoystickListeners.end();
    for (; itJoystickListener != itJoystickListenerEnd; ++itJoystickListener)
    {
        if (!itJoystickListener->second->sliderMoved(e, sliderID))
            break;
    }

    return true;
}

bool InputContext::buttonPressed(const OIS::JoyStickEvent& e, int button)
{
    itJoystickListener    = mJoystickListeners.begin();
    itJoystickListenerEnd = mJoystickListeners.end();
    for (; itJoystickListener != itJoystickListenerEnd; ++itJoystickListener)
    {
        if (!itJoystickListener->second->buttonPressed(e, button))
            break;
    }

    return true;
}

bool InputContext::buttonReleased(const OIS::JoyStickEvent& e, int button)
{
    itJoystickListener    = mJoystickListeners.begin();
    itJoystickListenerEnd = mJoystickListeners.end();
    for (; itJoystickListener != itJoystickListenerEnd; ++itJoystickListener)
    {
        if (!itJoystickListener->second->buttonReleased(e, button))
            break;
    }

    return true;
}
