﻿#ifndef INPUTCONTEXT_HPP
#define INPUTCONTEXT_HPP

#include <unordered_map>

#include <memory>
#include <OIS/OISEvents.h>
#include <OIS/OISInputManager.h>
#include <OIS/OISKeyboard.h>
#include <OIS/OISJoyStick.h>
#include <OIS/OISMouse.h>

#include <OGRE/OgreSingleton.h>
#include "Contexts/RenderContext.hpp"


/**
* @ingroup	Contexts
* @brief	InputContext contains application-wide input behaviour.
**/
class InputContext : public Ogre::Singleton<InputContext>, OIS::KeyListener, OIS::MouseListener, OIS::JoyStickListener
{
public:
	InputContext( void );
	~InputContext();

	bool initialise();
    void capture( void );

    void addKeyListener( OIS::KeyListener *keyListener, const std::string& instanceName );
    void addMouseListener( OIS::MouseListener *mouseListener, const std::string& instanceName );
    void addJoystickListener( OIS::JoyStickListener *joystickListener, const std::string& instanceName );

    void removeKeyListener( const std::string& instanceName );
    void removeMouseListener( const std::string& instanceName );
    void removeJoystickListener( const std::string& instanceName );

    void removeKeyListener( OIS::KeyListener *keyListener );
    void removeMouseListener( OIS::MouseListener *mouseListener );
    void removeJoystickListener( OIS::JoyStickListener *joystickListener );

    void removeAllListeners( void );
    void removeAllKeyListeners( void );
    void removeAllMouseListeners( void );
    void removeAllJoystickListeners( void );

    void setWindowExtents( int width, int height );

    OIS::Mouse*    getMouse( void );
    OIS::Keyboard* getKeyboard( void );
    OIS::JoyStick* getJoystick( unsigned int index );

    int getNumOfJoysticks( void );



private:

	OIS::Mouse        *m_pMouse;
    OIS::Keyboard     *m_pKeyboard;
	std::vector<OIS::JoyStick*> m_Joysticks;

	OIS::InputManager *mInputSystem;

    InputContext( const InputContext& ) { }
    InputContext & operator = ( const InputContext& );

    bool keyPressed( const OIS::KeyEvent &e );
    bool keyReleased( const OIS::KeyEvent &e );

    bool mouseMoved( const OIS::MouseEvent &e );
    bool mousePressed( const OIS::MouseEvent &e, OIS::MouseButtonID id );
    bool mouseReleased( const OIS::MouseEvent &e, OIS::MouseButtonID id );

    bool povMoved( const OIS::JoyStickEvent &e, int pov );
    bool axisMoved( const OIS::JoyStickEvent &e, int axis );
    bool sliderMoved( const OIS::JoyStickEvent &e, int sliderID );
    bool buttonPressed( const OIS::JoyStickEvent &e, int button );
    bool buttonReleased( const OIS::JoyStickEvent &e, int button );



    std::vector<OIS::JoyStick*>::iterator itJoystick;
    std::vector<OIS::JoyStick*>::iterator itJoystickEnd;

    std::unordered_map<std::string, OIS::KeyListener*> mKeyListeners;
    std::unordered_map<std::string, OIS::MouseListener*> mMouseListeners;
    std::unordered_map<std::string, OIS::JoyStickListener*> mJoystickListeners;

    std::unordered_map<std::string, OIS::KeyListener*>::iterator itKeyListener;
    std::unordered_map<std::string, OIS::MouseListener*>::iterator itMouseListener;
    std::unordered_map<std::string, OIS::JoyStickListener*>::iterator itJoystickListener;

    std::unordered_map<std::string, OIS::KeyListener*>::iterator itKeyListenerEnd;
    std::unordered_map<std::string, OIS::MouseListener*>::iterator itMouseListenerEnd;
    std::unordered_map<std::string, OIS::JoyStickListener*>::iterator itJoystickListenerEnd;

};

//=========================================================

#endif	// INPUTCONTEXT_HPP
