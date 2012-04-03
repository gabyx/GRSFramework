/*
*  OrbitCamera.h
*
*  Created by Gabriel Nützi on 21.03.10.
*  Copyright 2010 ETH. All rights reserved.
*
*/


#ifndef OrbitCamera_h
#define OrbitCamera_h

// Includes =================================
#define _USE_MATH_DEFINES
#include <cstdlib>
#include <iostream>
#include <vector>
#include <cmath>
#include <OGRE/Ogre.h>
#include <OIS/OISEvents.h>
#include <OIS/OISKeyboard.h>
#include <OIS/OISJoyStick.h>
#include <OIS/OISMouse.h>
// ==========================================


//using namespace Ogre;
//using namespace std;

/**
* 	@brief This is the OrbitCamera class which creates a camera in the given scene manager which can be manipulated by the user
*	by pressing A,S,D,W and right,left mouse click with mouse movements.
*/
class OrbitCamera : public OIS::KeyListener, OIS::MouseListener {

public:

   /** @brief Constructs an orbiting Camera in the given Scene Manager pSceneMgr.
   * @param pSceneMgr Pointer to en existing Scene Manager of Ogre.
   * @param name The name of the camera.
   * @param rotate_speed The rotation speed of the camera if the left mouse is pressed and moved!
   * @param translate_speed The translation_speed if the Camera is moved when the keys A,D,S,W are pressed.
   * @param r_init, phi_init, theta_init The
   */
   OrbitCamera(Ogre::SceneManager * pSceneMgr,
      Ogre::String name,
      double rotate_speed,
      double translate_speed,
      double r_init,
      double phi_init,
      double theta_init);
   ~OrbitCamera();

   /** \name Orbiting Objects */
   /* @{ */
   /// List of objects where we can orbit around with the camera.
   std::vector<Ogre::SceneNode*> m_OrbitNodeList;
   /// The actual index of the m_OrbitNodeList where m_pOrbitNode is positionated (meaning where the Camera points to)
   int m_OrbitNodeIndex;
   /* @} */

   /// The camera node whree the Ogre camera is attached.
   Ogre::SceneNode * m_pCamNode;

   /// Activates the camera in the viewport.
   void setActive();

   /** \name Disabling/Enabling Event Inputs */
   /* @{ */
   /// Enables the input.
   void enableInput();
   /** @brief Enables input from the Keyboard.
   *	This function is used to disable the input when e.g the mouse or another object should be
   *	receiving only inputs, otherwise the camera will still rotate.
   */
   void disableInput();
   /* @} */

   /// Get the Ogre camera.
   Ogre::Camera * getCamera(){return m_pCamera;};

   /// The update function, which is necessary to call in a render loop to make the camera turn and move.
   void update(double timeSinceLastFrame);

protected:

   Ogre::Camera * m_pCamera; 		///< The Ogre camera.
   Ogre::String m_Name; 		///< The name of the camera.

   Ogre::SceneManager * m_pSceneMgr; ///< The scene manager.

   /** \name Camera Rotation */
   /* @{ */
   Ogre::SceneNode * m_pOrbitNode; 					///< A helper node which rotates only around z-axis.
   Ogre::Real m_Rotate;								///< Rotation speed.
   Ogre::Real m_Move;								///< Translation speed.
   Ogre::Vector3 m_CamVelocityDirection;				///< Velocity direction.
   double m_phi_init, m_r_init, m_theta_init;	///< Initial values for the spherical coordinates to position the camera at beginning.
   /* @} */

   /** @brief Moved the m_pOrbitNode to a new scene node in the m_OrbitNodeList.
   * 	This function moves the m_pOrbitNode around, the m_pCamNode stays where it was located before.
   *	So the camera will no be moved.
   */
   void moveOrbitToNode( Ogre::SceneNode * const Obj);

   /** @brief Attaches the m_pOrbitNode to a scene node in the m_OrbitNodeList.
   * 	Attaching means that the m_pOrbitNode will be moving with selected scene node in the m_OrbitNodeList.
   */
   void attachDetachOrbitToNode();

   virtual void createScene(); ///< Creates the scene with a camera and some axis frames. See the code.

   /** \name KeyListener for keyboard inputs */
   /* @{ */
   virtual bool keyPressed(const OIS::KeyEvent &arg);
   virtual bool keyReleased(const OIS::KeyEvent &arg);
   /* @} */
   /** \name MouseListener for mouse inputs */
   virtual bool mouseMoved(const OIS::MouseEvent &arg);
   virtual bool mousePressed(const OIS::MouseEvent &arg, OIS::MouseButtonID id) {return true; };
   virtual bool mouseReleased(const OIS::MouseEvent &arg, OIS::MouseButtonID id) { return true; };
   /* @} */
   /** \name JoystickListener for joystick inputs */
   virtual bool buttonPressed(const OIS::JoyStickEvent &arg, int button) { return true; };
   virtual bool buttonReleased(const OIS::JoyStickEvent &arg, int button) { return true; };
   virtual bool axisMoved(const OIS::JoyStickEvent &arg, int axis) { return true; };
   /* @} */
};




#endif
