/*
 *  OrbitCamera.cpp
 *
 *  Created by Gabriel Nützi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */

 #include <GRSF/App/OrbitCamera.hpp>

#include "GRSF/Singeltons/Contexts/InputContext.hpp"

#include "GRSF/Common/LogDefines.hpp"

using namespace Ogre;
using namespace std;

OrbitCamera::OrbitCamera(SceneManager * pSceneMgr, Ogre::String name, double rotate_speed, double translate_speed, double r_init, double phi_init,double theta_init)
{

	m_pSceneMgr = pSceneMgr;
	m_Name = name;

	m_pCamera = m_pSceneMgr->createCamera(m_Name);

   // RenderContext::getSingleton().m_pViewport->setBackgroundColour(ColourValue(0.8f, 0.7f, 0.6f, 1.0f));

	m_pCamera->setAspectRatio(Real(RenderContext::getSingleton().m_pViewport->getActualWidth()) /
							  Real(RenderContext::getSingleton().m_pViewport->getActualHeight()));
	m_pCamera->setNearClipDistance((Ogre::Real)1);
	RenderContext::getSingleton().m_pViewport->setCamera(m_pCamera);

    m_Rotate = (Ogre::Real)rotate_speed;
    m_Move = (Ogre::Real)translate_speed;
    m_CamVelocityDirection = Vector3::ZERO;
	m_r_init = r_init;
	m_phi_init = phi_init;
	m_theta_init = theta_init;

    m_OrbitNodeIndex = 0;

	enableInput();

	createScene();
}

OrbitCamera::~OrbitCamera()
{
  DECONSTRUCTOR_MESSAGE
   disableInput();
	m_pSceneMgr->destroyCamera(m_pCamera);
}

void OrbitCamera::setActive(){
	RenderContext::getSingleton().m_pViewport->setCamera(m_pCamera);
}

void OrbitCamera::enableInput(){
	::InputContext::getSingleton().addKeyListener(this, m_Name);
	::InputContext::getSingleton().addMouseListener(this, m_Name);
}
void OrbitCamera::disableInput(){
	::InputContext::getSingleton().removeKeyListener( m_Name);
	::InputContext::getSingleton().removeMouseListener( m_Name);
}

void OrbitCamera::moveOrbitToNode(SceneNode * const Obj)
{


    bool OrbitNode_attached = false;

    Vector3 I_t_I_Cam;
    Quaternion q_I_Cam;

    if(m_pCamNode->getParentSceneNode() != m_pOrbitNode){
        cout << "Note: m_pCamNode is NOT attached to m_pOrbitNode"<<endl;
    }

    I_t_I_Cam = m_pCamNode->convertLocalToWorldPosition(Vector3::ZERO);
    q_I_Cam = m_pCamNode->convertLocalToWorldOrientation(Quaternion::IDENTITY);



    // move Orbit to new Obj

    //If m_pOrbitNode is attached somewhere, then detach , move, attach
    if (m_pOrbitNode->getParentSceneNode() != m_pSceneMgr->getRootSceneNode()){
        // m_pOrbitNode is attached, detach OrbitNode!
        OrbitNode_attached = true;
        attachDetachOrbitToNode();
    }

    m_pOrbitNode->setPosition(Obj->convertLocalToWorldPosition(Vector3::ZERO));
    m_pOrbitNode->resetOrientation(); // similar to WorldFrame (IDENTITY)
    cout << "OrbitNode moved to "<< m_OrbitNodeList[m_OrbitNodeIndex]->getName() <<endl;

    // move cam back to original position where it was!
    //cout << "Repositionate camera to original position"<<endl;
    Vector3 I_t_Orbit_Cam = I_t_I_Cam - m_pOrbitNode->getPosition();
    //turn Orbit(IDENTITY orientation) in Cam direction
    double phi = atan2(I_t_Orbit_Cam[1],I_t_Orbit_Cam[0]);
    double theta = atan2 (Vector2(I_t_Orbit_Cam[1],I_t_Orbit_Cam[0]).length() , I_t_Orbit_Cam[2] );

    m_pOrbitNode->roll(Radian((Ogre::Real)phi));
    Quaternion q_I_Orbit = m_pOrbitNode->convertLocalToWorldOrientation(Quaternion::IDENTITY);

    Quaternion q_Orbit_Cam = Quaternion(Radian((Ogre::Real)theta),Vector3(0,1,0))*Quaternion(Radian((Ogre::Real)M_PI/2),Vector3(0,0,1));

    Vector3 Orbit_t_Orbit_Cam = q_I_Orbit.Inverse() * I_t_Orbit_Cam;

    m_pCamNode->setPosition(Orbit_t_Orbit_Cam);
    m_pCamNode->setOrientation(q_Orbit_Cam);


    //OrbitNode was attached
    if (OrbitNode_attached){
        attachDetachOrbitToNode();
    }

}

void OrbitCamera::attachDetachOrbitToNode()
{

    SceneNode * Obj;

    if (m_pOrbitNode->getParentSceneNode() == m_pSceneMgr->getRootSceneNode())
    {
        //Attach OrbitNode to current Obj
        //cout << "OrbitNode detaching from Root"<<endl;
        Obj = m_OrbitNodeList[m_OrbitNodeIndex];



        // Repositionate Camera
        Vector3 I_t_I_Orbit = m_pOrbitNode->convertLocalToWorldPosition(Vector3::ZERO);
        Quaternion q_I_Orbit = m_pOrbitNode->convertLocalToWorldOrientation(Quaternion::IDENTITY);

        Vector3 I_t_I_Obj = Obj->convertLocalToWorldPosition(Vector3::ZERO);
        Quaternion q_I_Obj = Obj->convertLocalToWorldOrientation(Quaternion::IDENTITY);

        Quaternion q_Obj_Orbit = q_I_Obj.Inverse() * q_I_Orbit;
        Vector3 Obj_t_Obj_Orbit = q_I_Obj.Inverse() * (I_t_I_Orbit - I_t_I_Obj );


        m_pSceneMgr->getRootSceneNode()->removeChild(m_pOrbitNode);
        Obj->addChild(m_pOrbitNode);
        //Repositionate m_pOrbitNode
        m_pOrbitNode->setPosition(Obj_t_Obj_Orbit);
        m_pOrbitNode->setOrientation(q_Obj_Orbit);
        cout << "OrbitNode is now attached to "<<m_OrbitNodeList[m_OrbitNodeIndex]->getName()<<endl;
    }
    else
    {
        //Detach OrbitNode to current Object
        SceneNode* ParentNode = m_pOrbitNode->getParentSceneNode();
        //cout << "OrbitNode detaching from "<< ParentNode->getName()<<endl;

        Vector3 I_t_I_Orbit = m_pOrbitNode->convertLocalToWorldPosition(Vector3::ZERO);
        Quaternion q_I_Orbit = m_pOrbitNode->convertLocalToWorldOrientation(Quaternion::IDENTITY);

        ParentNode->removeChild(m_pOrbitNode);
        m_pSceneMgr->getRootSceneNode()->addChild(m_pOrbitNode);
        //Repositionate m_pOrbitNode
        m_pOrbitNode->setPosition(I_t_I_Orbit);
        m_pOrbitNode->setOrientation(q_I_Orbit);
        cout << "OrbitNode is now attached to Root"<<endl;
    }

}

void OrbitCamera::createScene()
{

    //Create Orbit Axes
	//Ogre::Entity* ent = m_pSceneMgr->createEntity("OrbitAxes", "axes.mesh");
	m_pOrbitNode = m_pSceneMgr->getRootSceneNode()->createChildSceneNode("OrbitNode");
   //m_pOrbitNode->attachObject(ent);

    // Create the Real Cam Node
    //ent = m_pSceneMgr->createEntity("CamNode", "axes.mesh");
    m_pCamNode =  m_pOrbitNode->createChildSceneNode("CamNode");
    //m_pCamNode->attachObject(ent);
    double phi = m_phi_init;
    double theta = m_theta_init;
    double r = m_r_init;
    m_pCamNode->rotate(Vector3(0,0,1),Radian((Ogre::Real)phi),Node::TS_LOCAL);
    m_pCamNode->rotate(Vector3(0,1,0),Radian((Ogre::Real)theta),Node::TS_LOCAL);
    m_pCamNode->translate(Vector3(0,0,(Ogre::Real)r),Node::TS_LOCAL);
    m_pCamNode->rotate(Vector3(0,0,1),Radian((Ogre::Real)M_PI/2),Node::TS_LOCAL);

    // Attach Camera and Light
    m_pCamNode->attachObject(m_pCamera);
    //m_pCamNode->attachObject(lightPtr);

}


void OrbitCamera::update(double timeSinceLastFrame)
{

	if (::InputContext::getSingleton().getKeyboard()->isKeyDown(OIS::KC_LSHIFT))
    {
        m_pOrbitNode->translate(m_CamVelocityDirection * (Ogre::Real)timeSinceLastFrame, Node::TS_LOCAL);

    }
    else
    {
        m_pCamNode->translate( m_CamVelocityDirection * (Ogre::Real)timeSinceLastFrame, Node::TS_LOCAL);

    }
    return;
}


// MouseListener
bool OrbitCamera::mouseMoved(const OIS::MouseEvent &e)
{


    if (e.state.buttonDown(OIS::MB_Right) && !::InputContext::getSingleton().getKeyboard()->isKeyDown(OIS::KC_LSHIFT))
    {
        m_pCamNode->roll(Degree(-m_Rotate * e.state.X.rel), Node::TS_WORLD);
        m_pCamNode->pitch(Degree(-m_Rotate * e.state.Y.rel), Node::TS_LOCAL);
    }

    if (e.state.buttonDown(OIS::MB_Left) && !::InputContext::getSingleton().getKeyboard()->isKeyDown(OIS::KC_LSHIFT) )
    {
        m_pOrbitNode->roll(Degree(-m_Rotate * e.state.X.rel), Node::TS_PARENT);
        m_pOrbitNode->yaw(Degree(-m_Rotate * e.state.Y.rel), Node::TS_LOCAL);
    }

    if (e.state.buttonDown(OIS::MB_Left)  &&  ::InputContext::getSingleton().getKeyboard()->isKeyDown(OIS::KC_LSHIFT))
    {
        m_pCamNode->translate( Vector3(0,0,-1) * m_Move * (Ogre::Real)::InputContext::getSingleton().getMouse()->getMouseState().Y.rel / 100.0, Node::TS_LOCAL);
    }


    return true;
}

bool OrbitCamera::keyPressed(const OIS::KeyEvent &e)
{
    switch (e.key)
    {
    case OIS::KC_X:
        if (m_OrbitNodeList.size()>0 && m_OrbitNodeIndex+1 <= m_OrbitNodeList.size()-1 )
        {
            m_OrbitNodeIndex++;
        }
        moveOrbitToNode(m_OrbitNodeList[m_OrbitNodeIndex]);
        break;

    case OIS::KC_Y:

        if (m_OrbitNodeList.size()>0 && m_OrbitNodeIndex-1 >= 0)
        {
            m_OrbitNodeIndex--;
        }
        moveOrbitToNode(m_OrbitNodeList[m_OrbitNodeIndex]);
        break;

    case OIS::KC_C:
        attachDetachOrbitToNode();
        break;
    //case OIS::KC_UP:
    case OIS::KC_W:
        m_CamVelocityDirection.z = -m_Move;
        break;

    //case OIS::KC_DOWN:
    case OIS::KC_S:
        m_CamVelocityDirection.z = +m_Move;
        break;

    //case OIS::KC_LEFT:
    case OIS::KC_A:
        m_CamVelocityDirection.x = -m_Move;
        break;

    //case OIS::KC_RIGHT:
    case OIS::KC_D:
        m_CamVelocityDirection.x = +m_Move;
        break;

    //case OIS::KC_PGDOWN:
    case OIS::KC_E:
        m_CamVelocityDirection.y = -m_Move;
        break;

    //case OIS::KC_PGUP:
    case OIS::KC_Q:
        m_CamVelocityDirection.y = +m_Move;
        break;

    default:
        break;
    }
    return true;
}

bool OrbitCamera::keyReleased(const OIS::KeyEvent &e)
{
    switch (e.key)
    {
    case OIS::KC_UP:
    case OIS::KC_W:
    case OIS::KC_DOWN:
    case OIS::KC_S:
        m_CamVelocityDirection.z = 0;
        break;

    case OIS::KC_LEFT:
    case OIS::KC_A:
    case OIS::KC_RIGHT:
    case OIS::KC_D:
        m_CamVelocityDirection.x = 0;
        break;

    case OIS::KC_PGDOWN:
    case OIS::KC_E:
    case OIS::KC_PGUP:
    case OIS::KC_Q:
        m_CamVelocityDirection.y = 0;
        break;
    default:
        break;
    }


    return true;
}
