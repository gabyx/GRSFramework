/*
 *  GRSF/Dynamics/Collision/CollisionData.hpp
 *
 *  Created by Gabriel Nützi on 19.03.10.
 *  Copyright 2010 -. All rights reserved.
 *
 */

#ifndef GRSF_Dynamics_Collision_CollisionData_hpp
#define GRSF_Dynamics_Collision_CollisionData_hpp


#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <utility>

#include <boost/any.hpp>


#include "GRSF/Common/TypeDefs.hpp"

#include "GRSF/Dynamics/Collision/ContactFrame.hpp"
#include "GRSF/Dynamics/Collision/ContactTag.hpp"

/**
* @ingroup Collision
* @brief This is the CollisionData class which describes one collision contact.
*/
/** @{ */
class CollisionData
{
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CollisionData(){
		  m_r_S1C1.setZero();
		  m_r_S2C2.setZero();
      m_overlap=0;
    }; ///< Constructor which sets all members to zero.

    ~CollisionData(){};

  RigidBodyType *  m_pBody1, * m_pBody2; ///< Two RigidBody pointers of the first and second RigidBody at this contact point.
  PREC m_overlap; ///< The overlap distance in the normal direction.

  /**
  * The contact frame which is always located at the contact point of the first body. The coordinate frame vectors are resolved in the I Frame!
  * @{
  */
  ContactFrame m_cFrame;
  //contains Vector3 m_cFrame.m_e_x, m_cFrame.m_e_y, m_cFrame.m_e_z;

  /** @} */

  /**
  * The distance vector from center of gravity S1 of the first body to the contact point and the same for the second body. The vectors are resolved in the I Frame!
  * @{
  */
  Vector3 m_r_S1C1, m_r_S2C2;
  /** @} */

  ContactTag m_ContactTag; ///< A tag classifying this contact pair. This is used to identify the contact for Percussion Caching.

};
/** @} */
#endif
