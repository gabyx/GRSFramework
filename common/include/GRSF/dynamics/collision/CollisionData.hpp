// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_collision_CollisionData_hpp
#define GRSF_dynamics_collision_CollisionData_hpp

#include <fstream>
#include <iostream>
#include <map>
#include <utility>
#include <vector>

#include <boost/any.hpp>

#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/dynamics/collision/ContactFrame.hpp"
#include "GRSF/dynamics/collision/ContactTag.hpp"

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

    CollisionData()
    {
        clear();
    };  ///< Constructor which sets all members to zero.

    ~CollisionData(){};

    inline void clear()
    {
        m_r_SC[0].setZero();
        m_r_SC[1].setZero();
        m_overlap = 0;
    };

    RigidBodyType* m_pBody[2] = {
        nullptr, nullptr};  ///< Two RigidBody pointers of the first and second RigidBody at this contact point.
    PREC m_overlap;         ///< The overlap distance in the normal direction.

    /**
    * The contact frame which is always located at the contact point of the first body. The coordinate frame vectors are
    * resolved in the I Frame!
    * @{
    */
    ContactFrame m_cFrame;
    // contains Vector3 m_cFrame.m_e_x, m_cFrame.m_e_y, m_cFrame.m_e_z;

    /** @} */

    /**
    * The distance vector from center of gravity S1 of the first body to the contact point and the same for the second
    * body. The vectors are resolved in the I Frame!
    * @{
    */
    Vector3 m_r_SC[2];
    /** @} */

    ContactTag m_contactTag;  ///< A tag classifying this contact pair. This is used to identify the contact for
                              /// Percussion Caching.
};
/** @} */
#endif
