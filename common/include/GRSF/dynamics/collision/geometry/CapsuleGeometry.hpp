// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_collision_geometry_CapsuleGeometry_hpp
#define GRSF_dynamics_collision_geometry_CapsuleGeometry_hpp

#include <boost/serialization/access.hpp>
#include "GRSF/common/TypeDefs.hpp"

class CapsuleGeometry
{
    public:
    DEFINE_MATRIX_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CapsuleGeometry() : /*m_pos(0,0,0),*/ m_normal(1, 0, 0), m_length(1), m_radius(0.5){};
    CapsuleGeometry(/*const Vector3 & pos,*/ const Vector3& n, PREC length, PREC radius)
        : /*m_pos(pos),*/ m_normal(n), m_length(length), m_radius(radius){};

    /*Vector3 m_pos;*/  ///< position vector in K frame of the center of the capsule (not needed so far)
    Vector3 m_normal;   ///< normal of the capsule direction in K frame
    PREC    m_length;   ///< length of the capsule
    PREC    m_radius;   ///< radius of the capsule rounding on top and bottom
    private:
    friend class boost::serialization::access;
};

#endif
