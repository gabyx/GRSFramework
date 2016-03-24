// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_collision_geometry_BoxGeometry_hpp
#define GRSF_dynamics_collision_geometry_BoxGeometry_hpp

#include "GRSF/common/TypeDefs.hpp"
#include <boost/serialization/access.hpp>

class BoxGeometry {
public:

    DEFINE_MATRIX_TYPES

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BoxGeometry() {
        m_extent.setZero();
        m_center.setZero();
    };
    BoxGeometry(Vector3 center, Vector3 extent): m_extent(extent), m_center(center) {};

    Vector3 m_extent; ///< Vector of the extend of the box in all directions.
    Vector3 m_center; ///< Vector to the center of the box in body frame! (mostly zero).

    /** Returns K_r_CenterP */
    Vector3 getPoint(unsigned int i) const{
        Vector3 point;
        point(0) = 0.5 * BoxGeometry::m_pointIdx[3*i]*m_extent(0);
        point(1) = 0.5 * BoxGeometry::m_pointIdx[3*i+1]*m_extent(1);
        point(2) = 0.5 * BoxGeometry::m_pointIdx[3*i+2]*m_extent(2);
        return point;
    }

private:
    friend class boost::serialization::access;
    static char m_pointIdx[8*3];
};

#endif
