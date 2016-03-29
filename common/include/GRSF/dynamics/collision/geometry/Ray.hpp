// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_collision_geometry_Ray_hpp
#define GRSF_dynamics_collision_geometry_Ray_hpp


#include "GRSF/common/TypeDefs.hpp"


class Ray {
public:

    DEFINE_MATRIX_TYPES

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Ray(Vector3 &p, Vector3 &d):
	    m_p(p), m_d(d)
    {
        m_d.normalize();
        m_mint = 0.0;
        m_maxt = std::numeric_limits<PREC>::max();
    }

    Ray(void) {
        m_p = Vector3(0.0, 0.0, 0.0);
        m_d = Vector3(0.0, 0.0, 1.0);
        m_mint = 0.0;
        m_maxt = std::numeric_limits<PREC>::max();
    }

    ~Ray(){};

    Vector3 getPointOnRay(double t) const {
        return m_p + (m_d*t);
    }

	Vector3 m_p; ///< start point
	Vector3 m_d; ///< normalized direction vector

	PREC m_mint; ///< min. line parameter t
	PREC m_maxt; ///< max. line parameter t

	// epsilon t to avoid self intersection due to rounding errors
	// if the ray starts exactly on an element
	static const double epsilon_t;

};


#endif //Ray_hpp
