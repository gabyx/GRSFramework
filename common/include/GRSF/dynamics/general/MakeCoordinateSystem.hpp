// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_MakeCoordinateSystem_hpp
#define GRSF_dynamics_general_MakeCoordinateSystem_hpp

#include <cmath>

#include "GRSF/common/TypeDefs.hpp"

namespace CoordinateSystem {

DEFINE_LAYOUT_CONFIG_TYPES

/**
* @ingroup Common
* @defgroup MakeCoordinateSystem Make Coordinate System
* @brief This is a helper function to make a orthogonal normed right-hand coordinate system from an input vector.
*/
/* @{ */
/**
* @brief This function makes an orthogonal normed right-hand coordinate system. If the z-axis is the input, then v1 is the x-axis and v2 the y-axis.
* @param v1 The input 3x1 vector.
* @param v2 The first orthogonal output 3x1 vector.
* @param v2 The second orthogonal output 3x1 vector.
*/

inline void makeCoordinateSystem(      Vector3 &v1,
                                       Vector3 &v2,
                                       Vector3 &v3)
{

   using std::abs;
   using std::sqrt;

   v1.normalize();

	if(abs(v1(0)) > abs(v1(2))){
    PREC invLen = 1.0 / sqrt(v1(0)*v1(0) + v1(2) * v1(2));
		v2 = typename MyMatrix::Vector3<PREC>(-v1(2) * invLen, 0, v1(0) *invLen);
	}
	else{
		PREC invLen = 1.0 / sqrt(v1(1)*v1(1) + v1(2) * v1(2));
		v2 = typename MyMatrix::Vector3<PREC>(0, v1(2) * invLen, -v1(1) *invLen);
	}
	v3 = v1.cross(v2);

  v2.normalize();
  v3.normalize();

};


inline void makeZAxisUp(Matrix33 & A_IK){
        // Make the z-Axis the axis which has the greatest z-component

        if(A_IK(0,2) > A_IK(1,2) && A_IK(0,2) > A_IK(2,2)){
           // x Axis becomes new z-Axis [x,y,z] -> [y,z,x]
           A_IK.col(0).swap(A_IK.col(2));
           A_IK.col(0).swap(A_IK.col(1));
        }else if(A_IK(1,2) > A_IK(0,2) && A_IK(1,2) > A_IK(2,2)){
            // y Axis becomes new z-Axis [x,y,z] -> [y,z,y]
           A_IK.col(0).swap(A_IK.col(2));
           A_IK.col(1).swap(A_IK.col(2));
        }
}


inline bool checkOrthogonality(Vector3 &v1,
                               Vector3 &v2,
                               Vector3 &v3,
                               PREC eps = 1e-6)
{

    if(v1.dot(v2) <= eps && v2.dot(v3) <= eps && v3.dot(v1) <= eps){
        return true;
    }
    return false;
}

/* @} */

};

#endif
