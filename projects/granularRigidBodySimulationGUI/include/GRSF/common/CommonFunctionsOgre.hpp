// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_CommonFunctionsOgre_hpp
#define GRSF_common_CommonFunctionsOgre_hpp

#include <string>
#include <vector>

#include <OGRE/Ogre.h>

#include "GRSF/dynamics/general/MyMatrixTypeDefs.hpp"
#include "GRSF/common/Asserts.hpp"


namespace OgreUtilities {


Ogre::StringVector convertToOgreStringVector(const std::vector<std::string> & vec);


/**
* @brief Converts a vector 3x1 from Ogre to an Eigen 3x1 Vector.
*/
template<typename PREC>
typename MyMatrix::Vector3<PREC> vectorFromOgre(const Ogre::Vector3& v) {
    Eigen::Matrix<double,3,1> vec;
    vec(0) = v.x;
    vec(1) = v.y;
    vec(2) = v.z;
    return vec;
}


/**
* @brief Converts a quaternion 4x1 from Ogre to an Eigen 4x1 Vector.
*/
template<typename PREC>
typename MyMatrix::Vector4<PREC> vectorFromOgre(const Ogre::Quaternion & v) {
    typename MyMatrix::Vector4<PREC> vec;
    vec(0) = v.w;
    vec(1) = v.x;
    vec(2) = v.y;
    vec(3) = v.z;
    return vec;
}

};
#endif
