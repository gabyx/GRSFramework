// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef TypeDefsPoints_hpp
#define TypeDefsPoints_hpp

#include "TypeDefs.hpp"

namespace TypeDefsPoints {

    DEFINE_MATRIX_TYPES

    using  Vector3List = StdVecAligned<Vector3>;
    using  Vector2List = StdVecAligned<Vector2>;

    using  Matrix3Dyn = MatrixStatDyn<3>;
    using  Matrix2Dyn = MatrixStatDyn<2>;

};

#define DEFINE_POINTS_CONFIG_TYPES \
    using Vector3List = typename TypeDefsPoints::Vector3List; \
    using Vector2List = typename TypeDefsPoints::Vector2List;\
    using Matrix3Dyn  = typename TypeDefsPoints::Matrix3Dyn;\
    using Matrix2Dyn  = typename TypeDefsPoints::Matrix2Dyn;

#endif
