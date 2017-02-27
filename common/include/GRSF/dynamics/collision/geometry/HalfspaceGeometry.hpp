// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_collision_geometry_HalfspaceGeometry_hpp
#define GRSF_dynamics_collision_geometry_HalfspaceGeometry_hpp

#include <boost/serialization/access.hpp>
#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/dynamics/collision/geometry/PlaneGeometry.hpp"

class HalfspaceGeometry : public PlaneGeometry
{
public:
    DEFINE_MATRIX_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    HalfspaceGeometry() : PlaneGeometry(){};
    HalfspaceGeometry(const Vector3& n /*,const Vector3 & p*/) : PlaneGeometry(n /*,p*/){};

private:
    friend class boost::serialization::access;
};

#endif
