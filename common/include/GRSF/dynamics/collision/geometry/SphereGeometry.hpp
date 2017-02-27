// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_collision_geometry_SphereGeometry_hpp
#define GRSF_dynamics_collision_geometry_SphereGeometry_hpp

#include <boost/serialization/access.hpp>

class SphereGeometry
{
public:
    DEFINE_MATRIX_TYPES

    SphereGeometry(PREC r) : m_radius(r){};
    SphereGeometry() : m_radius(0){};

    PREC m_radius;

private:
    friend class boost::serialization::access;
};

#endif
