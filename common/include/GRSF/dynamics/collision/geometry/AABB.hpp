// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_collision_geometry_AABB_hpp
#define GRSF_dynamics_collision_geometry_AABB_hpp

#include <algorithm>

#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/AssertionDebug.hpp"

#include "ApproxMVBB/AABB.hpp"

template<unsigned int D>
using AABB = ApproxMVBB::AABB<D>;

using AABB3d = ApproxMVBB::AABB3d;
using AABB2d = ApproxMVBB::AABB2d;


 #endif
