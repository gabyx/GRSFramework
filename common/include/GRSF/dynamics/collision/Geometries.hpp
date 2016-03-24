// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_collision_Geometries_hpp
#define GRSF_dynamics_collision_Geometries_hpp


#include "GRSF/dynamics/collision/geometry/Ray.hpp"
#include "GRSF/dynamics/collision/geometry/AABB.hpp"
#include "GRSF/dynamics/collision/geometry/SphereGeometry.hpp"
#include "GRSF/dynamics/collision/geometry/BoxGeometry.hpp"
#include "GRSF/dynamics/collision/geometry/HalfspaceGeometry.hpp"
#include "GRSF/dynamics/collision/geometry/CapsuleGeometry.hpp"

#include "GRSF/dynamics/collision/geometry/MeshGeometry.hpp"


#define DEFINE_GEOMETRY_PTR_TYPES( __RigidBodyType__ ) \
    using SphereGeomPtrType     = typename __RigidBodyType__::SphereGeomPtrType ; \
    using HalfspaceGeomPtrType  = typename __RigidBodyType__::HalfspaceGeomPtrType; \
    using BoxGeomPtrType        = typename __RigidBodyType__::BoxGeomPtrType; \
    using MeshPtrType           = typename __RigidBodyType__::MeshPtrType ;


#endif
