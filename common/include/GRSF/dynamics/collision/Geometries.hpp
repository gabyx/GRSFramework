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
