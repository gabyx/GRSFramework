#ifndef GRSF_Dynamics_Collision_Geometries_hpp
#define GRSF_Dynamics_Collision_Geometries_hpp


#include "GRSF/Dynamics/Collision/Geometry/Ray.hpp"
#include "GRSF/Dynamics/Collision/Geometry/AABB.hpp"
#include "GRSF/Dynamics/Collision/Geometry/SphereGeometry.hpp"
#include "GRSF/Dynamics/Collision/Geometry/BoxGeometry.hpp"
#include "GRSF/Dynamics/Collision/Geometry/HalfspaceGeometry.hpp"
#include "GRSF/Dynamics/Collision/Geometry/CapsuleGeometry.hpp"

#include "GRSF/Dynamics/Collision/Geometry/MeshGeometry.hpp"


#define DEFINE_GEOMETRY_PTR_TYPES( __RigidBodyType__ ) \
    using SphereGeomPtrType     = typename __RigidBodyType__::SphereGeomPtrType ; \
    using HalfspaceGeomPtrType  = typename __RigidBodyType__::HalfspaceGeomPtrType; \
    using BoxGeomPtrType        = typename __RigidBodyType__::BoxGeomPtrType; \
    using MeshPtrType           = typename __RigidBodyType__::MeshPtrType ;


#endif
