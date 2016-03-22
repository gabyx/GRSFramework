#ifndef GRSF_Dynamics_Collision_Geometry_AABB_hpp
#define GRSF_Dynamics_Collision_Geometry_AABB_hpp

#include <algorithm>

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/AssertionDebug.hpp"

#include "ApproxMVBB/AABB.hpp"

template<unsigned int D>
using AABB = ApproxMVBB::AABB<D>;

using AABB3d = ApproxMVBB::AABB3d;
using AABB2d = ApproxMVBB::AABB2d;


 #endif
