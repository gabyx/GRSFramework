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
