#ifndef GRSF_Dynamics_Collision_Geometry_HalfspaceGeometry_hpp
#define GRSF_Dynamics_Collision_Geometry_HalfspaceGeometry_hpp

#include <GRSF/Common/TypeDefs.hpp>
#include <boost/serialization/access.hpp>

#include "GRSF/Dynamics/Collision/Geometry/PlaneGeometry.hpp"

class HalfspaceGeometry : public PlaneGeometry {
public:

    DEFINE_MATRIX_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    HalfspaceGeometry(): PlaneGeometry() {};
    HalfspaceGeometry( const Vector3 & n, const Vector3 & p ): PlaneGeometry(n,p){};

private:
    friend class boost::serialization::access;

};


#endif
