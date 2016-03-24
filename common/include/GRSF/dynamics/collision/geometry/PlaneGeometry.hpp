#ifndef GRSF_dynamics_collision_geometry_PlaneGeometry_hpp
#define GRSF_dynamics_collision_geometry_PlaneGeometry_hpp

#include "GRSF/common/TypeDefs.hpp"

#include <boost/serialization/access.hpp>

class PlaneGeometry {
public:
    DEFINE_MATRIX_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PlaneGeometry() {
        m_normal.setZero();
        /*m_pos.setZero();*/
    }
    PlaneGeometry( const Vector3 & n/*, const Vector3 & p*/ ):m_normal(n)/*, m_pos(p)*/ {}

    Vector3 m_normal; // in K frame
    /*Vector3 m_pos;*/    // in K frame (not needed)

protected:

    friend class boost::serialization::access;


};


#endif
