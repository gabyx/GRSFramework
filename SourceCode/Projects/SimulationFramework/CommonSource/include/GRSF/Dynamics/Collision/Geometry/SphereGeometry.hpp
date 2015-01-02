#ifndef GRSF_Dynamics_Collision_Geometry_SphereGeometry_hpp
#define GRSF_Dynamics_Collision_Geometry_SphereGeometry_hpp

#include <boost/serialization/access.hpp>

class SphereGeometry {
public:
    DEFINE_MATRIX_TYPES

    SphereGeometry(PREC r) : m_radius(r){};
    SphereGeometry(): m_radius(0){};

    PREC m_radius;

private:

    friend class boost::serialization::access;

};


#endif
