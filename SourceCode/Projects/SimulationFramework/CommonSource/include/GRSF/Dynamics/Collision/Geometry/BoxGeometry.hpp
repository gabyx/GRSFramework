#ifndef GRSF_Dynamics_Collision_Geometry_BoxGeometry_hpp
#define GRSF_Dynamics_Collision_Geometry_BoxGeometry_hpp

#include "GRSF/Common/TypeDefs.hpp"
#include <boost/serialization/access.hpp>

class BoxGeometry {
public:

    DEFINE_MATRIX_TYPES

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BoxGeometry() {
        m_extent.setZero();
        m_center.setZero();
    };
    BoxGeometry(Vector3 center, Vector3 extent): m_extent(extent), m_center(center) {};

    Vector3 m_extent; ///< Vector of the extend of the box in all directions.
    Vector3 m_center; ///< Vector to the center of the box in body frame! (mostly zero).

    /** Returns K_r_CenterP */
    Vector3 getPoint(unsigned int i) const{
        Vector3 point;
        point(0) = 0.5 * BoxGeometry::m_pointIdx[3*i]*m_extent(0);
        point(1) = 0.5 * BoxGeometry::m_pointIdx[3*i+1]*m_extent(1);
        point(2) = 0.5 * BoxGeometry::m_pointIdx[3*i+2]*m_extent(2);
        return point;
    }

private:
    friend class boost::serialization::access;
    static char m_pointIdx[8*3];
};

#endif