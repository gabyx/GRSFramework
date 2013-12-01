#ifndef PlaneGeometry_hpp
#define PlaneGeometry_hpp

#include <TypeDefs.hpp>

#include <boost/serialization/access.hpp>

class PlaneGeometry {
public:
    DEFINE_LAYOUT_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PlaneGeometry() {
        m_normal.setZero();
        m_pos.setZero();
    }
    PlaneGeometry( const Vector3 & n, const Vector3 & p ):m_normal(n), m_pos(p) {}

    Vector3 m_normal; // in K frame
    Vector3 m_pos;    // in K frame

protected:

    friend class boost::serialization::access;


};


#endif
