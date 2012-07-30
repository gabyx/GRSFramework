#ifndef AABB_hpp
#define AABB_hpp

#include <algorithm>
#include "TypeDefs.hpp"



template<typename TLayoutConfig>
class AABB {
public:

    DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    AABB() {
        // Violating the constraint min<max for making a completey empty box!
        m_minPoint.x = std::numeric_limits<double>::max();
        m_maxPoint.x = std::numeric_limits<double>::min();
        m_minPoint.y = std::numeric_limits<double>::max();
        m_maxPoint.y = std::numeric_limits<double>::min();
        m_minPoint.z = std::numeric_limits<double>::max();
        m_maxPoint.z = std::numeric_limits<double>::min();

    };
    AABB( const Vector3 &p) {
        m_minPoint = p;
        m_maxPoint = p;
    };
    AABB( const Vector3 &l, const Vector3 &u) {
        m_minPoint = Vector3(std::min(l.x,u.x),std::min(l.y,u.y),std::min(l.z,u.z));
        m_maxPoint = Vector3(std::max(l.x,u.x),std::max(l.y,u.y),std::max(l.z,u.z));
    };

    AABB& unite(const Vector3 &p) {
        m_maxPoint.x = std::max(m_maxPoint.x,p.x);
        m_maxPoint.y = std::max(m_maxPoint.y,p.y);
        m_maxPoint.z = std::max(m_maxPoint.z,p.z);
        m_minPoint.x = std::min( m_minPoint.x,p.x);
        m_minPoint.y = std::min( m_minPoint.y,p.y);
        m_minPoint.z = std::min( m_minPoint.z,p.z);
        return *this;
    };

    AABB& unite(const AABB & box) {
        m_maxPoint.x = std::max(m_maxPoint.x,box.m_maxPoint.x);
        m_maxPoint.y = std::max(m_maxPoint.y,box.m_maxPoint.y);
        m_maxPoint.z = std::max(m_maxPoint.z,box.m_maxPoint.z);
        m_minPoint.x = std::min( m_minPoint.x,box. m_minPoint.x);
        m_minPoint.y = std::min( m_minPoint.y,box. m_minPoint.y);
        m_minPoint.z = std::min( m_minPoint.z,box. m_minPoint.z);
        return *this;
    };

    AABB operator+ (const Vector3 &p) {
        AABB a;
        a.m_maxPoint.x = std::max(m_maxPoint.x,p.x);
        a.m_maxPoint.y = std::max(m_maxPoint.y,p.y);
        a.m_maxPoint.z = std::max(m_maxPoint.z,p.z);
        a. m_minPoint.x = std::min( m_minPoint.x,p.x);
        a. m_minPoint.y = std::min( m_minPoint.y,p.y);
        a. m_minPoint.z = std::min( m_minPoint.z,p.z);
        return a;
    };

    AABB operator+ (const AABB & box) {
        AABB a;
        a.m_maxPoint.x = std::max(m_maxPoint.x,box.m_maxPoint.x);
        a.m_maxPoint.y = std::max(m_maxPoint.y,box.m_maxPoint.y);
        a.m_maxPoint.z = std::max(m_maxPoint.z,box.m_maxPoint.z);
        a. m_minPoint.x = std::min( m_minPoint.x,box. m_minPoint.x);
        a. m_minPoint.y = std::min( m_minPoint.y,box. m_minPoint.y);
        a. m_minPoint.z = std::min( m_minPoint.z,box. m_minPoint.z);
        return a;
    };

    AABB& operator+= (const AABB & box) {
        m_maxPoint.x = std::max(m_maxPoint.x,box.m_maxPoint.x);
        m_maxPoint.y = std::max(m_maxPoint.y,box.m_maxPoint.y);
        m_maxPoint.z = std::max(m_maxPoint.z,box.m_maxPoint.z);
        m_minPoint.x = std::min( m_minPoint.x,box. m_minPoint.x);
        m_minPoint.y = std::min( m_minPoint.y,box. m_minPoint.y);
        m_minPoint.z = std::min( m_minPoint.z,box. m_minPoint.z);
        return *this;
    };

    AABB & transform(const Matrix44 & M) {

        AABB ret(M * (Vector3( m_minPoint.x, m_minPoint.y, m_minPoint.z)));
        ret.unite(M* (Vector3(m_maxPoint.x, m_minPoint.y, m_minPoint.z)));
        ret.unite(M*(Vector3( m_minPoint.x, m_maxPoint.y, m_minPoint.z)));
        ret.unite(M*(Vector3( m_minPoint.x, m_minPoint.y, m_maxPoint.z)));
        ret.unite(M*(Vector3( m_minPoint.x, m_maxPoint.y, m_maxPoint.z)));
        ret.unite(M*(Vector3(m_maxPoint.x, m_maxPoint.y, m_minPoint.z)));
        ret.unite(M*(Vector3(m_maxPoint.x, m_minPoint.y, m_maxPoint.z)));
        ret.unite(M*(Vector3(m_maxPoint.x, m_maxPoint.y, m_maxPoint.z)));
        *this = ret;
        return *this;
    };

    bool overlaps(const AABB & box) const {
        bool x = (m_maxPoint.x >= box. m_minPoint.x) && ( m_minPoint.x <= box.m_maxPoint.x);
        bool y = (m_maxPoint.y >= box. m_minPoint.y) && ( m_minPoint.y <= box.m_maxPoint.y);
        bool z = (m_maxPoint.z >= box. m_minPoint.z) && ( m_minPoint.z <= box.m_maxPoint.z);
        return (x && y && z);
    };

    bool inside(const Vector3 &p) const {
        return (
                   p.x >= m_minPoint.x && p.x <= m_maxPoint.x &&
                   p.y >= m_minPoint.y && p.y <= m_maxPoint.y &&
                   p.z >= m_minPoint.z && p.z <= m_maxPoint.z);
    };


    void expand(PREC d) {
        m_minPoint -= Vector3(d,d,d);
        m_maxPoint += Vector3(d,d,d);
    };

    PREC volume() const {
        Vector3 d = m_maxPoint- m_minPoint;
        return d.x * d.y * d.z;
    };

    PREC maxExtend() const {
        Vector3 d = m_maxPoint- m_minPoint;
        if (d.x > d.y && d.x > d.z) {
            return 0;
        } else if (d.y > d.z) {
            return 1;
        } else {
            return 2;
        }
    };

    //info about axis aligned bounding box
    Vector3 m_minPoint;
    Vector3 m_maxPoint;
};

 #endif
