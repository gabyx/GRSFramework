#ifndef AABB_hpp
#define AABB_hpp

#include <algorithm>

#include "TypeDefs.hpp"



class AABB {
public:

    DEFINE_MATRIX_TYPES

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    AABB() {
        reset();
    };

    void reset(){
        // Violating the constraint min<max for making a completey empty box!
        m_minPoint(0) = std::numeric_limits<double>::max();
        m_maxPoint(0) = std::numeric_limits<double>::min();
        m_minPoint(1) = std::numeric_limits<double>::max();
        m_maxPoint(1) = std::numeric_limits<double>::min();
        m_minPoint(2) = std::numeric_limits<double>::max();
        m_maxPoint(2) = std::numeric_limits<double>::min();
    }

    AABB( const Vector3 &p) {
        m_minPoint = p;
        m_maxPoint = p;
    };
    AABB( const Vector3 &l, const Vector3 &u) {
        m_minPoint = Vector3(std::min(l(0),u(0)),std::min(l(1),u(1)),std::min(l(2),u(2)));
        m_maxPoint = Vector3(std::max(l(0),u(0)),std::max(l(1),u(1)),std::max(l(2),u(2)));
    };

    AABB& unite(const Vector3 &p) {
        m_maxPoint(0) = std::max(m_maxPoint(0),p(0));
        m_maxPoint(1) = std::max(m_maxPoint(1),p(1));
        m_maxPoint(2) = std::max(m_maxPoint(2),p(2));
        m_minPoint(0) = std::min( m_minPoint(0),p(0));
        m_minPoint(1) = std::min( m_minPoint(1),p(1));
        m_minPoint(2) = std::min( m_minPoint(2),p(2));
        return *this;
    };

    AABB& unite(const AABB & box) {
        m_maxPoint(0) = std::max(m_maxPoint(0),box.m_maxPoint(0));
        m_maxPoint(1) = std::max(m_maxPoint(1),box.m_maxPoint(1));
        m_maxPoint(2) = std::max(m_maxPoint(2),box.m_maxPoint(2));
        m_minPoint(0) = std::min( m_minPoint(0),box.m_minPoint(0));
        m_minPoint(1) = std::min( m_minPoint(1),box.m_minPoint(1));
        m_minPoint(2) = std::min( m_minPoint(2),box.m_minPoint(2));
        return *this;
    };

    AABB operator+ (const Vector3 &p) {
        AABB a;
        a.m_maxPoint(0) = std::max(m_maxPoint(0),p(0));
        a.m_maxPoint(1) = std::max(m_maxPoint(1),p(1));
        a.m_maxPoint(2) = std::max(m_maxPoint(2),p(2));
        a. m_minPoint(0) = std::min( m_minPoint(0),p(0));
        a. m_minPoint(1) = std::min( m_minPoint(1),p(1));
        a. m_minPoint(2) = std::min( m_minPoint(2),p(2));
        return a;
    };

    AABB operator+ (const AABB & box) {
        AABB a;
        a.m_maxPoint(0) = std::max(m_maxPoint(0),box.m_maxPoint(0));
        a.m_maxPoint(1) = std::max(m_maxPoint(1),box.m_maxPoint(1));
        a.m_maxPoint(2) = std::max(m_maxPoint(2),box.m_maxPoint(2));
        a. m_minPoint(0) = std::min( m_minPoint(0),box. m_minPoint(0));
        a. m_minPoint(1) = std::min( m_minPoint(1),box. m_minPoint(1));
        a. m_minPoint(2) = std::min( m_minPoint(2),box. m_minPoint(2));
        return a;
    };

    AABB& operator+= (const AABB & box) {
        m_maxPoint(0) = std::max(m_maxPoint(0),box.m_maxPoint(0));
        m_maxPoint(1) = std::max(m_maxPoint(1),box.m_maxPoint(1));
        m_maxPoint(2) = std::max(m_maxPoint(2),box.m_maxPoint(2));
        m_minPoint(0) = std::min( m_minPoint(0),box. m_minPoint(0));
        m_minPoint(1) = std::min( m_minPoint(1),box. m_minPoint(1));
        m_minPoint(2) = std::min( m_minPoint(2),box. m_minPoint(2));
        return *this;
    };

    Vector3 extent() const{
        return m_maxPoint - m_minPoint;
    };

    PREC maxExtent() const{
        return (m_maxPoint - m_minPoint).maxCoeff();
    };

    AABB & transform(const AffineTrafo & M) {

        AABB ret( M*(Vector3( m_minPoint(0), m_minPoint(1), m_minPoint(2))));
        ret.unite(M*(Vector3( m_maxPoint(0), m_minPoint(1), m_minPoint(2))));
        ret.unite(M*(Vector3( m_minPoint(0), m_maxPoint(1), m_minPoint(2))));
        ret.unite(M*(Vector3( m_minPoint(0), m_minPoint(1), m_maxPoint(2))));
        ret.unite(M*(Vector3( m_minPoint(0), m_maxPoint(1), m_maxPoint(2))));
        ret.unite(M*(Vector3( m_maxPoint(0), m_maxPoint(1), m_minPoint(2))));
        ret.unite(M*(Vector3( m_maxPoint(0), m_minPoint(1), m_maxPoint(2))));
        ret.unite(M*(Vector3( m_maxPoint(0), m_maxPoint(1), m_maxPoint(2))));
        *this = ret;
        return *this;
    };

    bool overlaps(const AABB & box) const {
        bool x = (m_maxPoint(0) >= box. m_minPoint(0)) && ( m_minPoint(0) <= box.m_maxPoint(0));
        bool y = (m_maxPoint(1) >= box. m_minPoint(1)) && ( m_minPoint(1) <= box.m_maxPoint(1));
        bool z = (m_maxPoint(2) >= box. m_minPoint(2)) && ( m_minPoint(2) <= box.m_maxPoint(2));
        return (x && y && z);
    };

    bool inside(const Vector3 &p) const {
        return (
                   p(0) >= m_minPoint(0) && p(0) <= m_maxPoint(0) &&
                   p(1) >= m_minPoint(1) && p(1) <= m_maxPoint(1) &&
                   p(2) >= m_minPoint(2) && p(2) <= m_maxPoint(2));
    };


    void expand(PREC d) {
        ASSERTMSG(d>=0,"d>=0")
        m_minPoint -= Vector3(d,d,d);
        m_maxPoint += Vector3(d,d,d);
    };

    void expand(Vector3 d) {
        ASSERTMSG(d(0)>=0 && d(1)>=0 && d(2)>=0,"d>=0")
        m_minPoint -= d;
        m_maxPoint += d;
    };

    PREC volume() const {
        Vector3 d = m_maxPoint- m_minPoint;
        return d(0) * d(1) * d(2);
    };

    //info about axis aligned bounding box
    Vector3 m_minPoint;
    Vector3 m_maxPoint;
};

 #endif
