// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef AABBnD_hpp
#define AABBnD_hpp

#include "GRSF/common/AssertionDebug.hpp"
#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/CPUTimer.hpp"
#include "GRSF/dynamics/collision/geometry/AABB.hpp"


template<unsigned int Dim>
class  AABBnD {
public:

    DEFINE_LAYOUT_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:

    template<unsigned int D = Dim,  bool = true>
    struct unite_impl{
        template<typename T, typename P>
        inline static void apply(T * t, const P & p){
            t->m_maxPoint(D-1) = std::max(t->m_maxPoint(D-1),p(D-1));
            t->m_minPoint(D-1) = std::min(t->m_minPoint(D-1),p(D-1));
            unite_impl<D-1>::apply(t,p);
        }
    };
    template<bool dummy>
    struct unite_impl<0,dummy>{
        template<typename T, typename P>
        inline static void apply(T * t, const P & p){}
    };

    template<unsigned int D = Dim,  bool = true>
    struct uniteBox_impl{
        template<typename T, typename B>
        inline static void apply(T * t, const B & b){
            t->m_maxPoint(D-1) = std::max(t->m_maxPoint(D-1),b.m_maxPoint(D-1));
            t->m_minPoint(D-1) = std::min(t->m_minPoint(D-1),b.m_minPoint(D-1));
            uniteBox_impl<D-1>::apply(t,b);
        }
    };
    template<bool dummy>
    struct uniteBox_impl<0,dummy>{
        template<typename T, typename B>
        inline static void apply(T * t, const B & b){}
    };


    template<unsigned int D = Dim, bool = true>
    struct reset_impl{
        template<typename T>
        inline static void apply(T * t){
            t->m_minPoint(D-1) = std::numeric_limits<PREC>::max();
            t->m_maxPoint(D-1) = std::numeric_limits<PREC>::lowest();
            reset_impl<D-1>::apply(t);
        }
    };

    template<bool dummy>
    struct reset_impl<0,dummy>{
        template<typename T>
        inline static void apply(T * t){}
    };

public:

    AABBnD() {
        reset();
    };
    void reset(){
        reset_impl<>::apply(this);
    }

    AABBnD( const VectorStat<Dim> &p): m_minPoint(p), m_maxPoint(p) {};

    AABBnD( const VectorStat<Dim> &l, const VectorStat<Dim> &u): m_minPoint(l), m_maxPoint(u) {
        GRSF_ASSERTMSG( (m_maxPoint >= m_minPoint).all(),
        "AABB initialized wrongly! min/max: " << m_minPoint.tranpose() <<"/" << m_maxPoint.transpose();)
    };


    template<typename Derived>
    AABBnD& unite(const MatrixBase<Derived> &p) {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,Dim);
      unite_impl<>::apply(this,p);
      return *this;
    };

    template<typename Derived>
    AABBnD& operator+=(const MatrixBase<Derived> &p){
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,Dim);
        unite_impl<>::apply(this,p);
        return *this;
    }


    void unite(const AABBnD & box) {
        uniteBox_impl<>::apply(this,box);
    };

    AABBnD& operator+=(const AABBnD & box){
        uniteBox_impl<>::apply(this,box);
        return *this;
    }

    AABBnD operator+ (const AABBnD & box){
        AABBnD r = this;
        uniteBox_impl<>::apply(&r,box);
        return r;
    }

    AABBnD& transform(const AffineTrafo & M) {
        GRSF_STATIC_ASSERTM(Dim==3,"So far AABB transform is only implemented in 3d")
        AABBnD ret( M*(Vector3( m_minPoint(0), m_minPoint(1), m_minPoint(2))));
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


    inline VectorStat<Dim> center(){ return 0.5*(m_maxPoint + m_minPoint);}

    inline bool overlaps(const AABBnD & box) const {
        return ((m_maxPoint.array() >= box.m_minPoint.array()) && (m_minPoint.array() <= box.m_maxPoint.array())).all();
    };

    template<typename Derived>
    inline bool overlaps(const MatrixBase<Derived> &p) const {
        return ((p.array() >= m_minPoint.array()) && (p.array() <= m_maxPoint.array())).all();
    };

    inline ArrayStat<Dim> extent() const{
        // since min <= max, extent can not be smaller than zero
        // , except if AABBnD contains no points/uninitialized (reset())
        return (m_maxPoint - m_minPoint).array();
    };

    inline PREC maxExtent() const{
        return (m_maxPoint - m_minPoint).maxCoeff();
    };

    inline bool isEmpty() const {
        return (m_maxPoint.array() <= m_minPoint.array()).any();
    }

    inline void expand(PREC d) {
        GRSF_ASSERTMSG(d>=0,"d>=0")
        m_minPoint.array() -= d;
        m_maxPoint.array() += d;
    };

    inline void expand(VectorStat<Dim> d) {
        GRSF_ASSERTMSG((d.array()>=0).all(), "d<0")
        m_minPoint -= d;
        m_maxPoint += d;
    };

    /** Adjust box that all axes have at least a minimal extent of maxExtent*p, if maxExtent*p < eps then all axes to default extent */
    void expandToMinExtentRelative(PREC p, PREC defaultExtent, PREC eps){
        ArrayStat<Dim> e = extent();
        VectorStat<Dim> c = center();
        typename ArrayStat<Dim>::Index idx;
        PREC ext = std::abs(e.maxCoeff(&idx)) * p;

        if( ext < eps ){ // extent of max axis almost zero, set all axis to defaultExtent --> cube
            ext = defaultExtent;
            m_minPoint = c - 0.5*ext;
            m_maxPoint = c + 0.5*ext;
        }else{
            for(int i=0;i<Dim;++i){
                if(i!=idx && std::abs(e(i)) < ext){
                    m_minPoint(i) = c(i) - 0.5*ext;
                    m_maxPoint(i) = c(i) + 0.5*ext;
                }
            }
        }
    }

    /** Adjust box that all axes have at least a minimal extent  minExtent*/
    void expandToMinExtentAbsolute(PREC minExtent){
        Array3 e = extent();
        Vector3 c = center();

        PREC l = 0.5*minExtent;
        for(int i=0;i<Dim;++i){
            if(std::abs(e(i)) < minExtent){
                m_minPoint(i) = c(i) - l;
                m_maxPoint(i) = c(i) + l;
            }
        }
    }

    /** Adjust box that all axes have at least a minimal extent  minExtent for each axis*/
    void expandToMinExtentAbsolute(ArrayStat<3> minExtent){
        Array3 e = extent();
        Vector3 c = center();

        for(int i=0;i<Dim;++i){
            PREC l = minExtent(i);
            if(std::abs(e(i)) < l){
                m_minPoint(i) = c(i) - 0.5*l;
                m_maxPoint(i) = c(i) + 0.5*l;
            }
        }
    }

    inline PREC volume() const {
        return (m_maxPoint - m_minPoint).prod();
    };

    //info about axis aligned bounding box
    VectorStat<Dim> m_minPoint;
    VectorStat<Dim> m_maxPoint;
};



DEFINE_LAYOUT_CONFIG_TYPES;


void AABBnDTest(){



    const unsigned int N = 1000;
    MatrixStatDyn<3> v(3,N);
    v.setRandom();
    AABB aabb;
    AABB aabb1(Vector3(1,1,1));
    AABBnD<3> aabb1nD(Vector3(1,1,1));
    AABBnD<3> aabb2;
    bool t;
    {
        START_TIMER(start)
        for(int j = 0; j < 100000; j++){
            aabb.reset();
            for(unsigned int i=0; i<N;++i){
                aabb.unite(v.col(i));
                t = aabb.isEmpty(); t &= aabb.overlaps(aabb1);
            }
        }
        STOP_TIMER_MILLI(count,start)
        std::cout << "Test AABB: " << count << std::endl;
        std::cout << "res: " << aabb.m_minPoint << std::endl;
    }

    {
        START_TIMER(start)
        for(int j = 0; j < 100000; j++){
            aabb.reset();
            for(unsigned int i=0; i<N;++i){
                aabb2.unite(v.col(i));
                t = aabb2.isEmpty(); t &= aabb2.overlaps(aabb1nD);
            }
        }
        STOP_TIMER_MILLI(count,start)
        aabb2.isEmpty();
        std::cout << "Test AABBnD<3>: " << count << std::endl;
        std::cout << "res: " << aabb2.m_minPoint << std::endl;
    }
}

#endif // AABBnD_hpp

