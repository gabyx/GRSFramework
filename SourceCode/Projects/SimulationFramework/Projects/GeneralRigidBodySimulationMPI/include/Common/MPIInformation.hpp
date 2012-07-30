
#ifndef MPIInformation_hpp
#define MPIInformation_hpp

#include <mpi.h>

#include "AssertionDebug.hpp"
#include "TypeDefs.hpp"

#include "AABB.hpp"

namespace MPILayer{


class ProcessInformation{
    std::string m_name;
    unsigned int m_rank;

    static const int MASTER = 0;
};

template<TLayoutConfig>
class CartesianGrid : private class AABB<TLayoutConfig> {

    DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
        CartesianGrid(): AABB(){
            m_dxyz.setZero();
            m_center.setZero();
            m_dim.setZero();
        };

        CartesianGrid(const Vector3 & center,
                      const Vector3 & dxyz,
                      const MyMatrix<unsigned int>::Vector3 & dim ){
            m_center = center;
            m_dim = dim;
            MyMatrix<unsigned int>::Vector3 d = dim/2;
            this->m_minPoint = center - d.asDiagonal()*dxyz;
            this->m_maxPoint = center + ((dim - d).asDiagonal()*dxyz);
        };

        const MyMatrix<unsigned int>::Vector3 getGridId(const Vector3 & point) const{
            ASSERTMSG(this->inside(point),"Point is not inside the Grid!");
            MyMatrix<unsigned int>::Vector3 v;
            v.array() =  ((point - this->m_minPoint).array()) / this->extent().array()
            return v;
        };



private:
    Vector3 m_dxyz;
    Vector3 m_center;
    MyMatrix<unsigned int>::Vector3 m_dim;
};


};

#endif
