#ifndef GRSF_Dynamics_General_CartesianGrid_hpp
#define GRSF_Dynamics_General_CartesianGrid_hpp

#include "GRSF/Common/AssertionDebug.hpp"
#include "GRSF/Common/TypeDefs.hpp"

#include <set>
#include "GRSF/Dynamics/Collision/Geometry/AABB.hpp"



struct NoCellData {};

template<typename TCellData = NoCellData>
class CartesianGrid {
    DEFINE_MATRIX_TYPES
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CartesianGrid();

    CartesianGrid(const AABB & aabb,
                  const MyMatrix<unsigned int>::Array3 & dim);
    ~CartesianGrid();



    /** Get cell index */
    template<typename Derived>
    MyMatrix<unsigned int>::Array3 getCellIndex(const MatrixBase<Derived> & I_point) const;
    template<typename Derived>
    MyMatrix<unsigned int>::Array3 getCellIndexClosest(const MatrixBase<Derived> & I_point) const;

    /** Get cell data  */
    template<typename Derived>
    TCellData * getCellData(const MatrixBase<Derived> & I_point) const;
    template<typename Derived>
    TCellData * getCellData(const ArrayBase<Derived> & index) const;

protected:


    Array3 m_dxyzInv;
    Array3 m_dxyz;
    MyMatrix<unsigned int>::Array3 m_dim;

    AABB m_Box; ///< AABB in frame I

    std::vector<TCellData> m_cellData;

    static char m_nbIndicesOff[26*3];

private:

};



template<typename TCellData>
CartesianGrid<TCellData>::CartesianGrid() {
    m_dxyz.setZero();
    m_dim.setZero();
};

template<typename TCellData>
CartesianGrid<TCellData>::~CartesianGrid() {
};

template<typename TCellData>
CartesianGrid<TCellData>::CartesianGrid(const AABB & aabb,
                                        const MyMatrix<unsigned int>::Array3 & dim) {
    ASSERTMSG(dim(0)*dim(1)*dim(2) != 0, "Dimension zero: " << dim)
    ASSERTMSG( aabb.isEmpty() == false, "CartesianGrid, wrongly initialized: maxPoint < minPoint");
    m_Box = aabb;
    m_dim = dim;

    m_dxyz = m_Box.extent() / dim.cast<PREC>();
    m_dxyzInv = m_dxyz.inverse();

    if(! std::is_same<TCellData,NoCellData>::value){
        m_cellData.resize(m_dim(0)*m_dim(1)*m_dim(2));
    }

};

template<typename TCellData>
template<typename Derived>
MyMatrix<unsigned int>::Array3 CartesianGrid<TCellData>::getCellIndex(const MatrixBase<Derived> & I_point) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,3);

    ASSERTMSG(m_Box.inside(I_point),"Point: " << I_point << " is not inside the Grid!");

    // calculate index normally and then project it into the feasible grid.
    MyMatrix<unsigned int>::Array3 v;
    v = (((I_point - m_Box.m_minPoint).array()) * m_dxyzInv).template cast<unsigned int>();

    return v;
};

template<typename TCellData>
template<typename Derived>
MyMatrix<unsigned int>::Array3 CartesianGrid<TCellData>::getCellIndexClosest(const MatrixBase<Derived> & I_point) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,3);

    typedef long long int LongInt;

    // calculate index normally and then project it into the feasible grid.
    MyMatrix<LongInt>::Array3 v;
    v =  (((I_point - m_Box.m_minPoint).array()) * m_dxyzInv).template cast<LongInt>();

    // prox  index to feasible range (cartesian prox)
    v(0) = std::max(   std::min( LongInt(m_dim(0)-1), v(0)),  0LL   );
    v(1) = std::max(   std::min( LongInt(m_dim(1)-1), v(1)),  0LL   );
    v(2) = std::max(   std::min( LongInt(m_dim(2)-1), v(2)),  0LL   );

    return v.cast<unsigned int>();
};

template<typename TCellData>
template<typename Derived>
TCellData * CartesianGrid<TCellData>::getCellData(const MatrixBase<Derived> & I_point) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,3);

    if(std::is_same<TCellData,NoCellData>::value){
            return nullptr;
    }else{
            auto i = getCellIndex(I_point);
            return &m_cellData[i(0) + m_dim(1)*i(1) + m_dim(1)*m_dim(2)*i(2) ];
    }
};

template<typename TCellData>
template<typename Derived>
TCellData* CartesianGrid<TCellData>::getCellData(const ArrayBase<Derived> & index) const {
    STATIC_ASSERT( std::is_integral<typename Derived::Scalar>::value );

    if(std::is_same<TCellData,NoCellData>::value){
        return nullptr;
    }else{
        ASSERTMSG( ( (index(0) >=0 && index(0) < m_dim(0)) &&
                (index(1) >=0 && index(1) < m_dim(1)) &&
                (index(2) >=0 && index(2) < m_dim(2)) ),
              "Index: " << index << " is out of bound" )
        return &m_cellData[index(0) + m_dim(1)*index(1) + m_dim(1)*m_dim(2)*index(2) ];
    }
}



template<typename TCellData>
char CartesianGrid<TCellData>::m_nbIndicesOff[26*3] = {
    1,0,0,
    1,0,1,
    1,0,-1,

    -1,0,0,
    -1,0,1,
    -1,0,-1,

    0,0,1,
    0,0,-1,

    0,1,0,
    0,1,1,
    0,1,-1,

    0,-1,0,
    0,-1,1,
    0,-1,-1,

    1,1,0,
    1,1,1,
    1,1,-1,

    -1,1,0,
    -1,1,1,
    -1,1,-1,

    1,-1,0,
    1,-1,1,
    1,-1,-1,

    -1,-1,0,
    -1,-1,1,
    -1,-1,-1,
};



#endif
