#ifndef GRSF_Dynamics_General_CartesianGrid_hpp
#define GRSF_Dynamics_General_CartesianGrid_hpp

#include "GRSF/Common/AssertionDebug.hpp"
#include "GRSF/Common/TypeDefs.hpp"

#include <set>
#include "GRSF/Dynamics/Collision/Geometry/AABB.hpp"



struct NoCellData {};

template<typename TCellData = NoCellData, typename TSize = unsigned int>
class CartesianGrid {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DEFINE_MATRIX_TYPES
    using Array3Int = typename MyMatrix<TSize>::Array3;

    CartesianGrid(){
        m_dxyz.setZero();
        m_dim.setZero();
    }

    CartesianGrid(const AABB3d & aabb,
                  const Array3Int & dim){
        ASSERTMSG(dim(0)*dim(1)*dim(2) != 0, "Dimension zero: " << dim)
        ASSERTMSG( aabb.isEmpty() == false, "CartesianGrid, wrongly initialized: maxPoint < minPoint");
        m_Box = aabb;
        m_dim = dim;

        m_dxyz = m_Box.extent() / dim.template cast<PREC>();
        m_dxyzInv = m_dxyz.inverse();

        if(! std::is_same<TCellData,NoCellData>::value){
            m_cellData.resize(m_dim(0)*m_dim(1)*m_dim(2));
        }

    }

    ~CartesianGrid(){}



    /** Get cell index */
    template<typename Derived>
    Array3Int getCellIndex(const MatrixBase<Derived> & I_point) const
    {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,3);

        ASSERTMSG(m_Box.overlaps(I_point),"Point: " << I_point << " is not inside the Grid!");

        // calculate index normally and then project it into the feasible grid.
        Array3Int v;
        v = (((I_point - m_Box.m_minPoint).array()) * m_dxyzInv).template cast<TSize>();

        return v;
    }

    template<typename Derived>
    Array3Int getCellIndexClosest(const MatrixBase<Derived> & I_point) const{
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,3);

        typedef long long int LongInt;

        // calculate index normally and then project it into the feasible grid.
        MyMatrix<LongInt>::Array3 v;
        v =  (((I_point - m_Box.m_minPoint).array()) * m_dxyzInv).template cast<LongInt>();

        // prox  index to feasible range (cartesian prox)
        v(0) = std::max(   std::min( LongInt(m_dim(0)-1), v(0)),  0LL   );
        v(1) = std::max(   std::min( LongInt(m_dim(1)-1), v(1)),  0LL   );
        v(2) = std::max(   std::min( LongInt(m_dim(2)-1), v(2)),  0LL   );

        return v.cast<TSize>();
    };

    /** Get cell data  */
    template<typename Derived>
    const TCellData * getCellData(const MatrixBase<Derived> & I_point) const {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,3);

        if(std::is_same<TCellData,NoCellData>::value){
                return nullptr;
        }else{
                auto i = getCellIndex(I_point);
                return &m_cellData[i(0) + m_dim(1)*i(1) + m_dim(1)*m_dim(2)*i(2) ];
        }
    }

    template<typename Derived>
    const TCellData * getCellData(const ArrayBase<Derived> & index) const
    {
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

protected:


    Array3 m_dxyzInv;
    Array3 m_dxyz;
    Array3Int m_dim;

    AABB3d m_Box;

    std::vector<TCellData> m_cellData;

    static char m_nbIndicesOff[26*3];

private:

};


template<typename TCellData, typename TSize>
char CartesianGrid<TCellData,TSize>::m_nbIndicesOff[26*3] = {
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
