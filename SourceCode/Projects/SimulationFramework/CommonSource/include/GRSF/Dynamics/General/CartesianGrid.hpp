#ifndef GRSF_Dynamics_General_CartesianGrid_hpp
#define GRSF_Dynamics_General_CartesianGrid_hpp

#include "GRSF/Common/AssertionDebug.hpp"
#include "GRSF/Common/TypeDefs.hpp"

#include <set>
#include "GRSF/Dynamics/Collision/Geometry/AABB.hpp"

#include "GRSF/Common/SfinaeMacros.hpp"

struct NoCellData {};

template<typename TCellData = NoCellData,
         typename TSize    = std::size_t,
         typename TLongInt = long int>
class CartesianGrid {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DEFINE_MATRIX_TYPES

    using SizeType = TSize;   ///< The type of the indices
    using LongInt = TLongInt; ///< used for index computations (negative numebrs needed), int32 is not faster than int64!

    using CellDataType = TCellData;

    using IndexType     = typename MyMatrix<SizeType>::Array3;
    using IndexLongType = typename MyMatrix<TLongInt>::Array3;

    using CellDataListType = std::vector<TCellData>;

    CartesianGrid(){
        m_dxyz.setZero();
        m_dim.setZero();
    }

    CartesianGrid(const AABB3d & aabb,
                  const IndexType & dim){
        ASSERTMSG(dim(0)*dim(1)*dim(2) != 0, "Dimension zero: " << dim)
        ASSERTMSG( aabb.isEmpty() == false, "CartesianGrid, wrongly initialized: maxPoint < minPoint");
        m_aabb = aabb;
        m_dim = dim;

        m_dxyz    = m_aabb.extent() / dim.template cast<PREC>();
        m_dxyzInv = m_dxyz.inverse();

        if(! std::is_same<TCellData,NoCellData>::value){
            m_cellData.resize(m_dim(0)*m_dim(1)*m_dim(2));
        }

    }

    ~CartesianGrid(){}

    inline IndexType getDimensions(){return m_dim;}
    inline Array3 getDx(){return m_dxyz;}
    inline Array3 getDxInv(){return m_dxyzInv;}

    /** Get cell index, points needs to be in same frame as aabb in this class */
    template<typename Derived>
    bool getCellIndex(const MatrixBase<Derived> & point, IndexType & index) const
    {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,3);

        m_temp =  (((point - m_aabb.m_minPoint).array()) * m_dxyzInv).template cast<LongInt>();

        // If inside grid
        if(  !( (m_temp < 0).any() || (m_temp >= m_dim.template cast<LongInt>() ).any()) ) {
           index = m_temp.template cast<SizeType>();
           return true;
        };
        return false;
    }

    template<typename Derived>
    IndexType getCellIndexClosest(const MatrixBase<Derived> & point) const{
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,3);

        // calculate index normally and then project it into the feasible grid.
        m_temp = (((point - m_aabb.m_minPoint).array()) * m_dxyzInv).template cast<LongInt>();

        // prox  index to feasible range (cartesian prox)
        //        m_temp(0) = std::max(   std::min( LongInt(m_dim(0)-1), m_temp(0)),  0LL   );
        //        m_temp(1) = std::max(   std::min( LongInt(m_dim(1)-1), m_temp(1)),  0LL   );
        //        m_temp(2) = std::max(   std::min( LongInt(m_dim(2)-1), m_temp(2)),  0LL   );
        // eigen is as fast
        m_temp = m_zero.max( (m_dim.template cast<LongInt>() -1).min(m_temp) );

        return m_temp.template cast<TSize>();
    };

    /** Get cell data  */
    template<typename Derived>
    typename CellDataListType::value_type * getCellData(const MatrixBase<Derived> & point){
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,3);

        if(!std::is_same<TCellData,NoCellData>::value){
            IndexType index;
            if(getCellIndex(point,index)){
                return &m_cellData[ getLinearIndex(index) ];
            }
        }
        return nullptr;
    }

    /** Get cell data  */
    template<typename Derived>
    typename CellDataListType::value_type * getCellData(const MatrixBase<Derived> & point,
                                                        IndexType & index)
    {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,3);
        if(!std::is_same<TCellData,NoCellData>::value){
            if(getCellIndex(point,index)){
                return &m_cellData[ getLinearIndex(index) ];
            }
        }
        return nullptr;
    }

    /** Get cell data closest  */
    template<typename Derived>
    TCellData * getCellDataClosest(const MatrixBase<Derived> & point) const {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,3);

        if(!std::is_same<TCellData,NoCellData>::value){
            IndexType index = getCellIndexClosest(point);
            return &m_cellData[ getLinearIndex(index) ];
        }
        return nullptr;
    }

    template<typename Derived,
             bool indexCheck = true,
             SFINAE_ENABLE_IF(indexCheck) >
    TCellData * getCellData(const ArrayBase<Derived> & index) const
    {
        STATIC_ASSERT( (std::is_same<typename Derived::Scalar, SizeType >::value) );

        if(!std::is_same<TCellData,NoCellData>::value  &&
           ( !indexCheck || !( (index < 0).any() || (index >= m_dim).any() ) ) /* a decent compiler will optimize out if !indexCheck = true */
          )
        {
            return &m_cellData[ getLinearIndex(index)  ];
        }
        return nullptr;
    }

    template<typename Derived,
             bool indexCheck,
             SFINAE_ENABLE_IF(!indexCheck) >
    TCellData * getCellData(const ArrayBase<Derived> & index) const
    {
        if(!std::is_same<TCellData,NoCellData>::value)
        {
            return &m_cellData[ getLinearIndex(index) ];
        }
        return nullptr;
    }

    /** Get cell point in K frame of grid,
    *   P = 0: Bottom
    *   P = 1: Center
    *   P = 2: Top
    *   index tuple must be valid, and is not checked!
    */
    template<unsigned int P = 0, typename Derived>
    Vector3 getCellPoint(const ArrayBase<Derived> & index) const{
        STATIC_ASSERT(P<=2)
        if( P==0 ){
            return m_aabb.m_minPoint.array()              + m_dxyz * index.template cast<PREC>() ;
        }else if(P==1){
            return m_aabb.m_minPoint.array() + 0.5*m_dxyz + m_dxyz * index.template cast<PREC>() ;
        }else {
            return m_aabb.m_minPoint.array() + m_dxyz     + m_dxyz * index.template cast<PREC>();
        }
    }

    template<typename TVisitor>
    void applyVisitor(TVisitor && v){

        IndexType idx;

        for( SizeType x = 0; x < m_dim(0); ++x ){
            idx(0) = x;
            for( SizeType y = 0; y < m_dim(1); ++y ){
                idx(1) = y;
                for( SizeType z = 0; z < m_dim(2); ++z ){
                    idx(2) = z;
                    v( m_cellData[ getLinearIndex(x,y,z) ] , idx );
                }
            }
        }

    }



protected:
    template<typename T1,typename T2, typename T3>
    inline SizeType getLinearIndex(T1 x, T2 y, T3 z){
        return x + m_dim(0)*y + m_dim(0)*m_dim(1)*z;
    }

    template<typename Derived>
    inline SizeType getLinearIndex(const ArrayBase<Derived> & index){
        return index(0) + m_dim(0)*index(1) + m_dim(0)*m_dim(1)*index(2);
    }

    CellDataListType m_cellData;

    Array3 m_dxyzInv;
    Array3 m_dxyz;
    IndexType m_dim;
    AABB3d m_aabb;

    // temporary
    mutable IndexLongType m_temp;

    static IndexLongType m_zero;
    static char m_nbIndicesOff[26*3];
};

template<typename TCellData, typename TSize, typename TLongInt>
typename CartesianGrid<TCellData,TSize,TLongInt>::IndexLongType CartesianGrid<TCellData,TSize,TLongInt>::m_zero = CartesianGrid<TCellData,TSize,TLongInt>::IndexLongType(0,0,0);

template<typename TCellData, typename TSize, typename TLongInt>
char CartesianGrid<TCellData,TSize, TLongInt>::m_nbIndicesOff[26*3] = {
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
