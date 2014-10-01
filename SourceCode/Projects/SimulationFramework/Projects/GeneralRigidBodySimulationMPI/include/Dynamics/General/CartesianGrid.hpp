#ifndef CartesianGrid_hpp
#define CartesianGrid_hpp

#include "AssertionDebug.hpp"
#include "TypeDefs.hpp"

#include <set>
#include "AABB.hpp"



struct NoCellData {};

template<typename TCellData = NoCellData>
class CartesianGrid {

    DEFINE_MATRIX_TYPES
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CartesianGrid();

    CartesianGrid(const Vector3 & minPoint,
                  const Vector3 & maxPoint,
                  const MyMatrix<unsigned int>::Vector3 & dim);
    ~CartesianGrid();



    /** Get cell index */
    MyMatrix<unsigned int>::Vector3 getCellIndex(const Vector3 & point) const;
    MyMatrix<unsigned int>::Vector3 getCellIndexClosest(const Vector3 & point) const;

    /** Get cell data  */
    TCellData * getCellData(const Vector3 & point) const;
    TCellData * getCellData(const MyMatrix<unsigned int>::Vector3 & index) const;

protected:



    Vector3 m_dxyz;
    MyMatrix<unsigned int>::Vector3 m_dim;

    AABB m_Box;

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
CartesianGrid<TCellData>::CartesianGrid(const Vector3 & minPoint,
                                        const Vector3 & maxPoint,
                                        const MyMatrix<unsigned int>::Vector3 & dim) {
    ASSERTMSG(dim(0)*dim(1)*dim(2) != 0, "Dimension zero: " << dim)
    ASSERTMSG( maxPoint(0) > minPoint(0) && maxPoint(1) > minPoint(1) && maxPoint(2) > minPoint(2), "CartesianGrid, wrongly initialized: maxPoint < minPoint");
    m_Box.m_minPoint = minPoint;
    m_Box.m_maxPoint = maxPoint;
    m_dim = dim;

    m_dxyz.array() = m_Box.extent().array() / dim.array().cast<PREC>();

    if(! std::is_same<TCellData,NoCellData>::value){
        m_cellData.resize(m_dim(0)*m_dim(1)*m_dim(2));
    }

};

template<typename TCellData>
MyMatrix<unsigned int>::Vector3 CartesianGrid<TCellData>::getCellIndex(const Vector3 & point) const {
    ASSERTMSG(m_Box.inside(point),"Point: " << point << " is not inside the Grid!");
    MyMatrix<unsigned int>::Vector3 v;
    v.array() =  (((point - m_Box.m_minPoint).array()) / m_dxyz.array()).cast<unsigned int>();
    ASSERTMSG( ( (v(0) >=0 && v(0) < m_dim(0)) && (v(1) >=0 && v(1) < m_dim(1)) && (v(2) >=0 && v(2) < m_dim(2)) ),
              "Index: " << v << " is out of bound" )
    return v;
};

template<typename TCellData>
MyMatrix<unsigned int>::Vector3 CartesianGrid<TCellData>::getCellIndexClosest(const Vector3 & point) const {

    // calculate index normally and then project it into the feasible grid.
    MyMatrix<int64_t>::Vector3 v;
    v.array() =  (((point - m_Box.m_minPoint).array()) / m_dxyz.array()).cast<int64_t>();

    // prox  index to feasible range (cartesian prox)
    v(0) = std::max(   std::min( int64_t(m_dim(0)-1), v(0)),  0L   );
    v(1) = std::max(   std::min( int64_t(m_dim(1)-1), v(1)),  0L   );
    v(2) = std::max(   std::min( int64_t(m_dim(2)-1), v(2)),  0L   );

    return v.cast<unsigned int>();
};

template<typename TCellData>
TCellData * CartesianGrid<TCellData>::getCellData(const Vector3 & point) const {
    if(std::is_same<TCellData,NoCellData>::value){
            return nullptr;
    }else{
            auto i = getCellIndex(point);
            return &m_cellData[i(0) + m_dim(1)*i(1) + m_dim(1)*m_dim(2)*i(2) ];
    }
};

template<typename TCellData>
TCellData* CartesianGrid<TCellData>::getCellData(const MyMatrix<unsigned int>::Vector3 & index) const {
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
