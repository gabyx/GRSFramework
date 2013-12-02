#ifndef CartesianGrid_hpp
#define CartesianGrid_hpp

#include "AssertionDebug.hpp"
#include "TypeDefs.hpp"

#include "AABB.hpp"

template<typename TCellData> class CartesianGrid;

struct NoCellData {};

template<>
class CartesianGrid<NoCellData> {

    DEFINE_LAYOUT_CONFIG_TYPES

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    CartesianGrid();

    CartesianGrid(const Vector3 & minPoint,
                  const Vector3 & maxPoint,
                  const MyMatrix<unsigned int>::Vector3 & dim, unsigned int cellNumberingStart );

    MyMatrix<unsigned int>::Vector3 getCellIndex(const Vector3 & point) const;
    MyMatrix<unsigned int>::Vector3 getCellIndex(unsigned int cellNumber) const;

    unsigned int getCellNumber(const Vector3 & point) const;
    unsigned int getCellNumber(const MyMatrix<unsigned int>::Vector3 & index) const;

    AABB getCellAABB(unsigned int cellNumber) const;

    std::set<unsigned int> getCellNeigbours(unsigned int cellNumber) const;

    /**
    * Gets the common cells between all cellNumbers and the neighbours of cell number cellNumber2
    */
    std::set<unsigned int> getCommonNeighbourCells(const std::set<unsigned int> & cellNumbers, unsigned int cellNumber2) const;


private:

    unsigned int m_cellNumberingStart;

    Vector3 m_dxyz;
    MyMatrix<unsigned int>::Vector3 m_dim;

    AABB m_Box;

    static char m_nbIndicesOff[26*3];
};


CartesianGrid<NoCellData>::CartesianGrid() {
    m_dxyz.setZero();
    m_dim.setZero();
    m_cellNumberingStart = 0;
};


CartesianGrid<NoCellData>::CartesianGrid(const Vector3 & minPoint,
        const Vector3 & maxPoint,
        const MyMatrix<unsigned int>::Vector3 & dim, unsigned int cellNumberingStart ) {
    ASSERTMSG( maxPoint(0) > minPoint(0) && maxPoint(1) > minPoint(1) && maxPoint(2) > minPoint(2), "CartesianGrid, wrongly initialized: maxPoint < minPoint");
    m_Box.m_minPoint = minPoint;
    m_Box.m_maxPoint = maxPoint;
    m_dim = dim;

    m_dxyz.array() = m_Box.extent().array() / dim.array().cast<PREC>();

    m_cellNumberingStart = cellNumberingStart;
};


MyMatrix<unsigned int>::Vector3 CartesianGrid<NoCellData>::getCellIndex(const Vector3 & point) const {
    ASSERTMSG(m_Box.inside(point),"Point: " << point << " is not inside the Grid!");
    MyMatrix<unsigned int>::Vector3 v;
    v.array() =  (((point - m_Box.m_minPoint).array()) / m_dxyz.array()).cast<unsigned int>();
    ASSERTMSG( ( (v(0) >=0 && v(0) < m_dim(0)) && (v(1) >=0 && v(1) < m_dim(1)) && (v(2) >=0 && v(2) < m_dim(2)) ),
              "Index: " << v << " is out of bound" )
    return v;
};


unsigned int CartesianGrid<NoCellData>::getCellNumber(const Vector3 & point) const {
    ASSERTMSG(m_Box.inside(point),"Point: " << point << " is not inside the Grid!");
    MyMatrix<unsigned int>::Vector3 v = getCellIndex(point);
    return getCellNumber(v);

};


unsigned int CartesianGrid<NoCellData>::getCellNumber(const MyMatrix<unsigned int>::Vector3 & v) const {
    ASSERTMSG( ( (v(0) >=0 && v(0) < m_dim(0)) && (v(1) >=0 && v(1) < m_dim(1)) && (v(2) >=0 && v(2) < m_dim(2)) ),
              "Index: " << v << " is out of bound" )

    unsigned int cellNumber = v(0) + v(1)*m_dim(0) + v(2) *(m_dim(0)*m_dim(1)) + m_cellNumberingStart;
    ASSERTMSG(cellNumber < m_dim(0)*m_dim(1)*m_dim(2) && cellNumber >= 0,"cellNumber: " << cellNumber <<" not in Dimension: "<<m_dim(0)<<","<<m_dim(1)<<","<<m_dim(2)<<std::endl );
    return cellNumber;

};


AABB CartesianGrid<NoCellData>::getCellAABB(unsigned int cellNumber) const{

     MyMatrix<unsigned int>::Vector3 cell_index = getCellIndex(cellNumber);
     Vector3 pL = cell_index.array().cast<PREC>() * m_dxyz.array();
     pL += m_Box.m_minPoint;
     Vector3 pU = (cell_index.array()+1).cast<PREC>()  * m_dxyz.array();
     pU += m_Box.m_minPoint;
     return AABB(pL,pU);
};



MyMatrix<unsigned int>::Vector3 CartesianGrid<NoCellData>::getCellIndex(unsigned int cellNumber) const{

    ASSERTMSG(cellNumber < m_dim(0)*m_dim(1)*m_dim(2) && cellNumber >= 0,"cellNumber: " << cellNumber <<" not in Dimension: "<<m_dim(0)<<","<<m_dim(1)<<","<<m_dim(2)<<std::endl );
    MyMatrix<unsigned int>::Vector3 v;
    unsigned int cellNumberTemp;

    cellNumberTemp = cellNumber;
    v(2) = cellNumberTemp / (m_dim(0)*m_dim(1));

    cellNumberTemp -= v(2)*(m_dim(0)*m_dim(1));
    v(1) = cellNumberTemp / (m_dim(0));

    cellNumberTemp -= v(1)*(m_dim(0));
    v(0) = cellNumberTemp;

    return v;
};


std::set<unsigned int> CartesianGrid<NoCellData>::getCellNeigbours(unsigned int cellNumber) const {
    std::set<unsigned int> v;
    // cellNumber zero indexed
    ASSERTMSG(cellNumber < m_dim(0)*m_dim(1)*m_dim(2) && cellNumber >= 0,"cellNumber: " << cellNumber <<" not in Dimension: "<<m_dim(0)<<","<<m_dim(1)<<","<<m_dim(2)<<std::endl );

    MyMatrix<unsigned int>::Vector3 cell_index = getCellIndex(cellNumber);

    MyMatrix<unsigned int>::Vector3 ind;

    for(int i=0; i<26; i++) {
        ind(0) = m_nbIndicesOff[i*3+0] + cell_index(0);
        ind(1) = m_nbIndicesOff[i*3+1] + cell_index(1);
        ind(2) = m_nbIndicesOff[i*3+2] + cell_index(2);

        if( ( ind(0) >=0 &&  ind(0) < m_dim(0)) &&
            ( ind(1) >=0 &&  ind(1) < m_dim(1)) &&
            ( ind(2) >=0 &&  ind(2) < m_dim(2)) ) {
            // Add neighbour
            std::pair< typename std::set<unsigned int>::iterator, bool> res =
                    v.insert(getCellNumber(ind));
            ASSERTMSG(res.second,"This neighbour number: "<< getCellNumber(ind) << " for cell number: "<< cellNumber <<" alreads exists!");
        }

    }
    return v;
};


std::set<unsigned int> CartesianGrid<NoCellData>::getCommonNeighbourCells(const std::set<unsigned int> & cellNumbers,
                                                                                 unsigned int cellNumber2) const
{

    std::set<unsigned int> nbRanks = getCellNeigbours(cellNumber2);

    std::set<unsigned int> intersec;
    // intersect nbRanks with cellNumbers
    set_intersection(cellNumbers.begin(),cellNumbers.end(),nbRanks.begin(),nbRanks.end(),
                  std::inserter(intersec,intersec.begin()));

    return intersec;
};



char CartesianGrid<NoCellData>::m_nbIndicesOff[26*3] = {
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
