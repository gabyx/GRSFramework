#ifndef CartesianGrid_hpp
#define CartesianGrid_hpp

#include "AssertionDebug.hpp"
#include "TypeDefs.hpp"

#include <set>
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



#endif
