#ifndef CartesianGrid_hpp
#define CartesianGrid_hpp

#include "AssertionDebug.hpp"
#include "TypeDefs.hpp"

#include "AABB.hpp"

template<typename TLayoutConfig, typename TCellData> class CartesianGrid;

struct NoCellData {};

template<typename TLayoutConfig>
class CartesianGrid<TLayoutConfig,NoCellData> {

    DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    CartesianGrid() {
        m_dxyz.setZero();
        m_dim.setZero();
        m_cellNumberingStart = 0;
    };

    CartesianGrid(const Vector3 & minPoint,
                  const Vector3 & maxPoint,
                  const MyMatrix<unsigned int>::Vector3 & dim, unsigned int cellNumberingStart ) {
        ASSERTMSG( maxPoint(0) > minPoint(0) && maxPoint(1) > minPoint(1) && maxPoint(2) > minPoint(2), "CartesianGrid, wrongly initialized: maxPoint < minPoint");
        m_Box.m_minPoint = minPoint;
        m_Box.m_maxPoint = maxPoint;
        m_dim = dim;

        m_dxyz.array() = m_Box.extent().array() / dim.array().template cast<PREC>();

        m_cellNumberingStart = cellNumberingStart;
    };

    const MyMatrix<unsigned int>::Vector3 getCellIndex(const Vector3 & point) const {
        ASSERTMSG(m_Box.inside(point),"Point is not inside the Grid!");
        MyMatrix<unsigned int>::Vector3 v;
        v.array() =  (((point - m_Box.m_minPoint).array()) / m_dxyz.array()).template cast<unsigned int>();
        return v;
    };

    unsigned int getCellNumber(const Vector3 & point) const {
        ASSERTMSG(m_Box.inside(point),"Point is not inside the Grid!");
        MyMatrix<unsigned int>::Vector3 v = getCellIndex(point);
        return v(0) + v(1)*m_dim(0) + v(2) *(m_dim(0)*m_dim(1)) + m_cellNumberingStart;

    };

    std::vector<unsigned int> getCellNeigbours(unsigned int cellNumber) const {
        std::vector<unsigned int> v;
        // cellNumber zero indexed
        ASSERTMSG(cellNumber < m_dim(0)*m_dim(1)*m_dim(2) && cellNumber >= 0,"cellNumber: " << cellNumber <<" not in Dimension: "<<m_dim(0)<<","<<m_dim(1)<<","<<m_dim(2)<<std::endl );

        unsigned int neigbourCell;
        unsigned z_index,y_index,x_index, cellNumberTemp;

        cellNumberTemp = cellNumber;
        z_index = cellNumberTemp / (m_dim(0)*m_dim(1));

        cellNumberTemp -= z_index*(m_dim(0)*m_dim(1));
        y_index = cellNumberTemp / (m_dim(0));

        cellNumberTemp -= y_index*(m_dim(0));
        x_index = cellNumberTemp;

        for(int i=0;i<26;i++){
            unsigned int x_ind = m_nbIndicesOff[i*3+0] + x_index;
            unsigned int y_ind = m_nbIndicesOff[i*3+1] + y_index;
            unsigned int z_ind = m_nbIndicesOff[i*3+2] + z_index;

            if( (x_ind >=0 && x_ind < m_dim(0)) &&
                (y_ind >=0 && y_ind < m_dim(1)) &&
                (z_ind >=0 && z_ind < m_dim(2)) )
                {
                    // Add neighbour
                    cellNumberTemp = x_ind + y_ind*m_dim(0) + z_ind *(m_dim(0)*m_dim(1))  +  m_cellNumberingStart;
                    v.push_back(cellNumberTemp);
                }

        }
        return v;
    }


private:

    unsigned int m_cellNumberingStart;

    Vector3 m_dxyz;
    MyMatrix<unsigned int>::Vector3 m_dim;

    AABB<TLayoutConfig> m_Box;

    static char m_nbIndicesOff[26*3];
};

  template<typename TLayoutConfig>
  char CartesianGrid<TLayoutConfig,NoCellData>::m_nbIndicesOff[26*3] =
                                    { 1,0,0,
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
