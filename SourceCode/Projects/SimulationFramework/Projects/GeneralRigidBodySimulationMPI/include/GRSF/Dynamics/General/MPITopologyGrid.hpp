#ifndef GRSF_Dynamics_General_MPITopologyGrid_hpp
#define GRSF_Dynamics_General_MPITopologyGrid_hpp

#include "GRSF/Common/AssertionDebug.hpp"
#include "GRSF/Common/TypeDefs.hpp"

#include <boost/variant.hpp>
#include <vector>
#include <map>

#include "GRSF/Common/UnorderedContainerHelpers.hpp"

#include "GRSF/Dynamics/General/CartesianGrid.hpp"
#include "GRSF/Dynamics/Collision/Geometry/AABB.hpp"
#include "GRSF/Dynamics/Collision/Collider.hpp"

#include "GRSF/Dynamics/General/MPITopologyVisitors.hpp"

namespace MPILayer {


template< typename ProcessTopologyBase>
class ProcessTopologyGrid : protected CartesianGrid<NoCellData, typename ProcessTopologyBase::RankIdType > {
public:

    DEFINE_MPI_INFORMATION_CONFIG_TYPES
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Base = CartesianGrid<NoCellData, typename ProcessTopologyBase::RankIdType >;
    using IndexType = typename Base::IndexType;

    using RankToAABBType = std::map<unsigned int, AABB3d >;
    using NeighbourRanksListType = typename ProcessTopologyBase::NeighbourRanksListType;
    using AdjacentNeighbourRanksMapType = typename ProcessTopologyBase::AdjacentNeighbourRanksMapType;

private:

    using Base::m_dim;
    using Base::m_dxyz;
    using Base::m_dxyzInv;
    using Base::m_cellData;
    using Base::m_nbIndicesOff;
    using Base::m_A_KI;

public:

    ProcessTopologyGrid(  NeighbourRanksListType & nbRanks, AdjacentNeighbourRanksMapType & adjNbRanks,
                          RankIdType processRank, RankIdType masterRank,
            			  const AABB3d & aabb,
                          const IndexType & dim,
                          bool aligned = true,
                          const Matrix33 & A_IK = Matrix33::Identity()
                          ):
    Base(aabb, dim, A_IK.transpose()), /** Grid wants A_KI */
    m_cellNumberingStart(masterRank), m_rank(processRank),
    m_axisAligned(aligned)
    {
       m_rank = processRank;

        //Initialize neighbours
        nbRanks = getCellNeighbours(m_rank);

        for( auto it = nbRanks.begin(); it!=nbRanks.end(); it++) {

            //Initialize adjacent neighbour ranks to m_nbRanks for this neighbour *it
            adjNbRanks[*it] = unorderedHelpers::makeIntersection(nbRanks, getCellNeighbours(*it));

            //Get all AABB's of this neighbours
            m_nbAABB[ *it ] = getCellAABB(*it);
        }


        //Get AABB of own rank!
        m_aabb = getCellAABB(m_rank);

    };


    RankIdType getRank() const{return m_rank;}

    RankIdType getCellRank(const Vector3 & I_point) const {
        MyMatrix::Array3<RankIdType> v;
        if(m_axisAligned){
             v = Base::template getCellIndexClosest<false>(I_point); // do not transform point (grid aligned with I system)
        }else{
             v = Base::template getCellIndexClosest<true>(I_point);  // transform input point to K frame
        }
        return getCellRank(v);
    };


    RankIdType getCellRank(const MyMatrix::Array3<RankIdType> & v) const {
        ASSERTMSG( ( (v(0) >=0 && v(0) < m_dim(0)) && (v(1) >=0 && v(1) < m_dim(1)) && (v(2) >=0 && v(2) < m_dim(2)) ),
                "Index: " << v << " is out of bound" )

        unsigned int cellRank = v(0) + v(1)*m_dim(0) + v(2) *(m_dim(0)*m_dim(1)) + m_cellNumberingStart;

        ASSERTMSG(cellRank < m_dim(0)*m_dim(1)*m_dim(2) + m_cellNumberingStart
                && cellRank >= m_cellNumberingStart,
                "cellRank: " << cellRank <<" not in Dimension: "<< m_dim(0)<<","<< m_dim(1)<<","<< m_dim(2)<<std::endl );
        return cellRank;
    };


    NeighbourRanksListType getCellNeighbours(RankIdType cellRank) const {
        NeighbourRanksListType v;
        // cellRank zero indexed
        ASSERTMSG(cellRank < m_dim(0)*m_dim(1)*m_dim(2) + m_cellNumberingStart
                && cellRank >= m_cellNumberingStart,
                "cellRank: " << cellRank <<" not in Dimension: "<< m_dim(0)<<","<< m_dim(1)<<","<< m_dim(2)<<std::endl );

        MyMatrix::Array3<RankIdType> cell_index = getCellIndex(cellRank);

        MyMatrix::Array3<RankIdType> ind;

        for(int i=0; i<26; i++) {
            ind(0) = m_nbIndicesOff[i*3+0] + cell_index(0);
            ind(1) = m_nbIndicesOff[i*3+1] + cell_index(1);
            ind(2) = m_nbIndicesOff[i*3+2] + cell_index(2);

            if( ( ind(0) >=0 &&  ind(0) < m_dim(0)) &&
                    ( ind(1) >=0 &&  ind(1) < m_dim(1)) &&
                    ( ind(2) >=0 &&  ind(2) < m_dim(2)) ) {
                // Add neighbour
                auto res = v.insert(getCellRank(ind));
                ASSERTMSG(res.second,"This neighbour number: "<< getCellRank(ind) << " for cell number: "<< cellRank <<" alreads exists!");
            }

        }
        return v;
    };

    MyMatrix::Array3<RankIdType> getCellIndex(RankIdType cellRank) const {

        ASSERTMSG(cellRank < m_dim(0)*m_dim(1)*m_dim(2) + m_cellNumberingStart
                && cellRank >= m_cellNumberingStart,
                "cellRank: " << cellRank <<" not in Dimension: "<< m_dim(0)<<","<< m_dim(1)<<","<< m_dim(2)<<std::endl );

        MyMatrix::Array3<unsigned int> v;
        unsigned int cellNumberTemp;

        cellNumberTemp = cellRank;
        v(2) = cellNumberTemp / (m_dim(0)*m_dim(1));

        cellNumberTemp -= v(2)*(m_dim(0)*m_dim(1));
        v(1) = cellNumberTemp / (m_dim(0));

        cellNumberTemp -= v(1)*(m_dim(0));
        v(0) = cellNumberTemp;

        return v;
    };

    /**
    * Gets the AABB, which extends to infinity for boundary cells!
    */
    AABB3d getCellAABB(RankIdType cellRank) const {

        MyMatrix::Array3<unsigned int> cell_index = getCellIndex(cellRank);
        AABB3d ret(m_aabb.m_minPoint);
        ret.m_minPoint.array() += cell_index.array().cast<PREC>()     * m_dxyz.array();
        ret.m_maxPoint.array() += (cell_index.array()+1).cast<PREC>() * m_dxyz.array();

        //Expand AABB each axis to max/min if this rank is a boundary cell!
        for(short i = 0; i<3; i++) {
            if(cell_index(i) == m_dim(i)-1) {
                ret.expandToMaxExtent<true>(i);
            }
            if( cell_index(i) == 0 ) {
                ret.expandToMaxExtent<false>(i);
            }
        }

        return ret;
    };

    bool checkOverlap(const RigidBodyType * body,
                      NeighbourRanksListType & neighbourProcessRanks,
                      bool & overlapsOwnRank) const {
        if(m_axisAligned) {
            return checkOverlapImpl(m_ColliderAABB,neighbourProcessRanks, overlapsOwnRank, body);
        } else {
            return checkOverlapImpl(m_ColliderOOBB,neighbourProcessRanks, overlapsOwnRank, body, m_A_KI );
        }
    }


private:

    template<typename Collider, typename... AddArgs >
    inline bool checkOverlapImpl(Collider & collider,
                                    NeighbourRanksListType & neighbourProcessRanks,
                                    bool & overlapsOwnRank,
                                    const RigidBodyType * body,
                                    AddArgs&&... args) const
    {
        // Check neighbour AABB
        for(auto it = m_nbAABB.begin(); it != m_nbAABB.end(); it++) {
            if( collider.checkOverlap(body,it->second, std::forward<AddArgs>(args)...) ) {
                neighbourProcessRanks.insert(it->first);
            }
        }
        // Check own AABB
        overlapsOwnRank = collider.checkOverlap(body, m_aabb, std::forward<AddArgs>(args)...);
        return neighbourProcessRanks.size() > 0;
    }

    RankIdType m_cellNumberingStart;

    RankIdType m_rank; ///< Own rank;

    RankToAABBType m_nbAABB; ///< Neighbour AABB in frame G
    AABB3d m_aabb; ///< Own AABB of this process in frame G

    bool m_axisAligned = true;

    ColliderAABB m_ColliderAABB;
    ColliderOOBB m_ColliderOOBB;
};

}; //MPILayer

#endif // MPITopologyGrid_hpp
