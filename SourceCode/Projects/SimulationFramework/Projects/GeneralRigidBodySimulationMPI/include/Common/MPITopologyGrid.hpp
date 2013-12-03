#ifndef MPITopologyGrid_hpp
#define MPITopologyGrid_hpp

#include "AssertionDebug.hpp"
#include "TypeDefs.hpp"

#include <boost/variant.hpp>
#include <vector>
#include <map>

#include "CartesianGrid.hpp"
#include "AABB.hpp"
#include "Collider.hpp"

#include "MPITopologyVisitors.hpp"

namespace MPILayer{

template< typename ProcessTopologyBase>
class ProcessTopologyGrid {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    typedef std::map<unsigned int, AABB > RankToAABBType;
    typedef typename ProcessTopologyBase::NeighbourRanksListType NeighbourRanksListType;
    typedef typename ProcessTopologyBase::AdjacentNeighbourRanksMapType AdjacentNeighbourRanksMapType;

    ProcessTopologyGrid(  NeighbourRanksListType & nbRanks, AdjacentNeighbourRanksMapType & adjNbRanks,
                          const Vector3 & minPoint,
                          const Vector3 & maxPoint,
                          const MyMatrix<unsigned int>::Vector3 & dim,
                          RankIdType processRank, unsigned int masterRank):
    m_nbRanks(nbRanks), m_rank(processRank), m_adjNbRanks(adjNbRanks), m_grid(minPoint,maxPoint, dim, masterRank)
    {
       m_rank = processRank;

        //Initialize neighbours
       m_nbRanks = m_grid.getCellNeigbours(m_rank);

        for( auto it = m_nbRanks.begin(); it!=m_nbRanks.end(); it++){

            //Initialize adjacent neighbour ranks to m_nbRanks for this neighbour *it
           m_adjNbRanks[*it] = m_grid.getCommonNeighbourCells(m_nbRanks, *it);

            //Get all AABB's of all neighbours
            m_nbAABB[ *it ] =  m_grid.getCellAABB(*it) ;
        }


        //Get AABB of own rank!
        m_aabb = m_grid.getCellAABB(this->m_rank);
    };

private:

    template<typename T> friend class TopologyVisitors::BelongsPointToProcess;
    template<typename T> friend class TopologyVisitors::CheckOverlap;

    RankIdType m_rank; ///< Own rank;

    // External References to change!
    NeighbourRanksListType & m_nbRanks; ///< Neighbour ranks
    AdjacentNeighbourRanksMapType & m_adjNbRanks; ///< Adjacent ranks between m_rank and each neighbour

    RankToAABBType m_nbAABB; ///< Neighbour AABB
    AABB m_aabb; ///< Own AABB of this process
    CartesianGrid<NoCellData> m_grid;

    Collider m_Collider;
};

}; //MPILayer

#endif // MPITopologyGrid_hpp
