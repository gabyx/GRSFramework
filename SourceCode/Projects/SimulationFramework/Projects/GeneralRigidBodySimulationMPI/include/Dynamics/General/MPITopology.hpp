#ifndef MPITopology_hpp
#define MPITopology_hpp

#include <mpi.h>

#include "AssertionDebug.hpp"
#include "TypeDefs.hpp"

//#include "MPIInformation.hpp"

#include "MPITopologyGrid.hpp"
#include "MPITopologyVisitors.hpp"

namespace MPILayer {

class ProcessTopology {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    typedef std::set<RankIdType> NeighbourRanksListType;
    typedef std::unordered_map<RankIdType, NeighbourRanksListType> AdjacentNeighbourRanksMapType;

    ProcessTopology(){}

    ~ProcessTopology(){
    }

    void init(RankIdType rank){m_rank = rank;}

    // Main function
    inline bool belongsPointToProcess(const Vector3 & point, RankIdType &neighbourProcessRank) const {
        TopologyVisitors::BelongsPointToProcess<ProcessTopology> vis(point,neighbourProcessRank);
        return m_procTopo.apply_visitor(vis);
    }

    inline bool belongsBodyToProcess(const RigidBodyType * body) const {
        RankIdType nb;
        return belongsPointToProcess(body->m_r_S,nb);
    }

    inline bool belongsBodyToProcess(const RigidBodyType * body, RankIdType &neighbourProcessRank) const {
        return belongsPointToProcess(body->m_r_S,neighbourProcessRank);
    }

    inline bool belongsPointToProcess(const Vector3 & point) const {
        unsigned int nb;
        return belongsPointToProcess(point,nb);
    }

    inline bool checkOverlap(   const RigidBodyType * body,
                                NeighbourRanksListType & neighbourProcessRanks,
                                bool & overlapsOwnProcess)
    {
        TopologyVisitors::CheckOverlap<ProcessTopology> vis(body,neighbourProcessRanks,overlapsOwnProcess);
        return m_procTopo.apply_visitor(vis);
    }


    const NeighbourRanksListType & getNeighbourRanks() const {
        return m_nbRanks;
    }

    const NeighbourRanksListType & getAdjacentNeighbourRanks(RankIdType neighbourRank) const {
        ASSERTMSG(  m_nbRanks.find(neighbourRank) !=  m_nbRanks.end(),
                  "No neighbour rank: " << neighbourRank << " for this process rank: "<< m_rank<<"!");
        ASSERTMSG( m_adjNbRanks.find(neighbourRank) != m_adjNbRanks.end(),
                  "No adjacent ranks for this neighbour: "<< neighbourRank << "for process rank: " << m_rank<<"!");
        return m_adjNbRanks.find(neighbourRank)->second;
    }

    void createProcessTopologyGrid(const Vector3 & minPoint,
                                   const Vector3 & maxPoint,
                                   const MyMatrix<unsigned int>::Vector3 & dim,
                                   unsigned int processRank, unsigned int masterRank)
    {
        // Assign a new Topology
        m_procTopo = ProcessTopologyGrid<ProcessTopology>(m_nbRanks,m_adjNbRanks, minPoint,maxPoint,dim, m_rank , masterRank);
    }

    private:

    boost::variant<boost::blank, ProcessTopologyGrid<ProcessTopology> > m_procTopo;
    RankIdType m_rank;

    // These values are set with call by references in the specific implementation classes
    NeighbourRanksListType  m_nbRanks;           ///< Neighbour ranks
    AdjacentNeighbourRanksMapType  m_adjNbRanks; ///< Adjacent ranks between m_rank and each neighbour

};


}; //MPILayer

#endif
