#ifndef GRSF_Dynamics_General_MPITopology_hpp
#define GRSF_Dynamics_General_MPITopology_hpp

#include <mpi.h>

#include "GRSF/Common/AssertionDebug.hpp"
#include "GRSF/Common/TypeDefs.hpp"

//#include "GRSF/Dynamics/General/MPIInformation.hpp"

#include "GRSF/Dynamics/General/MPITopologyGrid.hpp"
#include "GRSF/Dynamics/General/MPITopologyKdTree.hpp"
#include "GRSF/Dynamics/General/MPITopologyVisitors.hpp"

namespace MPILayer {

/**
* Base class for all Topologies, we avoided virtual function dispatch here (why, not so clear, because of overhead...)
* A nicer interface would be with dynamic polymorphism instead of using a variant class as member to do the member function dispatch
*/
class ProcessTopology {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    using NeighbourRanksListType = std::set<RankIdType>;
    using AdjacentNeighbourRanksMapType = std::unordered_map<RankIdType, NeighbourRanksListType>;

    private:
        /** Delete visitor for the variant */
        struct Deleter: public boost::static_visitor<void>{
            template<typename T>
            inline void operator()(T * t) const
            {
                delete t;
            }
            inline void operator()(boost::blank & b) const{}
        };

    public:

    ProcessTopology(){}

    ~ProcessTopology(){
    }

    void init(RankIdType rank){m_rank = rank;}

    // Main function
    inline bool belongsPointToProcess(const Vector3 & point, RankIdType &ownerRank) const {
        TopologyVisitors::BelongsPointToProcess<ProcessTopology> vis(point,ownerRank);
        return m_procTopo.apply_visitor(vis);
    }

    inline bool belongsBodyToProcess(const RigidBodyType * body) const {
        RankIdType ownerRank;
        return belongsPointToProcess(body->m_r_S,ownerRank);
    }

    inline bool belongsBodyToProcess(const RigidBodyType * body, RankIdType &ownerRank) const {
        return belongsPointToProcess(body->m_r_S,ownerRank);
    }

    inline bool belongsPointToProcess(const Vector3 & point) const {
        RankIdType ownerRank;
        return belongsPointToProcess(point,ownerRank);
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

    void createProcessTopologyGrid(unsigned int processRank, unsigned int masterRank,
                                   const AABB3d & aabb,
                                   const MyMatrix<unsigned int>::Array3 & dim,
                                   bool aligned = true,
                                   const Matrix33 & A_IK = Matrix33::Identity()
                                   )
    {
        // Assign a grid topology
        Deleter d;
        m_procTopo.apply_visitor(d);
        m_procTopo = new ProcessTopologyGrid<ProcessTopology>(m_nbRanks,m_adjNbRanks,
                                                          m_rank , masterRank,
                                                          aabb,dim,
                                                          aligned,
                                                          A_IK);

    }

    template<typename Tree>
    void createProcessTopologyKdTree(unsigned int processRank,
                                      unsigned int masterRank,
                                      std::unique_ptr<Tree> tree,
                                      const AABB3d & aabb,
                                      bool aligned = true,
                                      const Matrix33 & A_IK = Matrix33::Identity()

                                       )
    {
        // Assign a kdTree topology
        Deleter d;
        m_procTopo.apply_visitor(d);
        m_procTopo = new ProcessTopologyKdTree<ProcessTopology>(  m_nbRanks,m_adjNbRanks,
                                                                  m_rank , masterRank,
                                                                  std::move(tree),
                                                                  aabb,
                                                                  aligned,
                                                                  A_IK
                                                                  );

    }

    private:

    boost::variant<boost::blank,
                    ProcessTopologyGrid<ProcessTopology>  * ,
                    ProcessTopologyKdTree<ProcessTopology> *
                > m_procTopo;

    RankIdType m_rank;

    // These values are set by the create Functions for different topologies
    NeighbourRanksListType  m_nbRanks;           ///< Neighbour ranks
    AdjacentNeighbourRanksMapType  m_adjNbRanks; ///< Adjacent ranks between m_rank and each neighbour

};


}; //MPILayer

#endif
