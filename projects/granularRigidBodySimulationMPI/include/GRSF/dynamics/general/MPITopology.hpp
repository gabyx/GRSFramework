// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_MPITopology_hpp
#define GRSF_dynamics_general_MPITopology_hpp

#include <mpi.h>

#include "GRSF/common/Asserts.hpp"
#include "GRSF/common/TypeDefs.hpp"

//#include "GRSF/dynamics/general/MPIInformation.hpp"

#include "GRSF/dynamics/general/MPITopologyGrid.hpp"
#include "GRSF/dynamics/general/MPITopologyKdTree.hpp"
#include "GRSF/dynamics/general/MPITopologyVisitors.hpp"

namespace MPILayer
{
/**
* Base class for all Topologies, we avoided virtual function dispatch here (why, not so clear, because of overhead...)
* A nicer interface would be with dynamic polymorphism instead of using a variant class as member to do the member
* function dispatch
*/
class ProcessTopology
{
public:
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    using NeighbourRanksListType        = std::unordered_set<RankIdType>;
    using AdjacentNeighbourRanksMapType = std::unordered_map<RankIdType, NeighbourRanksListType>;

private:
    /** Delete visitor for the variant */
    struct Deleter : public boost::static_visitor<void>
    {
        template <typename T>
        inline void operator()(T* t) const
        {
            delete t;
        }
        inline void operator()(boost::blank& b) const
        {
        }
    };

public:
    ProcessTopology()
    {
    }

    ~ProcessTopology()
    {
    }

    void init(RankIdType rank)
    {
        m_rank = rank;
    }

    // Main function
    inline bool belongsPointToProcess(const Vector3& point, RankIdType& ownerRank) const
    {
        TopologyVisitors::BelongsPointToProcess<ProcessTopology> vis(point, ownerRank);
        return m_procTopo.apply_visitor(vis);
    }

    inline bool belongsBodyToProcess(const RigidBodyType* body) const
    {
        RankIdType ownerRank;
        return belongsPointToProcess(body->m_r_S, ownerRank);
    }

    inline bool belongsBodyToProcess(const RigidBodyType* body, RankIdType& ownerRank) const
    {
        return belongsPointToProcess(body->m_r_S, ownerRank);
    }

    inline bool belongsPointToProcess(const Vector3& point) const
    {
        RankIdType ownerRank;
        return belongsPointToProcess(point, ownerRank);
    }

    inline bool checkOverlap(const RigidBodyType* body,
                             NeighbourRanksListType& neighbourProcessRanks,
                             bool& overlapsOwnProcess)
    {
        TopologyVisitors::CheckOverlap<ProcessTopology> vis(body, neighbourProcessRanks, overlapsOwnProcess);
        return m_procTopo.apply_visitor(vis);
    }

    const NeighbourRanksListType& getNeighbourRanks() const
    {
        return m_nbRanks;
    }

    const NeighbourRanksListType& getAdjacentNeighbourRanks(RankIdType neighbourRank) const
    {
        GRSF_ASSERTMSG(m_nbRanks.find(neighbourRank) != m_nbRanks.end(),
                       "No neighbour rank: " << neighbourRank << " for this process rank: " << m_rank << "!");
        GRSF_ASSERTMSG(
            m_adjNbRanks.find(neighbourRank) != m_adjNbRanks.end(),
            "No adjacent ranks for this neighbour: " << neighbourRank << "for process rank: " << m_rank << "!");
        return m_adjNbRanks.find(neighbourRank)->second;
    }

    template <typename... T>
    void createProcessTopologyGrid(RankIdType masterRank, T&&... args
                                   /*const AABB3d & aabb,
                                   const MyMatrix::Array3<unsigned int> & dim,
                                   bool aligned = true,
                                   const Matrix33 & A_IK = Matrix33::Identity()*/
                                   )
    {
        // Assign a grid topology
        Deleter d;
        m_procTopo.apply_visitor(d);

        m_nbRanks.clear();
        m_adjNbRanks.clear();
        m_procTopo = new ProcessTopologyGrid<ProcessTopology>(
            m_nbRanks, m_adjNbRanks, m_rank, masterRank, std::forward<T>(args)...);
    }

    template <typename... T>
    void createProcessTopologyKdTree(RankIdType masterRank, T&&... args
                                     /*std::unique_ptr<Tree> tree,
                                      LeafNeighbourMapType & neighbours,
                                      const AABB3d & aabb,
                                      bool aligned ,
                                      const Matrix33 & A_IK = Matrix33::Identity()*/
                                     )
    {
        // Assign a kdTree topology
        Deleter d;
        m_procTopo.apply_visitor(d);

        m_nbRanks.clear();
        m_adjNbRanks.clear();
        m_procTopo = new ProcessTopologyKdTree<ProcessTopology>(
            m_nbRanks, m_adjNbRanks, m_rank, masterRank, std::forward<T>(args)...);
    }

private:
    boost::variant<boost::blank, ProcessTopologyGrid<ProcessTopology>*, ProcessTopologyKdTree<ProcessTopology>*>
        m_procTopo;

    RankIdType m_rank;

    // These values are set by the create Functions for different topologies
    NeighbourRanksListType m_nbRanks;            ///< Neighbour ranks
    AdjacentNeighbourRanksMapType m_adjNbRanks;  ///< Adjacent ranks between m_rank and each neighbour
};

};  // MPILayer

#endif
