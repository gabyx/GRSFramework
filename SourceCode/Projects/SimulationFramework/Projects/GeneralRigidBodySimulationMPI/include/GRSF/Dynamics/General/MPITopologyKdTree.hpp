#ifndef GRSF_Dynamics_General_MPITopologyKdTree_hpp
#define GRSF_Dynamics_General_MPITopologyKdTree_hpp

#include "GRSF/Common/AssertionDebug.hpp"
#include "GRSF/Common/TypeDefs.hpp"

#include <boost/variant.hpp>
#include <vector>
#include <map>

#include "GRSF/Dynamics/General/KdTree.hpp"

#include "GRSF/Dynamics/Collision/Geometry/AABB.hpp"
#include "GRSF/Dynamics/Collision/Collider.hpp"

#include "GRSF/Dynamics/General/MPITopologyVisitors.hpp"

namespace MPILayer{

template< typename ProcessTopologyBase>
class ProcessTopologyKdTree {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    using RankToAABBType = std::map<RankIdType, AABB3d >;
    using NeighbourRanksListType = typename ProcessTopologyBase::NeighbourRanksListType;
    using AdjacentNeighbourRanksMapType = typename ProcessTopologyBase::AdjacentNeighbourRanksMapType;

    using TreeType = KdTree::TreeSimpleS<>;
    using NodeType = typename TreeType::NodeType;
    using LeafNeighbourMapType = typename TreeType::LeafNeighbourMapType ;


    ProcessTopologyKdTree(  NeighbourRanksListType & nbRanks, AdjacentNeighbourRanksMapType & adjNbRanks,
                            RankIdType processRank, RankIdType masterRank,
                            std::unique_ptr<TreeType> tree,
                            const LeafNeighbourMapType & neighbours,
                            const AABB3d & aabb,
                            bool aligned,
                            const Matrix33 & A_IK = Matrix33::Identity())
            : m_rank(processRank), m_A_IK(A_IK), m_kdTree(tree.release())
    {

        // insert our neighbours
        auto it = neighbours.find(processRank);
        if(it == neighbours.end()){
            ERRORMSG("neighbours does not contain an entry for rank: " << processRank );
        }
        nbRanks.insert(it->second.begin(),it->second.end());

        // get adj neighbours
        for( auto it = nbRanks.begin(); it!=nbRanks.end(); ++it) {
            //Initialize adjacent neighbour ranks to m_nbRanks for this neighbours[*it]$
            auto itN = neighbours.find(*it);
             if(itN == neighbours.end()){
                ERRORMSG("neighbours does not contain an entry for rank: " << *it );
            }
            adjNbRanks.emplace(*it, getCommonNeighbourCells(nbRanks, itN->second) );
        }

        // get our aabb
        m_leaf = tree->getLeaf(m_rank);
        if(!m_leaf){
            ERRORMSG("Trying to get leaf node for our rank: " << m_rank << " failed!")
        }

        // adjust kdTree
        // TODO (unlink childs which corresponds to subtrees we never need to check because the leafs subtree are not neighbours)
        // not essential for correct algorithm

        // get lowest common ancestors of all boundary subtrees
        auto bndIt = m_leaf->getBoundaries().begin();
        m_lcaBoundary = nullptr;
        while( bndIt != m_leaf->getBoundaries().end()){
            if(m_lcaBoundary == nullptr){
                m_lcaBoundary = *bndIt;
            }
            else if( *bndIt != nullptr ){
                m_lcaBoundary = m_kdTree->getLowestCommonAncestor(m_lcaBoundary, *bndIt);
            }
            ++bndIt;
        }

        if(m_lcaBoundary == nullptr){
            ERRORMSG("Lowest common ancestor failed!");
        }

    };

    ProcessTopologyKdTree( ProcessTopologyKdTree && p): m_kdTree(p.m_kdTree) {
        p.m_kdTree = nullptr;
    }

    ~ProcessTopologyKdTree(){
        if(m_kdTree){
            delete m_kdTree;
        }
    }

    RankIdType getRank() const{return m_rank;}

    unsigned int getCellRank(const Vector3 & point) const {
        auto * leaf = m_kdTree->getLeaf(point);
        ASSERTMSG(leaf,"This should never be nullptr!");
        return leaf->getIdx();
    };


    bool checkOverlap(const RigidBodyType * body,
                      NeighbourRanksListType & neighbourProcessRanks,
                      bool & overlapsOwnRank) const {
        if(m_axisAligned) {

            m_colliderKdTree.checkOverlap(neighbourProcessRanks, m_lcaBoundary, body);
            overlapsOwnRank = m_colliderKdTree.checkOverlapNode(body,m_leaf);

        } else {
            m_colliderKdTree.checkOverlap(neighbourProcessRanks, m_lcaBoundary, body, m_A_IK );
            overlapsOwnRank = m_colliderKdTree.checkOverlapNode(body,m_leaf, m_A_IK);
        }

        return neighbourProcessRanks.size() > 0;
    }



private:

    /**
    * Gets the common cells between all cellNumbers and the neighbours of cell number cellNumber2
    */

    inline typename AdjacentNeighbourRanksMapType::mapped_type
    getCommonNeighbourCells(const NeighbourRanksListType & neighboursOurRanks,
                            const typename LeafNeighbourMapType::mapped_type & neighboursOtherRank) const {

        typename AdjacentNeighbourRanksMapType::mapped_type intersec;
        // intersect nbRanks with cellNumbers
        std::set_intersection(neighboursOurRanks.begin(), neighboursOurRanks.end(),
                              neighboursOtherRank.begin(),neighboursOtherRank.end(),
                std::inserter(intersec,intersec.begin()));

        return intersec;
    };

    template<typename T> friend class TopologyVisitors::BelongsPointToProcess;
    template<typename T> friend class TopologyVisitors::CheckOverlap;

    RankIdType m_rank; ///< Own rank;

    bool m_axisAligned = true;
    Matrix33 m_A_IK ; ///< The grid can be rotated, this is the transformation matrix from grid frame K to intertia frame I


    TreeType * m_kdTree;
    const NodeType * m_leaf; ///< This ranks leaf pointer
    const NodeType * m_lcaBoundary;

    ColliderKdTree<TreeType> m_colliderKdTree;

};


}; //MPILayer

#endif // MPITopologyKdTree_hpp
