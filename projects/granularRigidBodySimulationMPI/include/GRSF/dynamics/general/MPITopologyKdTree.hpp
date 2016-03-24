#ifndef GRSF_dynamics_general_MPITopologyKdTree_hpp
#define GRSF_dynamics_general_MPITopologyKdTree_hpp

#include "GRSF/common/AssertionDebug.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include <boost/variant.hpp>
#include <vector>
#include <map>

#include "GRSF/common/UnorderedContainerHelpers.hpp"

#include "GRSF/dynamics/general/KdTree.hpp"
#include "GRSF/dynamics/collision/geometry/AABB.hpp"
#include "GRSF/dynamics/collision/Collider.hpp"

#include "GRSF/dynamics/general/MPITopologyVisitors.hpp"

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
    using BoundaryInfoType = typename NodeType::BoundaryInfoType;
    using LeafNeighbourMapType = typename TreeType::LeafNeighbourMapType ;


    ProcessTopologyKdTree(  NeighbourRanksListType & nbRanks, AdjacentNeighbourRanksMapType & adjNbRanks,
                            RankIdType processRank, RankIdType masterRank,
                            std::shared_ptr<TreeType> tree,
                            const LeafNeighbourMapType & neighbours,
                            const AABB3d & aabb,
                            bool aligned,
                            const Matrix33 & A_IK = Matrix33::Identity())
            : m_rank(processRank), m_axisAligned(aligned), m_A_KI(A_IK.transpose()), m_kdTree(tree.get()), m_kdTreeRefCount(tree)
    {

        if(!m_kdTree){
            ERRORMSG("KdTree is nullptr")
        }

        // insert our neighbours
        auto it = neighbours.find(processRank);
        if(it == neighbours.end()){
            ERRORMSG("neighbours does not contain an entry for rank: " << processRank );
        }
        nbRanks.insert(it->second.begin(),it->second.end());

        // get adj neighbours
        for( auto & rank :  nbRanks) {
            std::cerr << "ProcessTopologyKdTree: NbRank: " << rank << std::endl;
            //Initialize adjacent neighbour ranks to m_nbRanks for this neighbours[*it]$
            auto itN = neighbours.find(rank);
             if(itN == neighbours.end()){
                ERRORMSG("neighbours does not contain an entry for rank: " << rank );
            }
            // build the intersection of our ranks with neighbours of other rank
            adjNbRanks.emplace(rank, unorderedHelpers::makeIntersection(nbRanks, itN->second) );
            for(auto & v : adjNbRanks[rank]){
                std::cerr << " adjNB: " << v << std::endl;
            }
        }

        // get our aabb

        m_leaf = m_kdTree->getLeaf(m_rank);
        if(!m_leaf){
            ERRORMSG("Trying to get leaf node for our rank: " << m_rank << " failed!")
        }
        std::cerr << " got leaf " << m_leaf->getIdx() << std::endl;

        // adjust kdTree
        // TODO (unlink childs which corresponds to subtrees we never need to check because the leafs subtree are not neighbours)
        // not essential for correct algorithm
        // but we leave the leafs which are not part of our neighbours as is, such that error messages contain these idx values
        // if a body overlaps into a neighbour which is not part of ours

        // the kdTree is already filling the whole space
        // no need to expand!


        // get lowest common ancestors of all boundary subtrees, including our own leaf node
        // the lca node is the node we start from for collision detection
        auto bndIt = m_leaf->getBoundaries().begin();
        auto root = m_kdTree->getRootNode();
        ASSERTMSG(root,"Root nullptr!")

        m_lcaBoundary = m_leaf;
        while( bndIt != m_leaf->getBoundaries().end() && m_lcaBoundary != root){
            if( *bndIt != nullptr ){
                m_lcaBoundary = m_kdTree->getLowestCommonAncestor(m_lcaBoundary, *bndIt);
            }
            ++bndIt;
        }
        // if m_leaf = root node or , the lca becomes nullptr which is not good,
        // we set it to the root node, this means we only have one leaf = 1 process = root node
        if(m_lcaBoundary == nullptr){
           m_lcaBoundary = root;
        }
        // collider should not add m_leaf in the results list over the overlap check
        m_colliderKdTree.setNonAddedLeaf(m_leaf);
        std::cerr << " finished " << std::endl;
    };

    ~ProcessTopologyKdTree(){}

    RankIdType getRank() const{return m_rank;}

    RankIdType getCellRank(const Vector3 & I_point) const {
        if(m_axisAligned) {
            auto * leaf = m_kdTree->getLeaf(I_point);
            ASSERTMSG(leaf,"This should never be nullptr!");
            return leaf->getIdx();
        }else{
            auto * leaf = m_kdTree->getLeaf(m_A_KI*I_point);
            ASSERTMSG(leaf,"This should never be nullptr!");
            return leaf->getIdx();
        }
    };


    bool checkOverlap(const RigidBodyType * body,
                      NeighbourRanksListType & neighbourProcessRanks,
                      bool & overlapsOwnRank) const {
        if(m_axisAligned) {
            overlapsOwnRank = m_colliderKdTree.checkOverlap(neighbourProcessRanks, m_lcaBoundary, body);
        } else {
            overlapsOwnRank = m_colliderKdTree.checkOverlap(neighbourProcessRanks, m_lcaBoundary, body, m_A_KI);
        }

        return neighbourProcessRanks.size() > 0;
    }



private:

    /**
    * Gets the common cells between all cellNumbers and the neighbours of cell number cellNumber2
    */

    template<typename SetType>
    inline typename AdjacentNeighbourRanksMapType::mapped_type
    getCommonNeighbourCells(const NeighbourRanksListType & neighboursOurRanks,
                            const SetType & neighboursOtherRank) const {

        typename AdjacentNeighbourRanksMapType::mapped_type intersec;
        // intersect nbRanks with cellNumbers

        // SetType needs to be sorted!

        std::set_intersection(neighboursOurRanks.begin(), neighboursOurRanks.end(),
                              neighboursOtherRank.begin(),neighboursOtherRank.end(),
                std::inserter(intersec,intersec.begin()));

        return intersec;
    };

    template<typename T> friend class TopologyVisitors::BelongsPointToProcess;
    template<typename T> friend class TopologyVisitors::CheckOverlap;

    RankIdType m_rank; ///< Own rank;

    bool m_axisAligned = true;
    Matrix33 m_A_KI ; ///< The grid can be rotated, this is the transformation matrix from grid frame K to intertia frame I


    TreeType * m_kdTree = nullptr;      ///< The kdTree
    const NodeType * m_leaf = nullptr; ///< This ranks leaf pointer
    const NodeType * m_lcaBoundary = nullptr;

    std::shared_ptr<TreeType> m_kdTreeRefCount; ///< only a stupid temporary to correctly hold the resource!


    ColliderKdTree<TreeType> m_colliderKdTree;

};


}; //MPILayer

#endif // MPITopologyKdTree_hpp
