#ifndef GRSF_Dynamics_General_SerializationHelpersKdTree_hpp
#define GRSF_Dynamics_General_SerializationHelpersKdTree_hpp

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/traits.hpp>
#include <boost/serialization/split_member.hpp>

#include "GRSF/Dynamics/General/KdTree.hpp"
#include "GRSF/Dynamics/Collision/SerializationHelpersGeometries.hpp"
#include "GRSF/Common/SerializationHelpersTuple.hpp"

/** Basic KDTree classes for serialization */
namespace boost{
    namespace serialization{

        template<typename Archive>
        void serialize(Archive & ar, KdTree::TreeStatistics & s, const unsigned int version){

            ar & s.m_computedTreeStats;
            ar & s.m_treeDepth;
            ar & s.m_avgSplitPercentage;
            ar & s.m_minLeafExtent;
            ar & s.m_maxLeafExtent;
            ar & s.m_avgLeafSize;
            ar & s.m_minLeafDataSize;
            ar & s.m_maxLeafDataSize;
            ar & s.m_computedNeighbourStats;
            ar & s.m_minNeighbours;
            ar & s.m_maxNeighbours;
            ar & s.m_avgNeighbours;

        }
    };
};

/** Serialization of a KdTree,
 *  Currently only KdTree::TreeSimpleS in combination with KdTree::NodeSimpleS
 *  is made for serialization since , private members are all accesible.
 *  Wrapping into this class, and sending this to an archive, serializes the tree!
 */
template<typename Tree>
class KdTreeSerializer: public boost::serialization::traits< KdTreeSerializer<Tree>,
                                    boost::serialization::object_serializable,
                                    boost::serialization::track_never
                        >
{
    public:

    KdTreeSerializer(Tree & t): m_t(t){}


    template<class Archive>
    void save(Archive & ar, unsigned int version) const
    {
        saveTree(ar,m_t);
    }

    template<class Archive>
    void load(Archive & ar, unsigned int version) const
    {
        loadTree(ar,m_t);
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER()

    private:

        using IndexType = std::size_t;

        using LinkListType = std::vector< IndexType >;

        template<typename NodeType>
        struct NodeSerializer;

        template<typename Traits>
        struct NodeSerializer< KdTree::NodeSimpleS<Traits> >{

            std::vector<IndexType> m_boundaries; // only for loading!

            using NodeType = typename KdTree::NodeSimpleS<Traits>;
            using BoundaryInfoType = typename NodeType::BoundaryInfoType;

            NodeSerializer(){}
            NodeSerializer(std::size_t nNodes, std::size_t nLeafs){
                m_boundaries.reserve(nLeafs*(BoundaryInfoType::size+1));
            }

            template<typename Archive>
            void save(Archive & ar, KdTree::NodeSimpleS<Traits> & treeN) const
            {
                static IndexType s;
                ar & treeN.m_idx;
                ar & treeN.m_splitAxis;
                ar & treeN.m_splitPosition;
                ar & treeN.m_treeLevel;
                ar & treeN.m_aabb;

                // serialize only leaf boundaries!
                // TODO instead of sending, recalculate aabb and bounds after receiving

                bool isLeaf = treeN.isLeaf();
                ar & isLeaf;
                if(isLeaf){
                    for(auto * b : treeN.m_bound){
                        s = ((b)? (b->m_idx+1) : 0);
                        ar & s ;
                    }
                }
            }

            template<typename Archive>
            void load(Archive & ar, KdTree::NodeSimpleS<Traits> & treeN)
            {

                ar & treeN.m_idx;
                ar & treeN.m_splitAxis;
                ar & treeN.m_splitPosition;
                ar & treeN.m_treeLevel;
                ar & treeN.m_aabb;

                bool isLeaf;
                ar & isLeaf;
                if(isLeaf){
                    m_boundaries.emplace_back(treeN.m_idx);
                    IndexType s;
                    for(unsigned int i=0; i<BoundaryInfoType::size;++i){
                        ar & s;
                        m_boundaries.emplace_back(s);
                    }
                }

            }

            template<typename NodeList>
            void setup(NodeList & nodes){
                // move over all boundary information and set link to subtrees in the corresp. node
                IndexType nodeIdx, bIdx;
                auto s = m_boundaries.size();
                for(std::size_t i=0; i<s; i+=BoundaryInfoType::size+1 ){

                    nodeIdx = m_boundaries[i];
                    ASSERTMSG(nodeIdx < nodes.size(), "out of bound: " << nodeIdx)

                    for(std::size_t b=0; b<BoundaryInfoType::size; ++b){

                        // m_boundaries[ (i+1)+b ]  -> is boundary node index
                        bIdx =  m_boundaries[ (i+1)+b ];

                        if(bIdx!=0){
                            --bIdx;
                            ASSERTMSG(bIdx < nodes.size(), "out of bound: " << bIdx)
                            nodes[ nodeIdx ]->m_bound.at(b) = nodes[ bIdx ];
                        }else{
                            nodes[ nodeIdx ]->m_bound.at(b) = nullptr;
                        }

                    }
                }
            }

        };


        template<typename Archive, typename Traits>
        void saveTree(Archive & ar, KdTree::TreeSimpleS<Traits> & tree) const
        {

            using NodeType = typename KdTree::TreeSimpleS<Traits>::NodeType;
            //using BoundaryInfoType = typename NodeType::BoundaryInfoType;
            IndexType s;

            // saving
            if(!tree.m_root){
                ERRORMSG("Saving empty tree is non-sense!")
            }

            // make all links (always 3 IndexTypes represent : parent -> child left/right (index = 0, means no child!) )
            LinkListType links;
            links.reserve((tree.m_nodes.size()-1)/2); //(exact number of links; edges = nodes.size()-1, divide by 2 )

            // save root idx (not needed it is by definition 0)
            // ar & tree.m_root->getIdx();

            s = tree.m_nodes.size();
            ar & s;
            s = tree.m_leafs.size();
            ar & s;

            NodeSerializer<NodeType> nodeSer;

            // serialize all nodes
            //std::cout << "SERIALIZE NODES" << std::endl;
            for(auto * n : tree.m_nodes){

                //std::cout << n->getIdx() << std::endl;
                nodeSer.save(ar ,*n);

                if(n->isLeaf()){
                    links.emplace_back(0);
                }else{
                    links.emplace_back( n->m_idx+1);
                    links.emplace_back( n->m_child[0]->m_idx);
                    links.emplace_back( n->m_child[1]->m_idx);
                }
            }

            // save root idx
            ar & tree.m_root->m_idx;

            //std::cout << "SERIALIZE NODES" << std::endl;

            // save links
            for(IndexType & l : links){
                ar <<  l;
            }

            // save statistics
            ar & tree.m_statistics;

        }

        template<typename Archive, typename Traits>
        void loadTree(Archive & ar, KdTree::TreeSimpleS<Traits> & tree) const
        {

            using NodeType = typename KdTree::TreeSimpleS<Traits>::NodeType;
            //using BoundaryInfoType = typename NodeType::BoundaryInfoType;

            ASSERTMSG(tree.m_root==nullptr, "Root node needs to be nullptr for loading!")


            IndexType nodes,leafs,links, idx;

            // load all nodes
            ar & nodes; // load node count
            ar & leafs; // load leaf count

            // setup nodes
            tree.m_nodes.reserve(nodes);

            NodeSerializer<NodeType> nodeSer(nodes,leafs);

            NodeType * node;
            for(IndexType i=0; i<nodes;++i){

                node = new NodeType{};
                nodeSer.load(ar,*node);

                ASSERTMSG(node->getIdx()==tree.m_nodes.size(),"size not equal:" << node->getIdx() << "," << tree.m_nodes.size() )
                tree.m_nodes.emplace_back(node);

            }

            // get root node
            ar & idx;
            tree.m_root = tree.m_nodes[idx];

            // sort nodes according to index (such that access is really easy for setting the links below)
            // this is not needed, since the tree's m_nodes is already continously index ordered by definition
            // std::sort(m_nodes.begin(), m_nodes.end(), [](NodeType *a, NodeType *b){a->m_idx  <  b->m_idx} );

            // load all links and link nodes
            NodeType * c, *p;
            for(IndexType i=0; i<nodes;++i){

                // get parent
                ar & idx; // index = 0 means, no links
                if(idx!=0){

                    --idx;
                    ASSERTMSG(idx < tree.m_nodes.size(),"Wrong idx: " << idx)
                    p = tree.m_nodes[idx];

                    ASSERTMSG(!p->isLeaf(), "Should not be leaf: idx"  << p->getIdx() <<","<< idx);

                    // get left child (zero means no child)
                    ar & idx;
                    ASSERTMSG(idx < tree.m_nodes.size(),"Wrong idx: " << idx)
                    c = tree.m_nodes[idx];
                    p->m_child[0] = c;
                    p->m_child[0]->m_parent = p;

                    // get right child
                    ar & idx;
                    ASSERTMSG(idx < tree.m_nodes.size(),"Wrong idx: " << idx)
                    c = tree.m_nodes[idx];
                    p->m_child[1] = c;
                    p->m_child[1]->m_parent = p;
                }
            }

            // setup nodes with node list
            nodeSer.setup(tree.m_nodes);

            // add all leafs
            tree.m_leafs.clear();
            tree.m_leafs.reserve(leafs);
            for(auto * n : tree.m_nodes){
                if(n->isLeaf()){
                    tree.m_leafs.emplace_back(n);
                }
            }




            // load statistics
            ar & tree.m_statistics;
        }

    Tree & m_t;
};


#endif
