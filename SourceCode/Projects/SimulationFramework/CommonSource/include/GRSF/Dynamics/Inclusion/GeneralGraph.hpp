#ifndef GRSF_Dynamics_Inclusion_GeneralGraph_hpp
#define GRSF_Dynamics_Inclusion_GeneralGraph_hpp


#include <cmath>
#include <assert.h>
#include <list>
#include <vector>
#include <map>
#include <string>
#include <memory>
#include <utility>
#include <algorithm>
#include <random>

namespace Graph {

// prototypes
template <typename NodeDataType, typename EdgeDataType> class Node;
template <typename NodeDataType, typename EdgeDataType> class Edge;
template <typename NodeDataType, typename EdgeDataType> class GeneralGraph;


/* Node Structures */
// We dont need to derive from this class! We do it with a templated acceptNode
//template <typename NodeDataType, typename EdgeDataType>
//class NodeVisitor {
//public:
//	virtual void visitNode(Node<NodeDataType, EdgeDataType>&) = 0;
//};


template <typename NodeDataType, typename EdgeDataType>
class Node {

public:
    // edge list
    std::vector<Edge<NodeDataType, EdgeDataType> *> m_edgeList;

    friend class GeneralGraph<NodeDataType, EdgeDataType>;
public:

    Node() : m_nodeNumber(-1) {};
    Node(unsigned int vN) : m_nodeNumber(vN) {};

    // node data
    const unsigned int m_nodeNumber;
    NodeDataType m_nodeData;

    // visitor dispatch
    //	template<typename TVisitor>
    //	void acceptVisitor(TVisitor & vv) {
    //		vv.visitNode(*this);
    //	}
};



/* Edge Structures */
// We dont need to derive from this class! We do it with a templated acceptNode
//template <typename NodeDataType, typename EdgeDataType>
//class EdgeVisitor {
//public:
//	virtual void visitEdge(Edge<NodeDataType, EdgeDataType>&) = 0;
//};

template <typename NodeDataType, typename EdgeDataType>
class Edge {
public:


    // graph structure
    Node<NodeDataType, EdgeDataType> *m_startNode;
    Node<NodeDataType, EdgeDataType> *m_endNode;
    Edge<NodeDataType, EdgeDataType> *m_twinEdge;

    // friends
    friend class Graph::GeneralGraph<NodeDataType, EdgeDataType>;
    friend class Graph::Node<NodeDataType, EdgeDataType>;
public:
    Edge() : m_startNode(nullptr), m_endNode(nullptr), m_twinEdge(nullptr), m_edgeNumber(0) {};
    Edge(unsigned int eN) : m_startNode(nullptr), m_endNode(nullptr), m_twinEdge(nullptr), m_edgeNumber(eN) {};

    /* Node<NodeDataType, EdgeDataType> * getStartNode(){return m_startNode;}
     Node<NodeDataType, EdgeDataType> * getEndNode(){return m_endNode;}
     Edge<NodeDataType, EdgeDataType> * getTwinEdge(){return m_twinEdge;}

     void setNodes( Node<NodeDataType, EdgeDataType> * sN, Node<NodeDataType, EdgeDataType> * eN, Edge<NodeDataType, EdgeDataType> * tE){
        m_startNode = sN;
        m_endNode = eN;
        m_twinEdge = tE;
     }
    */

    const unsigned int m_edgeNumber;
    EdgeDataType m_edgeData;

    // visitor dispatch
    //	template<typename TVisitor>
    //	void acceptVisitor(TVisitor & ev) {
    //		ev.visitEdge(*this);
    //	}
};



/** GeneralGraph */
template <typename NodeDataType, typename EdgeDataType>
class GeneralGraph {

protected:
    std::vector<Node<NodeDataType, EdgeDataType> *> m_nodes;
    std::vector<Edge<NodeDataType, EdgeDataType> *> m_edges;

public:
    using NodeType = Node<NodeDataType, EdgeDataType>;
    using EdgeType = Edge<NodeDataType, EdgeDataType>;

    typedef std::vector<NodeType* > NodeListType;
    typedef std::vector<EdgeType* > EdgeListType;
    typedef typename std::vector<NodeType* >::iterator NodeListIteratorType;
    typedef typename std::vector<EdgeType* >::iterator EdgeListIteratorType;


    GeneralGraph() {};
    ~GeneralGraph() {};

    void shrinkToFit(){
        m_nodes.shrink_to_fit();
        m_edges.shrink_to_fit();
    }

    void clearGraph() {

        for(auto & p : m_nodes){
            delete p;
        }
        m_nodes.clear();

        for(auto & p : m_edges){
            delete p;
        }
        m_edges.clear();


    }

    void reserveNodes(unsigned int n){
        m_nodes.reserve(n);
    }

    void reserveEdges(unsigned int n){
        m_edges.reserve(n);
    }

    template<typename... T>
    NodeType * addNode( T &&... t) {
        NodeType * p = new NodeType(std::forward<T>(t)...);
        m_nodes.emplace_back(p);
        return p;
    }
    template<typename... T>
    EdgeType * addEdge( T &&... t) {
        EdgeType * p = new EdgeType(std::forward<T>(t)...);
        m_edges.emplace_back(p);
        return p;
    }

    unsigned int getNumNodes() {
        return m_nodes.size();
    };
    unsigned int getNumEdges() {
        return m_edges.size();
    };


    NodeListType & getNodeList() {
        return m_nodes;
    };
    EdgeListType & getEdgeList() {
        return m_edges;
    };

    template<typename TNodeVisitor>
    void applyNodeVisitor(TNodeVisitor & vv) {
        for(auto & curr_node : m_nodes)
            vv.visitNode(*curr_node);
    }

    template<typename TEdgeVisitor>
    void applyEdgeVisitor(TEdgeVisitor & hev) {
        for(auto & curr_he : m_edges)
            hev.visitEdge(*curr_he);
    }

    void shuffleNodesUniformly(const unsigned int loops) {
        static std::default_random_engine g;
        std::uniform_int_distribution<unsigned int> r(0,m_nodes.size()-1);
        static NodeType * p1;

        for(auto it = m_nodes.begin(); it != m_nodes.end(); ++it ) {
            //swap the pointers
            p1 = *it;
            NodeType* & p2 = m_nodes[r(g)];
            *it = p2;
            p2 = p1;
        }
    }
};

}; // Namespace Graph


#endif

