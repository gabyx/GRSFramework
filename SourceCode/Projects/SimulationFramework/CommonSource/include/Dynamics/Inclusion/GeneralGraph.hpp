#ifndef GeneralGraph_hpp
#define GeneralGraph_hpp


#include <cmath>
#include <assert.h>
#include <list>
#include <vector>
#include <map>
#include <string>
#include <memory>
#include <utility>

#include <boost/shared_ptr.hpp>

namespace Graph{

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
	template<typename TVisitor>
	void acceptVisitor(TVisitor & vv) {
		vv.visitNode(*this);
	}
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
    Edge() : m_startNode(NULL), m_endNode(NULL), m_twinEdge(NULL), m_edgeNumber(0) {};
	Edge(unsigned int eN) : m_startNode(NULL), m_endNode(NULL), m_twinEdge(NULL), m_edgeNumber(eN){};

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
	template<typename TVisitor>
	void acceptVisitor(TVisitor & ev) {
		ev.visitEdge(*this);
	}
};



/** GeneralGraph */
template <typename NodeDataType, typename EdgeDataType>
class GeneralGraph{

protected:
	std::vector<Node<NodeDataType, EdgeDataType> *> m_nodes;
	std::vector<Edge<NodeDataType, EdgeDataType> *> m_edges;

public:

   typedef std::vector<Node<NodeDataType, EdgeDataType>* > NodeListType;
   typedef std::vector<Edge<NodeDataType, EdgeDataType>* > EdgeListType;
   typedef typename std::vector<Node<NodeDataType, EdgeDataType>* >::iterator NodeListIteratorType;
   typedef typename std::vector<Edge<NodeDataType, EdgeDataType>* >::iterator EdgeListIteratorType;


   GeneralGraph(){};

   ~GeneralGraph() {
		// cleanup allocated memory
		for(auto n_it = m_nodes.begin(); n_it != m_nodes.end(); n_it++)
			delete (*n_it);
		for(auto e_it = m_edges.begin(); e_it != m_edges.end(); e_it++)
			delete (*e_it);
	}

   unsigned int getNumNodes(){return m_nodes.size();};
   unsigned int getNumEdges(){return m_edges.size();};


   NodeListType & getNodeListRef(){ return m_nodes; };
   EdgeListType & getEdgeListRef(){ return m_edges; };

    template<typename TNodeVisitor>
	void applyNodeVisitor(TNodeVisitor & vv){
		for(auto curr_node = m_nodes.begin(); curr_node != m_nodes.end(); curr_node++)
			(*(*curr_node)).template acceptVisitor<TNodeVisitor>(vv);
	}

	template<typename TEdgeVisitor>
	void applyEdgeVisitor(TEdgeVisitor & hev){
		for(auto curr_he = m_edges.begin(); curr_he != m_edges.end(); curr_he++)
			(*(*curr_he)).template acceptVisitor<TEdgeVisitor>(hev);
	}
};

}; // Namespace Graph


#endif

