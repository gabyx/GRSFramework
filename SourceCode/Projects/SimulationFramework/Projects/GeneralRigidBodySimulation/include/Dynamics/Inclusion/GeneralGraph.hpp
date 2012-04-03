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

#include "TypeDefs.hpp"

#include <boost/shared_ptr.hpp>
using namespace std;

namespace Graph{

// prototypes
template <typename NodeDataType, typename EdgeDataType> class Node;
template <typename NodeDataType, typename EdgeDataType> class Edge;
template <typename NodeDataType, typename EdgeDataType> class GeneralGraph;


/* Node Structures */
template <typename NodeDataType, typename EdgeDataType>
class NodeVisitor {
public:
	virtual void visitNode(Node<NodeDataType, EdgeDataType>& v) = 0;
};


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
	void acceptVisitor(NodeVisitor<NodeDataType, EdgeDataType>& vv) {
		vv.visitNode(*this);
	}
};



/* Edge Structures */
template <typename NodeDataType, typename EdgeDataType>
class EdgeVisitor {
public:
	virtual void visitEdge(Edge<NodeDataType, EdgeDataType>& e) = 0;
};

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
   Edge() : m_startNode(NULL), m_endNode(NULL), m_twinEdge(NULL), edgeNumber(eN) {};
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
	void acceptVisitor(EdgeVisitor<NodeDataType, EdgeDataType>& ev) {
		ev.visitEdge(*this);
	}
};



/** GeneralGraph */
template <typename NodeDataType, typename EdgeDataType>
class GeneralGraph{

protected:
	std::vector<Node<NodeDataType, EdgeDataType>* > m_nodes;
	std::vector<Edge<NodeDataType, EdgeDataType> *> m_edges;

public:
	
   typedef typename std::vector<Node<NodeDataType, EdgeDataType>* > NodeList;
   typedef typename std::vector<Edge<NodeDataType, EdgeDataType>* > EdgeList;
   typedef typename std::vector<Node<NodeDataType, EdgeDataType>* >::iterator NodeListIterator;
   typedef typename std::vector<Edge<NodeDataType, EdgeDataType>* >::iterator EdgeListIterator;


   GeneralGraph(){};
	
   ~GeneralGraph() {
		// cleanup allocated memory
		for(typename std::vector<Node<NodeDataType, EdgeDataType>* >::iterator n_it = m_nodes.begin(); n_it != m_nodes.end(); n_it++)
			delete (*n_it);
		for(typename std::vector<Edge<NodeDataType, EdgeDataType>* >::iterator e_it = m_edges.begin(); e_it != m_edges.end(); e_it++)
			delete (*e_it);
	}

   unsigned int getNumNodes(){return m_nodes.size();};
	unsigned int getNumEdges(){return m_edges.size();};


   NodeList & getNodeListRef(){ return m_nodes; };
   EdgeList & getEdgeListRef(){ return m_edges; };
	
	void visitNodes(NodeVisitor<NodeDataType, EdgeDataType>& vv) {
		for(typename vector<Node<NodeDataType, FaceDataType, EdgeDataType>* >::iterator curr_node = vertices.begin(); curr_node != vertices.end(); curr_node++)
			(*curr_node)->acceptVisitor(vv);
	}
	void visitEdges(EdgeVisitor<NodeDataType, EdgeDataType>& hev) {
		for(typename vector<Edge<NodeDataType, EdgeDataType>* >::iterator curr_he = edges.begin(); curr_he != edges.end(); curr_he++)
			(*curr_he)->acceptVisitor(hev);
	}
};

}; // Namespace Graph


#endif

