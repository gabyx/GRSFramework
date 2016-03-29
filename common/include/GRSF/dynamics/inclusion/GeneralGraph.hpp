// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_inclusion_GeneralGraph_hpp
#define GRSF_dynamics_inclusion_GeneralGraph_hpp



#include <boost/preprocessor/iteration/local.hpp>
#include <boost/preprocessor/cat.hpp>

#include "GRSF/common/StaticAssert.hpp"
#include "GRSF/common/DemangleTypes.hpp"
#include "GRSF/common/MetaHelper.hpp"
#include "GRSF/common/TupleHelper.hpp"
#include "GRSF/common/LinearReusableStorage.hpp"

namespace Graph {


template<typename TEdgeData, typename TTraits>
class Edge;

template<typename TEdgeData, typename TTraits>
class Node;

template<typename TTraits>
class NodeBase;
template<typename TTraits>
class EdgeBase;

/** Traits to construct a graph which has N unique node types (NodeDataTypes) and E edge types (EdgeDataTypes)
*   such that ((N+1)*N)/2 = E, which means the upper triangular part of the node to edge type correlation table is fully filled
*
*    nodeDataTypes = meta::list<A, B, C>;
*    edgeDataTypes = meta::list<E_aa, E_ab, E_ac, E_bb, E_bc, E_cc>;
*
*      +------+------+------+
*      |  A   |  B   |  C   |
*    +----------------------+
*    | |      |      |      |
*    |A| E_aa | E_ab | E_ac |
*    | | 1--- | 2--- | 3--- |
*    +----------------------+
*    | |      |      |      |
*    |B| E_ab | E_bb | E_bc |
*    | |      | 4--- | 5--- |
*    +----------------------+
*    | |      |      |      |
*    |C| E_ac | E_bc | E_cc |
*    | |      |      | 6--- |
*    +-+------+------+------+
*
*   The edge data types can also contain void where you would not like to have any storage associated with it
*   Setting certain entries to void results in no storage support for this kind of type in the corresponding node
*/
template<typename TNodeData, typename TEdgeData>
struct GraphTraitsSymmetric {

    using NodeDataTypes = meta::unique<TNodeData>;
    using EdgeDataTypesOrig = TEdgeData;
    using EdgeDataTypes = metaAdd::remove<void,TEdgeData>;

    static const int nNodes = NodeDataTypes::size();
    static const int nEdgesOrig = EdgeDataTypesOrig::size();
    static const int nEdges = EdgeDataTypes::size();

    static_assert( (nNodes*(nNodes+1))/2 == nEdgesOrig ,"Number of edges e not equal to nodes n, ((n+1)*n)/2 = e!");

    template<std::size_t R, std::size_t C, std::size_t N>
    using toLinearIdx_c = metaAdd::toLinearIdxSym_c<R,C,N>;
    template<typename R, typename C, typename N>
    using toLinearIdx = metaAdd::toLinearIdxSym<R,C,N>;

    template<typename TTNodeData>
    using getNodeDataIdx = meta::_t<meta::find_index<NodeDataTypes,TTNodeData> >;


    /** build map , node data to all output edge data types (without void) */
    template<typename NodeData>
    using outEdgesForNode =
        metaAdd::remove<void,
            meta::transform<
                meta::as_list<meta::make_index_sequence< nNodes > >,
                    meta::compose<
                        meta::bind_front<meta::quote<meta::at>,  EdgeDataTypesOrig>,
                        meta::bind_back<meta::bind_front<meta::quote<toLinearIdx>, getNodeDataIdx<NodeData> >, meta::size_t<nNodes>
                    >
                >
            >
        >;

    /** build map , node data to all input edge data types (without void) */
    template<typename NodeData>
    using inEdgesForNode =
        metaAdd::remove<void,
            meta::transform<
                meta::as_list<meta::make_index_sequence< nNodes > >,
                meta::compose<
                    meta::bind_front<meta::quote<meta::at>,  EdgeDataTypesOrig>,
                    meta::bind_back<meta::quote<toLinearIdx>,  getNodeDataIdx<NodeData> , meta::size_t<nNodes> >
                >
            >
        >;


    template<typename T>
    using EdgeStorageType = LinearReusableStorage<T>;
    template<typename T>
    using NodeStorageType = LinearReusableStorage<T>;

    template<typename TTEdgeData>
    using toEdge = Edge<TTEdgeData,GraphTraitsSymmetric>;

    template<typename TTNodeData>
    using toNode = Node<TTNodeData,GraphTraitsSymmetric>;

    using EdgeTypes = meta::transform< EdgeDataTypes, meta::quote<toEdge> >;
    using NodeTypes = meta::transform< NodeDataTypes, meta::quote<toNode> >;


    /** Make a meta::list< Container< F<Item1> > , Container< F<Item2> >, ... >  */
    template<typename List, typename F, typename Container = meta::quote<std::vector> >
    using makeTupleContainer = meta::apply<
                                   meta::quote<std::tuple>,
                                       meta::transform<
                                       List,
                                       meta::compose<
                                           Container,
                                           F
                                       >
                                   >
                               >;


    struct details {

        template<unsigned int N, typename Types>
        struct castAndVisit;

        // macro for boost repeat (z= repetition dimension?? = unused,  N = counter, data = unused )
#define SWITCH_CASE( z, N, data ) \
            case N: v( *static_cast< meta::at<Types,meta::size_t<N> > * >(p)); break;

#define GENERATE_SWITCH(N) \
            template<typename Types> \
            struct castAndVisit<N,Types> { \
                template<typename T, typename Visitor> \
                static inline void apply(const std::size_t & id, T * p, Visitor && v ){ \
                    switch(id){\
                        BOOST_PP_REPEAT(N,SWITCH_CASE,~) \
                        default: ERRORMSG("Tried to cast type: " << demangle::type<T>() << " to wrong id: " << id ); break; \
                    }\
                }\
            }; \

#define BOOST_PP_LOCAL_MACRO(N) GENERATE_SWITCH(N)

        // Macro Loop: do the job form 0 to 10
        // needed whitespace here----*
#define BOOST_PP_LOCAL_LIMITS (1,10)


        // execute the switch macro loop
#include BOOST_PP_LOCAL_ITERATE()

        // undef all macro madness
#undef BOOST_PP_LOCAL_LIMITS
#undef BOOST_PP_LOCAL_MACRO
#undef GENERATE_SWITCH
#undef SWITCH_CASE
    };

    /** Cast and visitor switch, from base class to the corresponding type */
    template<typename Visitor>
    inline static void castAndVisitNodes(const std::size_t &id, NodeBase<GraphTraitsSymmetric> * p, Visitor && v ) {
        static const auto N = NodeTypes::size();
        details::template castAndVisit< N, NodeTypes >::template apply(id, p, std::forward<Visitor>(v));
    }

    /** Cast and visitor switch, from base class to the corresponding type */
    template<typename Visitor>
    inline static void castAndVisitEdges(const std::size_t &id, EdgeBase<GraphTraitsSymmetric> * p, Visitor && v ) {
        static const auto N = EdgeTypes::size();
        details::template castAndVisit< N , EdgeTypes >::template apply(id, p, std::forward<Visitor>(v));
    }

};



template<typename TTraits>
class NodeBase {
public:
    using Traits = TTraits;
    NodeBase(const std::size_t & t): m_type(t) {}

    template<typename Visitor>
    void visit(Visitor && v) { // Universal reference (binds to temporaries and lvalues)
        // cast to the specific type and visit ( like boost::variant type switch )
        Traits::castAndVisitNodes(m_type,this,std::forward<Visitor>(v));
    }

private:
    const std::size_t m_type;
};

template<typename TTraits>
class EdgeBase {
public:
    using Traits = TTraits;

    EdgeBase(const std::size_t & t): m_type(t) {}

    template<typename Visitor>
    void visit(Visitor && v) { // Universal reference (binds to temporaries and lvalues)
        // cast to the specific type and visit ( like boost::variant type switch )
        Traits::castAndVisitNodes(m_type,this,std::forward<Visitor>(v));
    }

private:
    const std::size_t m_type;
};



template<typename TEdgeData, typename TTraits  >
class Edge: public EdgeBase<TTraits> {
public:
    using Traits = TTraits;

    using StartNodeType = NodeBase<Traits>;
    using EndNodeType   = NodeBase<Traits>;
    using TwinEdgeType  = Edge;

    using EdgeDataType  = TEdgeData;

    template<typename... T>
    Edge(const std::size_t & i, T &&... t)
        : EdgeBase<Traits>( meta::find_index<typename Traits::EdgeDataTypes, EdgeDataType >::value )
        ,  m_data(std::forward<T>(t)...), m_id(i)
    {}

    inline std::size_t getId(){ return m_id;}

    inline EdgeDataType & getData() {
        return m_data;
    }

    template<typename Visitor>
    void visit(Visitor && v) {
        v(*this);
    }

    void clear() {
        m_startNode = nullptr;
        m_endNode   = nullptr;
        m_twinEdge  = nullptr;
    }

    inline void setStartNode(StartNodeType * p) {
        m_startNode = p;
    }
    inline void setEndNode(EndNodeType * p) {
        m_endNode = p;
    }
    inline void setTwinEdge(TwinEdgeType * p) {
        m_twinEdge = p;
    }

    inline StartNodeType* getStartNode() {
        return m_startNode;
    }
    inline EndNodeType*   getEndNode() {
        return m_endNode;
    }
    inline TwinEdgeType*  getTwinEdge() {
        return m_twinEdge;
    }

private:

    StartNodeType   * m_startNode = nullptr;
    EndNodeType     * m_endNode   = nullptr;
    TwinEdgeType        * m_twinEdge  = nullptr;

    EdgeDataType m_data;
    const std::size_t m_id;
};

template< typename TNodeData, typename TTraits>
class NodeEdgeSupport {
public:
    using NodeDataType = TNodeData;
    using Traits = TTraits;

    using OutEdgeDataTypes = meta::unique< typename Traits::template outEdgesForNode<NodeDataType> >;
    using OutEdgeStoragesTypes = typename Traits::template makeTupleContainer< OutEdgeDataTypes,
            meta::compose<
                meta::quote<metaAdd::addPointer>,
                meta::quote<Traits::template toEdge>
                >
            >;

    using InEdgeDataTypes  = meta::unique< typename Traits::template inEdgesForNode<NodeDataType> >;
    using InEdgeStoragesTypes  = typename Traits::template makeTupleContainer< InEdgeDataTypes,
            meta::compose<
                meta::quote<metaAdd::addPointer>,
                meta::quote<Traits::template toEdge>
                >
            >;
public:
    inline void clearConnections() {
        ClearConnections c;
        TupleVisit::visit(c,m_edgesIn);
        TupleVisit::visit(c,m_edgesOut);
    }

    template<typename T>
    void addEdgeIn(T * e) {
        using Idx = meta::find_index<InEdgeDataTypes, typename T::EdgeDataType>;
        std::get< Idx::type::value >(m_edgesIn).push_back(e);
    }

    template<typename T>
    void addEdgeOut(T * e) {
        using Idx = meta::find_index< OutEdgeDataTypes, typename T::EdgeDataType>;
        std::get< Idx::type::value >(m_edgesOut).push_back(e);
    }

    template<typename Visitor>
    void visitInEdges(Visitor && v) {
        EdgeDispatcher<Visitor> e( std::forward<Visitor>(v));
        TupleVisit::visit(e,m_edgesIn);
    }
    template<typename Visitor>
    void visitOutEdges(Visitor && v) {
        EdgeDispatcher<Visitor> e( std::forward<Visitor>(v) ) ;
        TupleVisit::visit(e,m_edgesOut);
    }

private:

    OutEdgeStoragesTypes m_edgesOut;
    InEdgeStoragesTypes  m_edgesIn;

    struct ClearConnections {
        template<typename TStorage>
        inline void operator()(TStorage & t) {
            t.clear();
        }
    };

    /** EdgeStorageVisitor: Visit all edges, moves over all storages and then loops through all edges calling the visitor */
    template<typename Visitor>
    struct EdgeDispatcher {
        EdgeDispatcher(Visitor && n): m_n(n) {}
        template<typename TStorage>
        inline void operator()(TStorage & t) {
            for(auto & e : t) {
                m_n(*e);
            }
        }
        Visitor & m_n;
    };

};

template<typename TNodeData, typename TTraits>
class Node: public NodeBase<TTraits> , public NodeEdgeSupport<TNodeData,TTraits> {
public:
    using NodeDataType = TNodeData;
    using Traits = TTraits;

    template<typename... T>
    Node(const std::size_t & i, T &&... t)
    : NodeBase<Traits>( meta::find_index<typename Traits::NodeDataTypes, NodeDataType >::value )
    , m_data(std::forward<T>(t)...), m_id(i)
    {}

    inline std::size_t getId(){ return m_id;}

    inline NodeDataType & getData() {
        return m_data;
    }

    inline void clear() {
        NodeEdgeSupport<TNodeData,TTraits>::clearConnections();
    }

private:
    NodeDataType m_data;
    const std::size_t  m_id;
};



template<typename GraphTraits>
class GeneralGraph {
public:

    using Traits = GraphTraits;
    using NodeDataTypes  = typename Traits::NodeDataTypes;
    using EdgeDataTypes  = typename Traits::EdgeDataTypes;

    template<typename TEdge>
    using EdgeStorageType = typename Traits::template EdgeStorageType<TEdge>;
    template<typename TNode>
    using NodeStorageType = typename Traits::template NodeStorageType<TNode>;


    using NodeStorageTuple = typename Traits::template makeTupleContainer< NodeDataTypes,
                                                                            meta::quote<Traits::template toNode>,
                                                                            meta::quote<NodeStorageType> >;

    using EdgeStorageTuple = typename Traits::template makeTupleContainer< meta::unique<EdgeDataTypes>,
                                                                            meta::quote<Traits::template toEdge>,
                                                                            meta::quote<EdgeStorageType> >;

    template<typename TNodeData>
    using toNodeStorageType  =  std::tuple_element_t< meta::find_index<NodeDataTypes,TNodeData>::value , NodeStorageTuple  >;
    template<typename TEdgeData>
    using toEdgeStorageType  =  std::tuple_element_t< meta::find_index<EdgeDataTypes,TEdgeData>::value , EdgeStorageTuple  >;

protected:

    NodeStorageTuple m_nodeStorage;
    EdgeStorageTuple m_edgeStorage;

    /** Node Dispatcher: Moves visitor over all storages, and applies the visitor to the storage */
    template<typename NodeVisitor>
    struct NodeDispatcher {

        NodeDispatcher(NodeVisitor && n): m_n(n) {}
        template<typename TStorage>
        inline void operator()(TStorage & t) {
            t.visit(m_n);
        }

        NodeVisitor && m_n;
    };

    /** Storage visitor: Delete all nodes */
    struct DeleteStorage {
        template<typename TStorage>
        void operator()(TStorage & t) {
            t.deleteAll();
        }
    };
    /** Storage visitor: Clear all storages */
    struct ClearStorages {
        template<typename TStorage>
        void operator()(TStorage & t) {
            t.clear();
        }
    };

    /** Get reference of the storage for a node type N */
    template<typename TNodeData>
    auto getNodeStorageRef() -> toNodeStorageType<TNodeData> &  {
        return std::get< meta::find_index< NodeDataTypes, TNodeData >::value >(m_nodeStorage); // get container
    }

    /** Get reference of the storage for a node type N */
    template<typename TEdgeData >
    auto getEdgeStorageRef() -> toEdgeStorageType<TEdgeData> &  {
        return std::get< meta::find_index< EdgeDataTypes, TEdgeData >::value >(m_edgeStorage); // get container
    }


public:

    ~GeneralGraph() {}

    void clear() {
        TupleVisit::visit(ClearStorages{},m_nodeStorage);
    }

    template<typename TNodeData>
    void reserveNodes(std::size_t nodes) {
        getNodeStorageRef<TNodeData>().reserve(nodes);
    }

    template<typename TEdgeData>
    void reserveEdges(std::size_t edges) {
        getEdgeStorageRef<TEdgeData>().reserve(edges);
    }

    void deleteNodes() {
        TupleVisit::visit(DeleteStorage{},m_nodeStorage);
    }

    void deleteEdges() {
        TupleVisit::visit(DeleteStorage{},m_edgeStorage);
    }

    void deleteAll() {
        deleteNodes();
        deleteEdges();
    }

    template<typename TNodeData>
    auto getNode(std::size_t i) -> decltype( getNodeStorageRef<TNodeData>().getNode(i) ) {
        return getNodeStorageRef<TNodeData>().getNode(i);
    }

    template<typename TNodeData, typename... T>
    typename Traits::template toNode<TNodeData> * emplaceNodeBack(T &&... t) {
        return getNodeStorageRef<TNodeData>().emplace_back(std::forward<T>(t)...);
    }

    template<typename TEdgeData, typename... T>
    typename Traits::template toEdge<TEdgeData> * emplaceEdgeBack(T &&... t) {
        return getEdgeStorageRef<TEdgeData>().emplace_back(std::forward<T>(t)...);
    }

    /** Visit all nodes */
    template<typename... NodeDataType, typename Visitor>
    void visitNodes(Visitor && v) {
        NodeDispatcher<Visitor> n( std::forward<Visitor>(v) );
        TupleVisit::visit< toNodeStorageType<NodeDataType>... >(n,m_nodeStorage);
    }

    template<typename TNodeData>
    void shuffleNodesUniformly() {
        auto nodeStorage = getNodeStorageRef<TNodeData>();
        nodeStorage.shuffleUniformly();
    }

};



}; // Namespace Graph


#endif

