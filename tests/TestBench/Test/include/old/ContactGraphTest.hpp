// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef ContactGraphTest_hpp
#define ContactGraphTest_hpp

#include "GRSF/common/CPUTimer.hpp"
#include "GRSF/common/EnumClassHelper.hpp"
#include "GRSF/common/StaticAssert.hpp"
#include "GRSF/common/TupleHelper.hpp"

#include <boost/mpl/assert.hpp>
#include <boost/mpl/at.hpp>
#include <boost/mpl/divides.hpp>
#include <boost/mpl/find.hpp>
#include <boost/mpl/fold.hpp>
#include <boost/mpl/insert.hpp>
#include <boost/mpl/modulus.hpp>
#include <boost/mpl/placeholders.hpp>
#include <boost/mpl/range_c.hpp>
#include <boost/mpl/same_as.hpp>
#include <boost/mpl/set.hpp>
#include <boost/mpl/size.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/type_traits/is_same.hpp>
#include <vector>

namespace mpl = boost::mpl;

enum class EdgeDataTypesEnum : int
{
    EDGE_AA,
    EDGE_AB,
    EDGE_BB,
};

class EdgeBase
{
public:
    EdgeBase(const EdgeDataTypesEnum& t) : m_type(t)
    {
    }

protected:
    EdgeDataTypesEnum m_type;
};

enum class NodeDataTypesEnum : int
{
    NODE_A,
    NODE_B
};

class NodeBase
{
public:
    NodeBase(const NodeDataTypesEnum& t) : m_type(t)
    {
    }

protected:
    NodeDataTypesEnum m_type;
};

template <typename TEdgeData,
          typename TNodeStart = NodeBase,
          typename TNodeEnd   = NodeBase,
          typename TEdgeTwin  = EdgeBase>
class Edge : public EdgeBase
{
public:
    using EdgeDataType = TEdgeData;

    template <typename... T>
    Edge(std::size_t i, T&&... t) : EdgeBase(EdgeDataType::enumType), m_counter(i), m_data(std::forward<T>(t)...)
    {
    }

    void setStart(TNodeStart* p)
    {
        m_startNode = p;
    }
    void setEnd(TNodeEnd* p)
    {
        m_endNode = p;
    }
    void setTwin(TEdgeTwin* p)
    {
        m_twinEdge = p;
    }

    TNodeStart* getStart()
    {
        return m_startNode;
    }
    TNodeEnd* getEnd()
    {
        return m_endNode;
    }
    TEdgeTwin* getTwin()
    {
        return m_twinEdge;
    }

    inline EdgeDataType& getData()
    {
        return m_data;
    }

private:
    EdgeDataType m_data;
    std::size_t  m_counter;

    NodeBase* m_startNode = nullptr;
    NodeBase* m_endNode   = nullptr;
    EdgeBase* m_twinEdge  = nullptr;
};

template <typename TNodeData>
class Node : public NodeBase
{
public:
    using NodeDataType = TNodeData;
    using EdgeList     = std::vector<EdgeBase*>;

    template <typename... T>
    Node(std::size_t i, T&&... t) : NodeBase(NodeDataType::enumType), m_counter(i), m_data(std::forward<T>(t)...)
    {
    }

    inline NodeDataType& getData()
    {
        return m_data;
    }
    inline EdgeList& getEdgeListRef()
    {
        return m_edges;
    }

private:
    NodeDataType m_data;
    std::size_t  m_counter;

    std::vector<EdgeBase*> m_edges;
};

/** Example EdgeData */
class EdgeDataAA
{
public:
    EdgeDataAA(int _a) : a(_a)
    {
    }
    static const EdgeDataTypesEnum enumType;
    int                            a;
};
const EdgeDataTypesEnum EdgeDataAA::enumType = EdgeDataTypesEnum::EDGE_AA;

class EdgeDataAB
{
public:
    EdgeDataAB(float _a) : a(_a)
    {
    }
    static const EdgeDataTypesEnum enumType;
    float                          a;
};
const EdgeDataTypesEnum EdgeDataAB::enumType = EdgeDataTypesEnum::EDGE_AB;

class EdgeDataBB
{
public:
    EdgeDataBB(double _a) : a(_a)
    {
    }
    static const EdgeDataTypesEnum enumType;
    double                         a;
};
const EdgeDataTypesEnum EdgeDataBB::enumType = EdgeDataTypesEnum::EDGE_BB;

/** Example NodeData */
class NodeDataA
{
public:
    NodeDataA(){};
    NodeDataA(int                  _a) : a(_a){};
    static const NodeDataTypesEnum enumType;
    int                            a;
};
const NodeDataTypesEnum NodeDataA::enumType = NodeDataTypesEnum::NODE_A;

class NodeDataB
{
public:
    NodeDataB(){};
    NodeDataB(double               _a) : a(_a){};
    static const NodeDataTypesEnum enumType;
    double                         a;
};
const NodeDataTypesEnum NodeDataB::enumType = NodeDataTypesEnum::NODE_B;

template <typename TNode>
class NodeStorage
{
public:
    using NodeType = TNode;

    ~NodeStorage()
    {
        deleteAll();
    };

    template <typename... T>
    std::pair<NodeType*, bool> insert(T&&... t)
    {
        NodeType* p;
        bool      inserted = false;

        if (m_insertIdx < m_storage.size())
        {
            p = m_storage[m_insertIdx];  // get pointer from storage
        }
        else
        {
            // allocate new node, and emplace
            p        = new NodeType{m_insertIdx, std::forward<T>(t)...};
            inserted = true;
            m_storage.emplace_back(p);
        }
        ++m_insertIdx;
        return std::make_pair(p, inserted);
    }

    void clear()
    {
        m_insertIdx = 0;
    }

    void deleteAll()
    {
        clear();
        for (auto& n : m_storage)
        {
            delete n;
        }
    }

    void reserve(std::size_t nodes)
    {
        std::size_t idx = m_insertIdx;
        std::size_t s   = m_storage.size();
        if (nodes > s)
        {
            for (std::size_t i = 0; i < (nodes - s); ++i)
            {
                insert();
            }
        }

        m_insertIdx = idx;
    }

    template <typename Visitor>
    void visit(Visitor& v)
    {
        std::size_t s = m_insertIdx;
        for (std::size_t i = 0; i < s; ++i)
        {
            v(*m_storage[i]);
        }
    }

    NodeType* getNode(std::size_t i)
    {
        return m_storage[i];
    }

private:
    std::vector<NodeType*> m_storage;
    // Take care, if you want to switch this to a value-type all returned pointer in insertNode are invalidate if
    // reallocation happens

    std::size_t m_insertIdx = 0;  ///< Insertion idx counter, to
};

template <typename TEdge>
using EdgeStorage = NodeStorage<TEdge>;

namespace CGTraitsDetails
{
namespace details
{
template <class T, class R>
struct ToStdTuple;

template <class... TTypes, class X>
struct ToStdTuple<std::tuple<TTypes...>, X>
{
    typedef std::tuple<TTypes..., X> type;
};
};

template <typename Result>
struct mplToTuple
{
    using type = typename mpl::fold<Result, std::tuple<>, details::ToStdTuple<mpl::_1, mpl::_2>>::type;
};

template <template <typename> class Wrap, typename T>
struct makeTupleWrap;

template <template <typename> class Wrap, typename... E>
struct makeTupleWrap<Wrap, std::tuple<E...>>
{
    using type = std::tuple<Wrap<E>...>;
};
};

template <typename NodeTypes, typename EdgeTypes>
struct ContactGraphTraits;

template <typename... TNodaData, typename... TEdgeData>
struct ContactGraphTraits<std::tuple<TNodaData...>, std::tuple<TEdgeData...>>
{
    using NodeDataTypes = boost::mpl::vector<TNodaData...>;
    using EdgeDataTypes = boost::mpl::vector<TEdgeData...>;

    /** Unique NodeType */
    using NodeTypes = boost::mpl::vector<Node<TNodaData>...>;

    /** Non-Unique EdgeTypes,  size N(N+1)/2 types for each relation between each node to node
    * (node,node) -> edge type map (only upper diagonal is saved)
    */
    using EdgeTypes = boost::mpl::vector<Edge<TEdgeData>...>;  //

    struct details
    {
        using unique_node_types =
            typename mpl::fold<ContactGraphTraits::NodeTypes,
                               mpl::set0<>,
                               mpl::insert<mpl::placeholders::_1,
                                           mpl::placeholders::_2> /** _1 = State = set<...>, _2 element of NodeTypes*/
                               >::type;                           /** mpl::set */

        static const int size_unique_node_types = mpl::size<unique_node_types>::value;

        using unique_edge_types =
            typename mpl::fold<EdgeTypes,
                               mpl::set0<>,
                               mpl::insert<mpl::placeholders::_1,
                                           mpl::placeholders::_2> /** _1 = State = set<...>, _2 element of NodeTypes*/
                               >::type;                           /** mpl::set */

        static const int size_unique_edge_types = mpl::size<EdgeTypes>::value;
    };

    GRSF_STATIC_ASSERTM(details::size_unique_edge_types ==
                                details::size_unique_node_types * (details::size_unique_node_types + 1) / 2 ||
                            details::size_unique_edge_types == 1,
                        "NUMBER OF SPECIFIED EDGE TYPES INCORRECT");
    /** Fails if EdgeTypes are not N(N+1)/2 types (N = unique_node_types) or 1 (which results !*/
    GRSF_STATIC_ASSERTM(details::size_unique_node_types == mpl::size<NodeTypes>::value,
                        "NON UNIQUE NODE TYPES SPECIFIED");
    /** Fails if NodeTypes are not unique!*/

    using NodeStorageTuple = std::tuple<NodeStorage<Node<TNodaData>>...>; /** no need to adjust, those are unique!*/

    /** mpl::vector */
    using UniqueEdgeTypes =
        typename mpl::fold<typename details::unique_edge_types,
                           mpl::vector<>,
                           mpl::push_front<mpl::placeholders::_1,
                                           mpl::placeholders::_2> /** _1 = State = set<...>, _2 element of NodeTypes*/
                           >::type;

    using UniqueEdgeStorageTuple =
        typename CGTraitsDetails::makeTupleWrap<EdgeStorage,
                                                typename CGTraitsDetails::mplToTuple<UniqueEdgeTypes>::type>::type;

    using EdgeStorageTuple = UniqueEdgeStorageTuple;

    /** Define the storage type for a node type N
    /* NodeType --> NodeStorageType
    */
    template <typename TNode>
    struct nodeStorageTypeByNodeType
    {
        using iter                    = typename mpl::find<NodeTypes, TNode>::type;
        static const unsigned int idx = iter::pos::value;
        GRSF_STATIC_ASSERT((std::is_same<TNode, typename mpl::deref<iter>::type>::value))
        using type = typename std::tuple_element<idx, NodeStorageTuple>::type;
    };

    /** Define the storage type for a node type N
    /* EdgeType --> EdgeStorageType
    */
    template <typename TEdge>
    struct edgeStorageTypeByEdgeType
    {
        using iter = typename mpl::find<UniqueEdgeTypes, TEdge>::type;
        GRSF_STATIC_ASSERT((std::is_same<TEdge, typename mpl::deref<iter>::type>::value))
        static const unsigned int idx = iter::pos::value;
        using type                    = typename std::tuple_element<idx, EdgeStorageTuple>::type;
    };

    /** NodeData --> NodeType */
    template <typename TTNodeData>
    struct nodeTypeByNodeData
    {
        struct same_as_nodedata
        {
            template <typename T>
            struct apply
            {
                using type = typename mpl::same_as<typename T::NodeDataType>::template apply<TTNodeData>::type;
            };
        };
        using iter = typename mpl::find_if<NodeTypes, same_as_nodedata>::type;
        GRSF_STATIC_ASSERT((!std::is_same<iter, typename mpl::end<NodeTypes>::type>::value))

        using type = typename mpl::deref<iter>::type;
    };

    /** EdgeData --> EdgeType */
    template <typename TTEdgeData>
    struct edgeTypeByEdgeData
    {
        struct same_as_edgedata
        {
            template <typename T>
            struct apply
            {
                using type = typename mpl::same_as<typename T::EdgeDataType>::template apply<TTEdgeData>::type;
            };
        };

        using iter = typename mpl::find_if<EdgeTypes, same_as_edgedata>::type;
        GRSF_STATIC_ASSERT((!std::is_same<iter, typename mpl::end<EdgeTypes>::type>::value))

        using type                    = typename mpl::deref<iter>::type;
        static const unsigned int idx = iter::pos::value;
    };

    /** (NodeType1, NodeType2) --> EdgeType */
    template <typename TNode1, typename TNode2>
    struct edgeTypeByNodeType
    {
        /** extract both indices from the list*/
        using iter1 = typename mpl::find<NodeTypes, TNode1>::type;
        GRSF_STATIC_ASSERT((!std::is_same<iter1, typename mpl::end<NodeTypes>::type>::value))  // not end iterator
        static const unsigned int i = iter1::pos::value;

        using iter2 = typename mpl::find<NodeTypes, TNode2>::type;
        GRSF_STATIC_ASSERT((!std::is_same<iter2, typename mpl::end<NodeTypes>::type>::value))  // not end iterator
        static const unsigned int j = iter2::pos::value;

        static const unsigned int linearIdx =
            (j >= i) ? i * (details::size_unique_edge_types - 1) + j : j * (details::size_unique_edge_types - 1) + i;

        using edgetype =
            typename mpl::at_c<EdgeTypes, linearIdx>::type;  // type in map -> (node i, node j) -> edge type

        // Final return values
        using iter                    = typename mpl::find<UniqueEdgeTypes, edgetype>::type;
        static const unsigned int idx = iter::pos::value;
        using type                    = typename mpl::deref<iter>::type;
    };

    /** (EdgeType) --> EdgeData */
    template <typename TEdge>
    struct edgeDataByEdgeType
    {
        using iter = typename mpl::find<UniqueEdgeTypes, TEdge>::type;

        GRSF_STATIC_ASSERT((!std::is_same<iter, typename mpl::end<UniqueEdgeTypes>::type>::value))

        template <typename T>
        struct extractData;

        template <typename D>
        struct extractData<Edge<D>>
        {
            using type = D;
        };
        static const unsigned int idx = iter::pos::value;
        using type                    = typename extractData<TEdge>::type;
    };

    /** (NodeData1,NodeData2) --> EdgeData */
    template <typename TNodeData1, typename TNodeData2>
    struct edgeDataByNodeData
    {
        using res =
            edgeDataByEdgeType<typename edgeTypeByNodeType<typename nodeTypeByNodeData<TNodeData1>::type,
                                                           typename nodeTypeByNodeData<TNodeData2>::type>::type>;

        using type                    = typename res::type;
        static const unsigned int idx = res::idx;
    };

    /** Experiment to inject the start end node into the edgetypes */
    template <unsigned int Idx>
    struct nodeTypesByIndex
    {
        static const unsigned int m =
            mpl::divides<mpl::int_<Idx>, mpl::int_<mpl::size<NodeTypes>::type::value - 1>>::type::value;

        static const unsigned int i =
            (mpl::size<NodeTypes>::type::value - 1 < m) ? mpl::size<NodeTypes>::type::value - 1 : m;

        static const unsigned int j = Idx - i * (mpl::size<NodeTypes>::type::value - 1);

        using typeA = typename mpl::at_c<NodeTypes, i>::type;
        using typeB = typename mpl::at_c<NodeTypes, j>::type;
    };

    template <unsigned int Idx>
    struct nodeTypeAByIndex
    {
        using type = typename nodeTypesByIndex<Idx>::typeA;
    };

    template <unsigned int Idx>
    struct nodeTypeBByIndex
    {
        using type = typename nodeTypesByIndex<Idx>::typeB;
    };

    struct makeEdge
    {
        template <typename T1, typename T2>
        struct apply
        {
            using type =
                Edge<T1, typename nodeTypeAByIndex<T2::value>::type, typename nodeTypeBByIndex<T2::value>::type>;
        };
    };

    using EdgeTypes2 = typename mpl::
        transform<EdgeDataTypes, mpl::range_c<int, 0, mpl::size<EdgeDataTypes>::type::value>, makeEdge>::type;

    using EdgeTypes3 =
        typename mpl::fold<EdgeTypes2,
                           mpl::vector<>,
                           mpl::push_front<mpl::placeholders::_1,
                                           mpl::placeholders::_2> /** _1 = State = set<...>, _2 element of NodeTypes*/
                           >::type;
    /** Experiment to inject the start end node into the edgetypes */
};

template <typename ContactGraphTraits>
class ContactGraph
{
public:
    using Traits    = ContactGraphTraits;
    using NodeTypes = typename Traits::NodeTypes;
    using EdgeTypes = typename Traits::EdgeTypes;

    using NodeStorageTuple = typename Traits::NodeStorageTuple;
    using EdgeStorageTuple = typename Traits::EdgeStorageTuple;

private:
    NodeStorageTuple m_nodeStorage;
    EdgeStorageTuple m_edgeStorage;

    /** Node Dispatcher: Moves visitor over all storages, and applies the visitor to the storage */
    template <typename NodeVisitor>
    struct NodeDispatcher
    {
        NodeDispatcher(NodeVisitor& n) : m_n(n)
        {
        }
        template <typename TStorage>
        inline void operator()(TStorage& t)
        {
            t.visit(m_n);
        }

        NodeVisitor& m_n;
    };

    /** Storage visitor: Delete all nodes */
    struct DeleteStorage
    {
        template <typename TStorage>
        void operator()(TStorage& t)
        {
            t.deleteAll();
        }
    };
    /** Storage visitor: Clear all storages */
    struct ClearStorages
    {
        template <typename TStorage>
        void operator()(TStorage& t)
        {
            t.clear();
        }
    };

    /** Storage Dispatcher: Moves visitor over all storages (nodes and edges) */
    struct StorageDispatchAll
    {
        template <typename TupleType,
                  unsigned int Idx = std::tuple_size<TupleType>::value - 1,
                  typename Visitor,
                  typename std::enable_if<(Idx > 0), void*>::type = nullptr>
        static void dispatch(Visitor& v, TupleType& t)
        {
            auto& c = std::get<Idx>(t);
            v(c);
            StorageDispatchAll::dispatch<TupleType, Idx - 1>(v, t);
        }

        template <typename TupleType,
                  unsigned int Idx,
                  typename Visitor,
                  typename std::enable_if<Idx == 0, void*>::type = nullptr>
        static void dispatch(Visitor& v, TupleType& t)
        {
            auto& c = std::get<Idx>(t);
            v(c);
        }
    };

    /** Get reference of the storage for a node type N */
    template <typename TNode>
    auto getNodeStorageRef() -> typename Traits::template nodeStorageTypeByNodeType<TNode>::type&
    {
        return std::get<Traits::template nodeStorageTypeByNodeType<TNode>::idx>(m_nodeStorage);  // get container
    }

    /** Get reference of the storage for a node type N */
    template <typename TEdge>
    auto getEdgeStorageRef() -> typename Traits::template edgeStorageTypeByEdgeType<TEdge>::type&
    {
        return std::get<Traits::template edgeStorageTypeByEdgeType<TEdge>::idx>(m_edgeStorage);  // get container
    }

public:
    ~ContactGraph()
    {
    }

    void clear()
    {
        ClearStorages c;
        StorageDispatchAll::dispatch(c, m_nodeStorage);
    }

    template <typename TNodeData>
    void reserveNodes(std::size_t nodes)
    {
        using NodeType = typename Traits::template nodeTypeByNodeData<TNodeData>::type;
        auto& c        = getNodeStorageRef<NodeType>();
        c.reserve(nodes);
    }

    template <typename TEdgeData>
    void reserveEdges(std::size_t edges)
    {
        using EdgeType = typename Traits::template edgeTypeByEdgeData<TEdgeData>::type;
        auto& c        = getEdgeStorageRef<EdgeType>();
        c.reserve(edges);
    }

    void deleteNodes()
    {
        DeleteStorage d;
        StorageDispatchAll::dispatch(d, m_nodeStorage);
    }

    void deleteEdges()
    {
        DeleteStorage d;
        StorageDispatchAll::dispatch(d, m_edgeStorage);
    }

    void deleteAll()
    {
        deleteNodes();
        deleteEdges();
    }

    template <typename TNodeData>
    auto getNode(std::size_t i) -> decltype(
        this->template getNodeStorageRef<typename Traits::template nodeTypeByNodeData<TNodeData>::type>().getNode(i))
    {
        using NodeType = typename Traits::template nodeTypeByNodeData<TNodeData>::type;
        return getNodeStorageRef<NodeType>().getNode(i);
    }

    template <typename TNodeData, typename... T>
    std::pair<typename Traits::template nodeTypeByNodeData<TNodeData>::type*, bool> insertNode(T&&... t)
    {
        using NodeType = typename Traits::template nodeTypeByNodeData<TNodeData>::type;
        return getNodeStorageRef<NodeType>().insert(std::forward<T>(t)...);
    }

    template <typename TEdgeData, typename... T>
    std::pair<typename Traits::template edgeTypeByEdgeData<TEdgeData>::type*, bool> insertEdge(T&&... t)
    {
        using EdgeType = typename Traits::template edgeTypeByEdgeData<TEdgeData>::type;
        return getEdgeStorageRef<EdgeType>().insert(std::forward<T>(t)...);
    }

    /** Visit all nodes */
    template <typename Visitor>
    void visitNodes(Visitor& v)
    {
        NodeDispatcher<Visitor> n(v);
        StorageDispatchAll::dispatch(n, m_nodeStorage);
    }

    constexpr std::size_t sizeOfNodeStorages()
    {
        return std::tuple_size<NodeStorageTuple>::value;
    };
    constexpr std::size_t sizeOfEdgeStorages()
    {
        return std::tuple_size<EdgeStorageTuple>::value;
    };
};

template <typename Graph>
struct TestVisitor
{
    using NodeTypes = typename Graph::NodeTypes;

    template <typename T>
    inline void operator()(T& t)
    {
        visit(t.getData());
    }

    void visit(NodeDataA& t)
    {
        // std::cout << "Visitor NodeA: " << typeid(t).name() << std::endl;
        c++;
    }

    void visit(NodeDataB& t)
    {
        c++;  // std::cout << "Visitor NodeB: " << typeid(t).name() << std::endl;
    }
    unsigned int c = 0;
};

template <class T>
void foo();

void contactGraphTest()
{
    using NodeA  = NodeDataA;
    using NodeB  = NodeDataB;
    using Traits = ContactGraphTraits<std::tuple<NodeDataA, NodeDataB>, std::tuple<EdgeDataAA, EdgeDataAB, EdgeDataBB>>;

    using GraphType = ContactGraph<Traits>;

    typename GraphType::Traits::template nodeTypeByNodeData<NodeDataA>::type aaa(1);

    GraphType g;
    int       a    = 3;
    double    b    = 3.4;
    int       size = 1000000;

    //    g.reserveNodes<NodeA>(size);
    //    g.reserveNodes<NodeB>(size);

    START_TIMER(ins1);
    for (int i = 0; i < size; i++)
    {
        g.insertNode<NodeA>(a);
        g.insertNode<NodeB>(b);
    }
    STOP_TIMER_MILLI(insend1, ins1);
    std::cout << "Insert took: " << insend1 << " ms" << std::endl;

    std::cout << " Next Visitor : " << std::endl;
    TestVisitor<GraphType> v;
    START_TIMER(start);
    g.visitNodes(v);
    STOP_TIMER_MILLI(count, start);
    std::cout << "Visitor took: " << count << " ms"
              << " counter: " << v.c << std::endl;

    v.c = 0;

    g.clear();
    std::cout << "Cleared graph " << std::endl;

    START_TIMER(ins2);
    for (int i = 0; i < size; i++)
    {
        g.insertNode<NodeA>(a);
        g.insertNode<NodeB>(b);
    }
    STOP_TIMER_MILLI(insend2, ins2);
    std::cout << "Insert took: " << insend2 << " ms" << std::endl;

    START_TIMER(start2);
    g.visitNodes(v);
    STOP_TIMER_MILLI(count2, start2);
    std::cout << "visitor took: " << count2 << " ms"
              << " counter: " << v.c << std::endl;

    // insert some edges
    g.insertEdge<EdgeDataAA>(a);
    {
        using a_t = typename Traits::nodeTypeByNodeData<NodeA>::type;
        using b_t = typename Traits::nodeTypeByNodeData<NodeB>::type;

        // a_t::asd;
        // b_t::asd;

        using c_t = typename Traits::edgeTypeByNodeType<a_t, b_t>::type;
        // c_t::asd;

        using d_t = typename Traits::edgeDataByEdgeType<c_t>::type;
        // d_t::asd;

        using e_t = typename Traits::edgeDataByNodeData<NodeA, NodeB>::type;
        // e_t::asd;

        g.insertEdge<e_t>(a);
    }

    // Example: Connect nodeB 1 with nodeB 1
    auto* n1 = g.getNode<NodeA>(0);
    auto* n2 = g.getNode<NodeB>(0);
    auto  ep = g.insertEdge<typename Traits::edgeDataByNodeData<NodeA, NodeB>::type>(a);
    std::cout << n1->getData().a << std::endl;
    std::cout << n2->getData().a << std::endl;
    ep.first->setStart(n1);  // not type safe;
    ep.first->setEnd(n2);
    n1->getEdgeListRef().push_back(ep.first);

    Traits::EdgeTypes3::as;
    // Traits::nodeTypesByIndex<2>::typeA::asd;
    std::cout << "Node Storages: " << g.sizeOfNodeStorages() << std::endl;
    std::cout << "Edge Storages: " << g.sizeOfEdgeStorages() << std::endl;
}

#endif
