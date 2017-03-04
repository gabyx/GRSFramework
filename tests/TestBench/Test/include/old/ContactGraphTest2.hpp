// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include <meta.hpp>

#include "GRSF/common/CPUTimer.hpp"
#include "GRSF/common/DemangleTypes.hpp"
#include "GRSF/common/EnumClassHelper.hpp"
#include "GRSF/common/StaticAssert.hpp"
#include "GRSF/common/TupleHelper.hpp"

#include <tuple>
#include <vector>

#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/iteration/local.hpp>

#include "GRSF/dynamics/inclusion/GeneralGraph.hpp"

using namespace Graph;

struct NodeDataA
{
};
struct NodeDataB
{
};
struct NodeDataC
{
};
struct NodeDataD
{
};  // Dummy

struct EdgeDataAA
{
};
struct EdgeDataAB
{
};
struct EdgeDataAC
{
};
struct EdgeDataBB
{
};
struct EdgeDataBC
{
};
struct EdgeDataCC
{
};

template <typename Traits>
struct TestVisitor
{
    using NodeDataTypes = typename Traits::NodeDataTypes;

    template <typename T>
    inline void operator()(T& t)
    {
        visit(t.getData());

        EdgeVisitor e;
        t.visitOutEdges(e);
        // std::cout << " Visited edges: " << e.c << " for " << demangle::type<T>().substr(0,20) << std::endl;
    }

    void visit(NodeDataA& t)
    {
        c++;
    }

    void visit(NodeDataB& t)
    {
        c++;
    }

    void visit(NodeDataC& t)
    {
        c++;
    }

    struct EdgeVisitor
    {
        template <typename T>
        inline void operator()(T& t)
        {
            visit(t.getData());
        }

        void visit(EdgeDataAA& t)
        {
            c++;
        }

        void visit(EdgeDataAB& t)
        {
            c++;
        }

        void visit(EdgeDataAC& t)
        {
            c++;
        }

        void visit(EdgeDataBB& t)
        {
            c++;
        }

        void visit(EdgeDataBC& t)
        {
            c++;
        }

        void visit(EdgeDataCC& t)
        {
            c++;
        }

        unsigned int c = 0;
    };

    unsigned int c = 0;
};

int contactGraphTest()
{
    int size = 1000000;
    {
        using cg =
            GraphTraitsSymmetric<meta::list<NodeDataA, NodeDataB, NodeDataC>,
                                 meta::list<EdgeDataAA, EdgeDataAB, EdgeDataAC, EdgeDataBB, EdgeDataBC, EdgeDataCC>>;

        std::cout << "InEdge Types B: " << demangle::type(Node<NodeDataB, cg>::InEdgeDataTypes()) << std::endl;
        // std::cout << "InEdge Types Storage B: " << type(Node<NodeDataB,cg>::InEdgeStoragesTypes() ) << std::endl;
        std::cout << "OutEdge Types B: " << demangle::type(Node<NodeDataB, cg>::OutEdgeDataTypes()) << std::endl;

        Node<NodeDataA, cg> a(0);
        Node<NodeDataB, cg> b(1);
        Node<NodeDataC, cg> c(2);

        Edge<EdgeDataAB, cg> e_ab(0);
        e_ab.setStartNode(&a);
        e_ab.setEndNode(&b);

        Edge<EdgeDataBC, cg> e_bc(0);
        e_bc.setStartNode(&b);
        e_bc.setEndNode(&c);

        a.addEdgeIn(&e_ab);
        b.addEdgeOut(&e_ab);

        b.addEdgeIn(&e_bc);
        c.addEdgeOut(&e_bc);

        Edge<EdgeDataAC, cg> e_ca(0);
        e_ca.setStartNode(&c);
        e_ca.setEndNode(&a);

        GeneralGraph<cg> g;

        START_TIMER(ins1);
        for (int i = 0; i < size; i++)
        {
            g.insertNode<NodeDataA>();
            g.insertNode<NodeDataB>();
        }
        STOP_TIMER_MILLI(insend1, ins1);
        std::cout << "Insert took: " << insend1 << " ms" << std::endl;

        std::cout << " Next Visitor : " << std::endl;
        TestVisitor<cg> v;
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
            g.insertNode<NodeDataA>();
            g.insertNode<NodeDataB>();
        }
        STOP_TIMER_MILLI(insend2, ins2);
        std::cout << "Insert took: " << insend2 << " ms" << std::endl;

        START_TIMER(start2);
        g.visitNodes(v);
        STOP_TIMER_MILLI(count2, start2);
        std::cout << "visitor took: " << count2 << " ms"
                  << " counter: " << v.c << std::endl;
    }
    {
        using cg =
            GraphTraitsSymmetric<meta::list<NodeDataA, NodeDataB, NodeDataC>,
                                 meta::list<EdgeDataAA, EdgeDataAA, EdgeDataAA, EdgeDataAA, EdgeDataAA, EdgeDataAA>>;

        std::cout << "InEdge Types B: " << demangle::type(Node<NodeDataB, cg>::InEdgeDataTypes()) << std::endl;
        // std::cout << "InEdge Types Storage B: " << type(Node<NodeDataB,cg>::InEdgeStoragesTypes() ) << std::endl;
        std::cout << "OutEdge Types B: " << demangle::type(Node<NodeDataB, cg>::OutEdgeDataTypes()) << std::endl;

        Node<NodeDataA, cg> a(0);
        Node<NodeDataB, cg> b(0);
        Node<NodeDataC, cg> c(0);

        Edge<EdgeDataAA, cg> e_ab(0);
        e_ab.setStartNode(&a);
        e_ab.setEndNode(&b);

        Edge<EdgeDataAA, cg> e_bc(0);
        e_bc.setStartNode(&b);
        e_bc.setEndNode(&c);

        a.addEdgeIn(&e_ab);
        b.addEdgeOut(&e_ab);

        b.addEdgeIn(&e_bc);
        c.addEdgeOut(&e_bc);

        Edge<EdgeDataAA, cg> e_ca(0);
        e_ca.setStartNode(&c);
        e_ca.setEndNode(&a);
    }

    {
        std::cout << "Visitor Test: " << std::endl;
        using cg =
            GraphTraitsSymmetric<meta::list<NodeDataA, NodeDataB, NodeDataC>,
                                 meta::list<EdgeDataAA, EdgeDataAB, EdgeDataAC, EdgeDataBB, EdgeDataBC, EdgeDataCC>>;
        GeneralGraph<cg> g;

        std::default_random_engine gen;
        std::uniform_int_distribution<int> d(0, size - 1);

        START_TIMER(ins1);
        for (int i = 0; i < size; i++)
        {
            g.insertNode<NodeDataA>();
            g.insertNode<NodeDataB>();
            g.insertNode<NodeDataC>();
        }
        STOP_TIMER_MILLI(insend1, ins1);
        std::cout << "Insert took: " << insend1 << " ms" << std::endl;

        // Connections from A to (A,B,C)
        for (int i = 0; i < size; i++)
        {
            auto idx = d(gen);
            auto a   = g.getNode<NodeDataA>(idx);

            auto s    = d(gen);
            auto idx2 = d(gen);
            if (s < size / 3)
            {
                auto aa = g.getNode<NodeDataA>(idx2);
                auto e  = g.insertEdge<EdgeDataAA>().first;
                e->setStartNode(a);
                e->setEndNode(aa);
                a->addEdgeOut(e);
                aa->addEdgeIn(e);
            }
            else if (s >= size / 3 && s < 2 * size / 3)
            {
                auto bb = g.getNode<NodeDataB>(idx2);
                auto e  = g.insertEdge<EdgeDataAB>().first;
                e->setStartNode(a);
                e->setEndNode(bb);
                a->addEdgeOut(e);
                bb->addEdgeIn(e);
            }
            else
            {
                auto cc = g.getNode<NodeDataC>(idx2);
                auto e  = g.insertEdge<EdgeDataAC>().first;
                e->setStartNode(a);
                e->setEndNode(cc);
                a->addEdgeOut(e);
                cc->addEdgeIn(e);
            }
        }

        // Connections from A to (A,B,C)
        for (int i = 0; i < size; i++)
        {
            auto idx = d(gen);
            auto a   = g.getNode<NodeDataC>(idx);

            auto s    = d(gen);
            auto idx2 = d(gen);
            if (s < size / 3)
            {
                auto aa = g.getNode<NodeDataA>(idx2);
                auto e  = g.insertEdge<EdgeDataAC>().first;
                e->setStartNode(a);
                e->setEndNode(aa);
                a->addEdgeOut(e);
                aa->addEdgeIn(e);
            }
            else if (s >= size / 3 && s < 2 * size / 3)
            {
                auto bb = g.getNode<NodeDataB>(idx2);
                auto e  = g.insertEdge<EdgeDataBC>().first;
                e->setStartNode(a);
                e->setEndNode(bb);
                a->addEdgeOut(e);
                bb->addEdgeIn(e);
            }
            else
            {
                auto cc = g.getNode<NodeDataC>(idx2);
                auto e  = g.insertEdge<EdgeDataCC>().first;
                e->setStartNode(a);
                e->setEndNode(cc);
                a->addEdgeOut(e);
                cc->addEdgeIn(e);
            }
        }

        TestVisitor<cg> v;
        START_TIMER(start2);
        g.visitNodes(v);
        STOP_TIMER_MILLI(count2, start2);
        std::cout << "visitor took: " << count2 << " ms"
                  << " counter: " << v.c << std::endl;
    }

    {
        // Fails to compile if you replace EdgeDataAC with void
        // ( then NodeDataA has no support for a In or Out Type, compile error which is good!)
        using cg = GraphTraitsSymmetric<meta::list<NodeDataA, NodeDataB, NodeDataC>,
                                        meta::list<void, void, EdgeDataAC, void, void, EdgeDataCC>>;

        GeneralGraph<cg> g;
        g.insertNode<NodeDataC>();
        auto a = g.insertNode<NodeDataA>().first;
        g.insertEdge<EdgeDataCC>();
        // g.insertEdge<EdgeDataBC>(); // compile error no edge with this type

        Edge<EdgeDataAC, cg>* edge = 0;

        a->addEdgeIn(edge);
        a->addEdgeOut(edge);

        Edge<EdgeDataBC, cg>* edge2 = 0;
        auto b = g.insertNode<NodeDataB>().first;
        // b->addEdgeOut(edge2);
    }

    {
        using cg = GraphTraitsSymmetric<meta::list<NodeDataA, NodeDataB, NodeDataC>,
                                        meta::list<void, void, void, void, void, void>>;

        GeneralGraph<cg> g;

        auto n = g.insertNode<NodeDataC>().first;
        std::cout << sizeof(*n) << std::endl;
    }
    return 0;
}
