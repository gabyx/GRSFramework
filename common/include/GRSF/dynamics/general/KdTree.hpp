// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_KdTree_hpp
#define GRSF_dynamics_general_KdTree_hpp

#include <boost/serialization/level.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/split_member.hpp>

#include "GRSF/common/SerializationHelpersEigen.hpp"

#include "ApproxMVBB/KdTree.hpp"

// namespace alias
namespace KdTree = ApproxMVBB::KdTree;


/** Forward declaration */
template<typename Tree> class KdTreeSerializer;

namespace ApproxMVBB{
    namespace KdTree {

        /** Inherit from TreeSimple and add some serialization */
        template<
            typename TTraits
        >
        class NodeSimpleS : public NodeSimple<TTraits,NodeSimpleS<TTraits> >
        {
            public:

            using Traits = TTraits;
            using Base = NodeSimple<TTraits,NodeSimpleS<TTraits> >;

            template<typename Tree>
            friend class ::KdTreeSerializer;

            NodeSimpleS(): Base() {}

            /** Copy from a Tree TTree if possible*/
            template< typename TNode>
            explicit NodeSimpleS( TNode && node): Base( std::forward<TNode>(node) )
            {}


        };


        /** Inherit from TreeSimple and add some serialization */
        template<
            typename TTraits = TreeSimpleTraits<KdTree::NoData<3>, KdTree::NodeSimpleS >
        >
        class TreeSimpleS : public TreeSimple<TTraits>
        {

            public:

            template<typename Tree>
            friend class ::KdTreeSerializer;

            using Traits = TTraits;
            using Base = TreeSimple<TTraits>;

            /** Standart constructor */
            TreeSimpleS(): Base() {}

            /** Copy from a Tree TTree if possible*/
            template< typename TTree>
            explicit TreeSimpleS( TTree && tree): Base( std::forward<TTree>(tree) )
            {}


        };

    };

};



#endif
