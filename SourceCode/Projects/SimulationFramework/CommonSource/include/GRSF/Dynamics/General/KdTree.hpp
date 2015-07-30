

#ifndef GRSF_Dynamics_General_KdTree_hpp
#define GRSF_Dynamics_General_KdTree_hpp

#include <boost/serialization/level.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/split_member.hpp>

#include "GRSF/Dynamics/General/SerializationHelpersEigen.hpp"

#include "ApproxMVBB/KdTree.hpp"

// namespace alias
namespace KdTree = ApproxMVBB::KdTree;


/** Forward declaration */
template<typename Tree> class KdTreeSerializer;

namespace ApproxMVBB{
    namespace KdTree {

        /*

        /** Inherit from TreeSimple and add some serialization */
        template<
            typename TTraits
        >
        class TreeNodeSimpleS : public TreeNodeSimple<TTraits,TreeNodeSimpleS<TTraits> >
        {
            public:

            using Traits = TTraits;
            using Base = TreeNodeSimple<TTraits,TreeNodeSimpleS<TTraits> >;

            template<typename Tree>
            friend class ::KdTreeSerializer;

            TreeNodeSimpleS(): Base() {}

            /** Copy from a Tree TTree if possible*/
            template< typename TNode>
            explicit TreeNodeSimpleS( TNode && node): Base( std::forward<TNode>(node) )
            {}


        };

        /*

        /** Inherit from TreeSimple and add some serialization */
        template<
            typename TTraits = TreeSimpleTraits<KdTree::NoData<3>, KdTree::TreeNodeSimpleS >
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
