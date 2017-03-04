// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef TemplateStateTree_hpp
#define TemplateStateTree_hpp

#include <tuple>
#include <type_traits>

namespace TemplateTree1
{
struct NoType
{
};

template <typename T>
struct Leaf
{
    static const unsigned int NLeafs = 1;
    T m_t;

    //	template<unsigned int N>
    //	struct getType{
    //        using type = T;
    //	};
    //
    //	template<unsigned int N = 0>
    //	T & get(){
    //        return m_t;
    //	}
};

/** Determine number of types Leaf<T> in a tuple T =====================================*/
template <typename T>
struct determineLeafs
{
    // Add 1 if type is a Leaf<T>
    template <typename TT>
    static constexpr unsigned int getValue(Leaf<TT>* t = nullptr)
    {
        return 1;
    }
    // Add 0 if type something different
    template <typename TT>
    static constexpr unsigned int getValue(TT* t = nullptr)
    {
        return 0;
    }

    // Recursion over all elements in the tuple, each element needs to have a integral NLeafs
    // std::tuple_element<N,T>::type can be a type which computes NLeafs again by a certain function
    template <unsigned int N, typename std::enable_if<(N > 0)>::type* = nullptr>
    static constexpr unsigned int getNLeafs()
    {
        return std::tuple_element<N, T>::type::NLeafs + getNLeafs<N - 1>();
    };
    // Abort if tuple index is 0
    template <unsigned int N, typename std::enable_if<(N == 0)>::type* = nullptr>
    static constexpr unsigned int getNLeafs()
    {
        return std::tuple_element<N, T>::type::NLeafs;
    };

    /** The number of Leaf<T> types in the tuple T , this travers also into the types in the tuple T */
    static const unsigned int value = getNLeafs<std::tuple_size<T>::value - 1>();
};
/* ======================================================================================================*/

/** Leaf Type Getter ====================================================================================*/

/** Declaration for undefined spezialisation */
template <unsigned int N,           /* The global Leaf index */
          unsigned int CurrLeafIdx, /* For the recursion, current global leaf index */
          unsigned int ChildIdx,    /* Idx into the tuple of TState::TupleType */
          typename TState,          /* a type State<...> */
          typename Enable = void>   /* Enable only if ChildIdx < TState::NState */
struct getLeafType_imp;

/** Take this spezialization if  ChildIdx <= TState::NStates, traverse the tree */
template <unsigned int N, unsigned int CurrLeafIdx, unsigned int ChildIdx, typename TState>
struct getLeafType_imp<N, CurrLeafIdx, ChildIdx, TState, typename std::enable_if<(ChildIdx < TState::NStates)>::type>
{
    /** Defines the tuple element type */
    using TupleElement = typename TState::template getType<ChildIdx>::type;

    using type = typename std::conditional<
        (N >= CurrLeafIdx + TupleElement::NLeafs),  // if global leaf index > current + child::NLeafs then
        /* getType of next tuple entry ChildIdx+1*/
        typename getLeafType_imp<N, CurrLeafIdx + TupleElement::NLeafs, ChildIdx + 1, TState>::type,
        /* else visit TupleElement, set childIdx to 0 */
        typename getLeafType_imp<N, CurrLeafIdx, 0, TupleElement>::type>::type;
};

/** Take this spezialization if ChildIdx >= TState::NStates, return Leaf<NoType> */
template <unsigned int N, unsigned int CurrLeafIdx, unsigned int ChildIdx, typename TState>
struct getLeafType_imp<N, CurrLeafIdx, ChildIdx, TState, typename std::enable_if<(ChildIdx >= TState::NStates)>::type>
{
    using type = Leaf<NoType>;
};

/** Specialization for Leaf and if ChildIdx < TState::NStates */
template <unsigned int N, unsigned int CurrLeafIdx, unsigned int ChildIdx, typename T>
struct getLeafType_imp<N, CurrLeafIdx, ChildIdx, Leaf<T>, void>
{
    using type = T;
};

/** Get the leaf type with global index N */
template <unsigned int N, typename State>
struct getLeafType
{
    using type = typename getLeafType_imp<N, 0, 0, State>::type;
};

/* ======================================================================================================*/

/** Leaf Getter ====================================================================================*/

// next sub state
template <unsigned int ChildIdx, typename TState>
struct Traversal
{
    static_assert(ChildIdx < TState::NStates,
                  "This should not happen, we reached this States tuple size and did not find the type to get");

    using TupleElement = typename TState::template getType<ChildIdx>::type;

    template <unsigned int N,
              unsigned int CurrLeafIdx,
              typename std::enable_if<(N >= CurrLeafIdx + TupleElement::NLeafs)>::type* = nullptr>
    static auto visit(TState& state) ->
        typename getLeafType_imp<N, CurrLeafIdx + TupleElement::NLeafs, ChildIdx + 1, TState>::type&
    {
        return Traversal<ChildIdx + 1, TState>::template visit<N, CurrLeafIdx + TupleElement::NLeafs>(state);
    }

    template <unsigned int N,
              unsigned int CurrLeafIdx,
              typename std::enable_if<!(N >= CurrLeafIdx + TupleElement::NLeafs)>::type* = nullptr>
    static auto visit(TState& state) -> typename getLeafType_imp<N, CurrLeafIdx, 0, TupleElement>::type&
    {
        return Traversal<0, TupleElement>::template visit<N, CurrLeafIdx>(state.template get<ChildIdx>());
    }
};

// spezialize for Leaf<T>
template <unsigned int ChildIdx, typename T>
struct Traversal<ChildIdx, Leaf<T>>
{
    template <unsigned int N, unsigned int CurrLeafIdx>
    static auto visit(Leaf<T>& leaf) -> T&
    {
        return leaf.m_t;
    }
};

template <unsigned int N, typename TState>
typename getLeafType<N, TState>::type& getLeaf_imp(TState& state)
{
    return Traversal<0, TState>::template visit<N, 0>(state);
}
/* ======================================================================================================*/

template <typename... States>
struct State
{
    using TupleType = std::tuple<States...>;

    /** Global Number of Leafs seen from this State */
    static const unsigned int NLeafs = determineLeafs<TupleType>::value;

    /** Number of SubStates in this State */
    static const unsigned int NStates = std::tuple_size<TupleType>::value;

    /** Type Getter =============================================================*/
    template <unsigned int N, typename Enable = void>
    struct getType;

    // partial spezialization for getting type idx (N < NStates)
    template <unsigned int N>
    struct getType<N, typename std::enable_if<(N < NStates)>::type>
    {
        using type = typename std::tuple_element<N, TupleType>::type;
    };
    // partial spezialization for getting type idx (N >= NStates) which is undefined => Leaf<NoType>
    template <unsigned int N>
    struct getType<N, typename std::enable_if<(N >= NStates)>::type>
    {
        using type = Leaf<NoType>;
    };
    /*=============================================================================*/

    /** Value Getter for this State ==============================================================*/
    template <unsigned int N>
    typename std::tuple_element<N, TupleType>::type& get()
    {
        return std::get<N>(m_tuple);
    }
    /*=============================================================================*/

    /** Leaf Value Getter for this State ==============================================================*/
    template <unsigned int N>
    typename getLeafType<N, State>::type& getLeaf()
    {
        return getLeaf_imp<N, State>(*this);
    }
    /*=============================================================================*/

    /** Storage */
    TupleType m_tuple;
};
};

namespace TemplateTree2
{
struct NoType
{
};

template <typename... States>
struct State;

/** Get NLeafs type for an other type T */
template <typename T>
struct getNLeafs
{
    static const unsigned int value = 1;
};

/** Get NLeafs for a State<Args...> type */
template <typename... Args>
struct getNLeafs<State<Args...>>
{
    static const unsigned int value = State<Args...>::NLeafs;
};

/** Determine number of not State types in a tuple T =====================================*/
template <typename T>
struct determineLeafs
{
    // Recursion over all elements in the tuple, each element needs to have a integral NLeafs
    // std::tuple_element<N,T>::type can be a type which computes NLeafs again by a certain function
    template <unsigned int N, typename std::enable_if<(N > 0)>::type* = nullptr>
    static constexpr unsigned int determineLeafs_imp()
    {
        using TupleType = typename std::tuple_element<N, T>::type;
        return getNLeafs<TupleType>::value + determineLeafs_imp<N - 1>();
    };

    // Abort if tuple index is 0
    template <unsigned int N, typename std::enable_if<(N == 0)>::type* = nullptr>
    static constexpr unsigned int determineLeafs_imp()
    {
        using TupleType = typename std::tuple_element<N, T>::type;
        return getNLeafs<TupleType>::value;
    };

    /** The number of Leaf<T> types in the tuple T , this travers also into the types in the tuple T */
    static const unsigned int value = determineLeafs_imp<std::tuple_size<T>::value - 1>();
};
/* ======================================================================================================*/

/** Leaf Type Getter ====================================================================================*/

/** Take this spez. if we are at any type other thatn State<Args...> */
template <typename T>
struct getLeafType_imp
{
    template <unsigned int N, unsigned int ChildIdx>
    struct defineType
    {
        using type = T;
    };
};

/** Take this spezialization if we are at a State<Args...> */
template <typename... Args>
struct getLeafType_imp<State<Args...>>
{
    using TState = State<Args...>;

    /**  Declaration for spezialization */
    template <unsigned int N,         /* The global Leaf index */
              unsigned int ChildIdx,  /* Idx into the tuple of TState::TupleType */
              typename Enable = void> /* Enable only if ChildIdx < TState::NState */
    struct defineType;

    /** Take this spezialization if ChildIdx is in range of TState */
    template <unsigned int N, unsigned int ChildIdx>
    struct defineType<N, ChildIdx, typename std::enable_if<(ChildIdx < TState::NStates)>::type>
    {
        /** Defines the child element of the tuple */
        using ChildElement                      = typename TState::template getType<ChildIdx>::type;
        static const unsigned int NLeafsElement = getNLeafs<ChildElement>::value;

        using type =
            typename std::conditional<(N >=
                                       NLeafsElement),  // if current global leaf index N at TState > child::NLeafs then
                                      /* go to next element in this state */
                                      typename defineType<N - NLeafsElement, ChildIdx + 1>::type,
                                      /* else visit ChildElement, set childIdx to 0 */
                                      typename getLeafType_imp<ChildElement>::template defineType<N, 0>::type>::type;
    };

    /** Take this spezialization if ChildIdx is out of range (ChildIdx >= TState::NStates, return NoType) */
    template <unsigned int N, unsigned int ChildIdx>
    struct defineType<N, ChildIdx, typename std::enable_if<(ChildIdx >= TState::NStates)>::type>
    {
        using type = NoType;
    };
};

/** Get the leaf type with global index N */
template <unsigned int N, typename State>
struct getLeafType
{
    using type = typename getLeafType_imp<State>::template defineType<N, 0>::type;
};

/* ======================================================================================================*/

/** Leaf Getter ====================================================================================*/

template <typename T>
struct LeafGetter
{
    template <unsigned int N,        /* The global Leaf index */
              unsigned int ChildIdx> /* Idx into the tuple of TState::TupleType */
    struct getter
    {
        static T& get(T& t)
        {
            return t;
        }
    };
};

// next sub state
template <typename... Args>
struct LeafGetter<State<Args...>>
{
    using TState = State<Args...>;

    /**  Declaration for spezialization */
    template <unsigned int N,         /* The global Leaf index */
              unsigned int ChildIdx,  /* Idx into the tuple of TState::TupleType */
              typename Enable = void> /* Enable only if ChildIdx < TState::NState */
    struct getter;

    /** Getter function if N >= NLeafsElement, jump to next child in state*/
    template <unsigned int N, unsigned int ChildIdx>
    struct getter<
        N,
        ChildIdx,
        typename std::enable_if<(N >= getNLeafs<typename TState::template getType<ChildIdx>::type>::value)>::type>
    {
        using ChildElement                      = typename TState::template getType<ChildIdx>::type;
        static const unsigned int NLeafsElement = getNLeafs<ChildElement>::value;

        static auto get(TState& state) ->
            typename getLeafType_imp<TState>::template defineType<N - NLeafsElement, ChildIdx + 1>::type&
        {
            return LeafGetter<TState>::template getter<N - NLeafsElement, ChildIdx + 1>::get(state);
        }
    };

    /** Getter function if N < NLeafsElement, visit this state */
    template <unsigned int N, unsigned int ChildIdx>
    struct getter<
        N,
        ChildIdx,
        typename std::enable_if<!(N >= getNLeafs<typename TState::template getType<ChildIdx>::type>::value)>::type>
    {
        using ChildElement                      = typename TState::template getType<ChildIdx>::type;
        static const unsigned int NLeafsElement = getNLeafs<ChildElement>::value;

        static auto get(TState& state) -> typename getLeafType_imp<ChildElement>::template defineType<N, 0>::type&
        {
            return LeafGetter<ChildElement>::template getter<N, 0>::get(state.template get<ChildIdx>());
        }
    };
};

template <unsigned int N, typename TState, typename std::enable_if<(N < TState::NLeafs)>::type* = nullptr>
typename getLeafType<N, TState>::type& getLeaf_imp(TState& state)
{
    return LeafGetter<TState>::template getter<N, 0>::get(state);
}

template <unsigned int N, typename TState, typename std::enable_if<(N >= TState::NLeafs)>::type* = nullptr>
typename getLeafType<0, TState>::type& getLeaf_imp(TState& state)
{
    static_assert(N < TState::NLeafs, "Leaf index N out of range!");
    // return something which is not valid (return first leaf w)
    return LeafGetter<TState>::template getter<0, 0>::get(state);
}

/* ======================================================================================================*/

template <typename... States>
struct State
{
    using TupleType = std::tuple<States...>;

    /** Global Number of Leafs seen from this State */
    static const unsigned int NLeafs = determineLeafs<TupleType>::value;

    /** Number of SubStates in this State */
    static const unsigned int NStates = std::tuple_size<TupleType>::value;

    /** Type Getter =============================================================*/
    template <unsigned int N, typename Enable = void>
    struct getType;

    // partial spezialization for getting type idx (N < NStates)
    template <unsigned int N>
    struct getType<N, typename std::enable_if<(N < NStates)>::type>
    {
        using type = typename std::tuple_element<N, TupleType>::type;
    };
    // partial spezialization for getting type idx (N >= NStates) which is undefined => Leaf<NoType>
    template <unsigned int N>
    struct getType<N, typename std::enable_if<(N >= NStates)>::type>
    {
        using type = NoType;
    };
    /*=============================================================================*/

    /** Value Getter for this State ===============================================*/
    template <unsigned int N>
    typename std::tuple_element<N, TupleType>::type& get()
    {
        return std::get<N>(m_tuple);
    }
    /*=============================================================================*/

    /** Leaf Value Getter for this State (complicated) ============================*/
    template <unsigned int N>
    auto getLeaf() -> decltype(getLeaf_imp<N, State>(*this))
    {
        return getLeaf_imp<N, State>(*this);
    }
    /*=============================================================================*/

    /** Storage */
    TupleType m_tuple;
};
};

void templateStateTree()
{
    //    {
    //        // your code goes here
    //        using namespace TemplateTree1;
    //
    //        using myState = State<
    //                State<
    //                     Leaf<int>
    //                >,
    //                State<
    //                    Leaf<double>,
    //                    Leaf<int>
    //                >,
    //                Leaf<std::string>,
    //                State<
    //                    Leaf<double>,
    //                    Leaf<int>
    //                >,
    //                State<
    //                    Leaf<double>,
    //                    Leaf<int>
    //                >
    //            >;
    //
    //        myState ss;
    //
    //        std::cout << "Leafs: " << ss.NLeafs << std::endl;
    //        std::cout << "Leaf types:==========================" << std::endl;
    //        std::cout << typeid(typename getLeafType<0,myState>::type ).name() << std::endl;
    //        std::cout << typeid(typename getLeafType<1,myState>::type ).name() << std::endl;
    //        std::cout << typeid(typename getLeafType<2,myState>::type ).name() << std::endl;
    //        std::cout << typeid(typename getLeafType<3,myState>::type ).name() << std::endl;
    //        std::cout << typeid(typename getLeafType<4,myState>::type ).name() << std::endl;
    //        std::cout << typeid(typename getLeafType<5,myState>::type ).name() << std::endl;
    //        std::cout << typeid(typename getLeafType<6,myState>::type ).name() << std::endl;
    //        std::cout << typeid(typename getLeafType<7,myState>::type ).name() << std::endl;
    //        std::cout << typeid(typename getLeafType<8,myState>::type ).name() << std::endl;
    //        std::cout << "=====================================" << std::endl;
    //
    //        std::cout << "Get types:==========================" << std::endl;
    //        ss.getLeaf<0>() = 4;
    //        ss.getLeaf<1>() = 3.5;
    //        ss.getLeaf<3>() = "asds";
    //        std::cout << ss.getLeaf<0>() << std::endl;
    //        std::cout << ss.getLeaf<1>() << std::endl;
    //        std::cout << ss.getLeaf<2>() << std::endl;
    //        std::cout << ss.getLeaf<3>() << std::endl;
    //        std::cout << ss.getLeaf<4>() << std::endl;
    //        std::cout << ss.getLeaf<5>() << std::endl;
    //        std::cout << ss.getLeaf<6>() << std::endl;
    //        //std::cout << ss.getLeaf<7>() << std::endl;
    //
    //    }

    {
        using namespace TemplateTree2;

        using myState = State<State<int, State<float, State<double, State<int>>>>, State<int, float>, double>;

        myState ss;

        std::cout << "Leafs: " << ss.NLeafs << std::endl;
        std::cout << "Leaf types:==========================" << std::endl;
        std::cout << typeid(typename getLeafType<0, myState>::type).name() << std::endl;
        std::cout << typeid(typename getLeafType<1, myState>::type).name() << std::endl;
        std::cout << typeid(typename getLeafType<2, myState>::type).name() << std::endl;
        std::cout << typeid(typename getLeafType<3, myState>::type).name() << std::endl;
        std::cout << typeid(typename getLeafType<4, myState>::type).name() << std::endl;
        std::cout << typeid(typename getLeafType<5, myState>::type).name() << std::endl;
        std::cout << typeid(typename getLeafType<6, myState>::type).name() << std::endl;
        std::cout << typeid(typename getLeafType<7, myState>::type).name() << std::endl;
        std::cout << typeid(typename getLeafType<8, myState>::type).name() << std::endl;
        std::cout << "=====================================" << std::endl;
        ss.getLeaf<0>() = 4;
        ss.getLeaf<1>() = 3.5;
        ss.getLeaf<3>() = 4;
        std::cout << ss.getLeaf<0>() << std::endl;
        std::cout << ss.getLeaf<1>() << std::endl;
        std::cout << ss.getLeaf<2>() << std::endl;
        std::cout << ss.getLeaf<3>() << std::endl;
        std::cout << ss.getLeaf<4>() << std::endl;
        std::cout << ss.getLeaf<5>() << std::endl;
        std::cout << ss.getLeaf<6>() << std::endl;
        // std::cout << ss.getLeaf<7>() << std::endl;
    }
}

#endif  // TemplateStateTree_hpp
