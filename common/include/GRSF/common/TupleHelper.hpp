// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_TupleHelper_hpp
#define GRSF_common_TupleHelper_hpp

#include <functional>
#include <tuple>

#include "GRSF/common/MetaHelper.hpp"
#include "GRSF/common/StaticAssert.hpp"

/** from http://stackoverflow.com/questions/2590677/how-do-i-combine-hash-values-in-c0x */

template <typename>
class TupleHash;

template <typename... TTypes>
class TupleHash<std::tuple<TTypes...>>
{
private:
    using Tuple = std::tuple<TTypes...>;

    template <int N>
    size_t combine_hash(const Tuple& value) const
    {
        return 0;
    }

    template <int N, typename THead, typename... TTail>
    size_t combine_hash(const Tuple& value) const
    {
        constexpr int Index = N - sizeof...(TTail) - 1;
        size_t seed         = combine_hash<N, TTail...>(value);  // get seed of last value
        seed ^= std::hash<THead>()(std::get<Index>(value)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }

public:
    size_t operator()(const Tuple& value) const
    {
        return combine_hash<sizeof...(TTypes), TTypes...>(value);
    }
};

/** Tuple Index */

template <class T, class Tuple>
struct TupleIndex;

template <class T, class... Types>
struct TupleIndex<T, std::tuple<T, Types...>>
{
    static const std::size_t value = 0;
};

template <class T, class U, class... Types>
struct TupleIndex<T, std::tuple<U, Types...>>
{
    static const std::size_t value = 1 + TupleIndex<T, std::tuple<Types...>>::value;
};

/**  Moves visitor over all entries */
namespace TupleVisit
{
/** Visit all recursion */
namespace details
{
template <std::size_t Idx = 0,
          typename Visitor,
          typename... T,
          typename std::enable_if<Idx == sizeof...(T), void*>::type = nullptr>
static inline void visitAll(Visitor&& v, std::tuple<T...>& t)
{
}

template <std::size_t Idx = 0,
          typename Visitor,
          typename... T,
          typename std::enable_if<Idx<sizeof...(T), void*>::type = nullptr> static inline void visitAll(
              Visitor&& v, std::tuple<T...>& t)
{
    v(std::get<Idx>(t));
    visitAll<Idx + 1, Visitor, T...>(std::forward<Visitor>(v), t);
}
};

/** Visit only certain types recursion */
namespace details
{
template <typename List>
struct visitIndices;

template <template <typename...> class List>
struct visitIndices<List<>>
{
    template <typename Visitor, typename Tuple>
    inline static void apply(Visitor&& v, Tuple& t)
    {
    }
};

template <template <typename...> class List, typename I, typename... O>
struct visitIndices<List<I, O...>>
{
    template <typename Visitor, typename Tuple>
    inline static void apply(Visitor&& v, Tuple& t)
    {
        v(std::get<I::value>(t));
        visitIndices<meta::list<O...>>::apply(std::forward<Visitor>(v), t);
    }
};

template <typename... Types, typename Visitor, typename... T>
inline static void visitSpecific(Visitor&& v, std::tuple<T...>& t)
{
    using types      = meta::list<Types...>;
    using tupleTypes = meta::list<T...>;
    // make indices for all types (removing unkown indexes)
    // gets all indices for each type in "types" and joins them to a single indices list
    using indices = metaAdd::remove<
        meta::npos,
        meta::join<meta::transform<types, meta::bind_back<meta::quote<metaAdd::find_indices>, tupleTypes>>>>;

    GRSF_STATIC_ASSERTM(indices::size() != 0, "Tried to visit tuple but the types have not been found!");

    // Run Visitor for each index
    visitIndices<indices>::apply(std::forward<Visitor>(v), t);
}
};

/// Visits all types which match \p First till \p Others  in the tuple \p t  and applies the visitor \p v.
/// visit<int,int,float>( vis , std::tuple<int,int,double>{}) will visit two times all int types and one time float
template <typename First, typename... Others, typename Visitor, typename... T>
inline void visit(Visitor&& v, std::tuple<T...>& t)
{
    details::visitSpecific<First, Others...>(v, t);
}

/// Visits all types in the tuple \p t  and applies the visitor \p v.
template <typename Visitor, typename... T>
inline void visit(Visitor&& v, std::tuple<T...>& t)
{
    details::visitAll(v, t);
}
};

#endif
