#ifndef GRSF_Common_TupleHelper_hpp
#define GRSF_Common_TupleHelper_hpp

#include <functional>
#include <tuple>

/** from http://stackoverflow.com/questions/2590677/how-do-i-combine-hash-values-in-c0x */

template<typename>
class TupleHash;

template<typename... TTypes>
class TupleHash<std::tuple<TTypes...> >
{
private:
    using Tuple = std::tuple<TTypes...>;

    template<int N>
    size_t combine_hash(const Tuple & value) const { return 0; }

    template<int N, typename THead, typename... TTail>
    size_t combine_hash(const Tuple & value) const
    {
        constexpr int Index = N - sizeof...(TTail) - 1;
        size_t seed = combine_hash<N, TTail...>(value); // get seed of last value
        seed ^= std::hash<THead>()(std::get<Index>(value)) + 0x9e3779b9 + (seed<<6) + (seed>>2);
        return seed;
    }

public:
    size_t operator()(const Tuple & value) const
    {
        return combine_hash<sizeof...(TTypes), TTypes...>(value);
    }
};


/** Tuple Index */

template <class T, class Tuple>
struct TupleIndex;

template <class T, class... Types>
struct TupleIndex<T, std::tuple<T, Types...> > {
    static const std::size_t value = 0;
};

template <class T, class U, class... Types>
struct TupleIndex<T, std::tuple<U, Types...> > {
    static const std::size_t value = 1 + TupleIndex<T, std::tuple<Types...> >::value;
};




/**  Moves visitor over all entries */
struct TupleVisit{

    template<typename TupleType,
             unsigned int Idx = std::tuple_size<TupleType>::value -1 ,
             typename Visitor,
             typename std::enable_if< (Idx > 0), void*>::type = nullptr
                     >
    static void dispatch(Visitor & v, TupleType & t) {
        auto & c = std::get<Idx>(t);
        v(c);
        TupleVisit::dispatch<TupleType,Idx-1>(v,t);
    }

    template<typename TupleType,
             unsigned int Idx,
             typename Visitor,
             typename std::enable_if< Idx == 0, void*>::type = nullptr
             >
    static void dispatch(Visitor & v, TupleType & t) {
        auto & c = std::get<Idx>(t);
        v(c);
    }
};


#endif

