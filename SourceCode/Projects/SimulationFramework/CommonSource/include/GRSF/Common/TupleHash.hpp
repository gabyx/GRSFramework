#ifndef GRSF_Common_TupleHash_hpp
#define GRSF_Common_TupleHash_hpp

#include <functional>
#include <tuple>

/** from http://stackoverflow.com/questions/2590677/how-do-i-combine-hash-values-in-c0x */

namespace TupleHash
{

    template<typename>
    class hash;

    template<typename... TTypes>
    class hash<std::tuple<TTypes...> >
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
};

#endif

