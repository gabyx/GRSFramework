#ifndef GRSF_Dynamics_Collision_ContactTag_hpp
#define GRSF_Dynamics_Collision_ContactTag_hpp

#include <functional>
#include <tuple>

#include <GRSF/Common/TypeDefs.hpp>
#include "GRSF/Common/AssertionDebug.hpp"

#include "GRSF/Dynamics/General/RigidBodyId.hpp"



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




/**
* @ingroup Contact
* @brief This is the ContactTag class which is used for the hash table boost::unordered_map.
*/
/** @{ */
class ContactTag{
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES

    using ContactTagTuple = std::tuple< RigidBodyIdType, unsigned char, unsigned int, RigidBodyIdType , unsigned char, unsigned int>;

    void set( const RigidBodyIdType & b1,
              unsigned char type1,
              unsigned int id1,
              const RigidBodyIdType & b2 ,
              unsigned char type2 ,
              unsigned int id2);

    bool operator==(ContactTag const& c2) const;

    friend class ContactTagHash;

private:

    /**
    * This tuple builds up the hash. It consists of:
    * - Body with smaller id is always first!
    * - uint64_t: Body1 Ptr
    * - unsigned char: Type1: None = 0, Face = 1, Edge = 2, Vertex = 3
    * - unsigned int: some id, e.g. face idx, edge idx, vertex idx
    * - uint64_t: Body2 Ptr
    * - unsigned char: Type2: None = 0, Face = 1, Edge = 2, Vertex = 3
    * - unsigned int: some id, e.g. face idx, edge idx, vertex idx
    */
    ContactTagTuple m_tag{}; //value initialized
};

/**
* @ingroup Contact
* @brief This is the ContactTag functor which hashs a ContactTag!
*/
class ContactTagHash : std::unary_function<ContactTag, std::size_t>{
public:
    std::size_t operator()(ContactTag const& c) const {
        TupleHash::hash<ContactTag::ContactTagTuple> hasher;
        return hasher(c.m_tag);
    }
};


#endif
