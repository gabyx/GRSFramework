#ifndef GRSF_Dynamics_Collision_ContactTag_hpp
#define GRSF_Dynamics_Collision_ContactTag_hpp



#include <GRSF/Common/TypeDefs.hpp>
#include "GRSF/Common/AssertionDebug.hpp"

#include "GRSF/Common/TupleHash.hpp"

#include "GRSF/Dynamics/General/RigidBodyId.hpp"




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
