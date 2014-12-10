#ifndef GMSF_Dynamics_Collision_ContactTag_hpp
#define GMSF_Dynamics_Collision_ContactTag_hpp

#include <functional>
#include <cstdint>
#include <boost/tuple/tuple.hpp>
#include <boost/functional/hash.hpp>


#include "GMSF/Common/AssertionDebug.hpp"

class ContactTag;

/**
* @ingroup Contact
* @brief This is the ContactTag functor which hashs a ContactTag!
*/
class ContactTagHash : std::unary_function<ContactTag, std::size_t>{
  public:
   std::size_t operator()(ContactTag const& c) const;
};

/**
* @ingroup Contact
* @brief This is the ContactTag class which is used for the hash table boost::unordered_map.
*/
/** @{ */
class ContactTag{
public:

  using ContactTagTuple = boost::tuples::tuple<std::uint64_t, unsigned char, unsigned int, std::uint64_t, unsigned char, unsigned int>;

  ContactTag();

  ContactTag( std::uint64_t b1, unsigned char type1, unsigned int id1, std::uint64_t b2 , unsigned char type2 , unsigned int id2);

   bool operator==(ContactTag const& c2) const;

   friend class ContactTagHash;

/**
* This tuple builds up the hash. It consists of:
* - Body with smalled id is always first!
* - uint64_t: Body1 Ptr
* - unsigned char: Type1: None = 0, Face = 1, Edge = 2, Vertex = 3
* - unsigned int: Id of the geomTypeId1
* - uint64_t: Body2 Ptr
* - unsigned char: Type2: None = 0, Face = 1, Edge = 2, Vertex = 3
* - unsigned int: Id of the geomTypeId2
*/
   ContactTagTuple m_tag;
private:

};


/**
* @ingroup Contact
* This is the function to generate a ContactTag from two bodies:
* @param b1 Pointer to body 1.
* @param type1 Type of the contact at body 1. None = 0, Face = 1, Edge = 2, Vertex = 3
* @param id1 The id of the contact type1.
* @param b2 Pointer to body 2.
* @param type2 Type of the contact at body 2. None = 0, Face = 1, Edge = 2, Vertex = 3
* @param id2 The id of the contact type2.
*/
template<typename TRigidBody>
ContactTag makeContactTag( const TRigidBody* b1, unsigned char type1, unsigned int id1, const TRigidBody * b2 , unsigned char type2 , unsigned int id2)
{
   // Make sure we always construct the same Tag!
   if(b1->m_id < b2->m_id){
      return ContactTag(b1->m_id,type1,id1,b2->m_id,type2,id2);
   }
   return ContactTag(b2->m_id,type2,id2,b1->m_id,type1,id1);

}
/** @} */
#endif
