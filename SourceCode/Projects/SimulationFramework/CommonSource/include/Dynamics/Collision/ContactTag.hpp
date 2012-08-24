#ifndef ContactTag_hpp
#define ContactTag_hpp
#include <boost/tuple/tuple.hpp>
#include <boost/functional/hash.hpp>
#include <boost/unordered_map.hpp>
#include <boost/cstdint.hpp>

#include "AssertionDebug.hpp"

class ContactTag;

/**
* @ingroup Contact
* @brief This is the ContactTag has functor which hashs a ContactTag!
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

  typedef boost::tuples::tuple<boost::uint64_t, unsigned char, unsigned int, boost::uint64_t, unsigned char, unsigned int> ContactTagTuple;

  ContactTag();

  ContactTag( boost::uint64_t b1, unsigned char type1, unsigned int id1, boost::uint64_t b2 , unsigned char type2 , unsigned int id2);

   bool operator==(ContactTag const& c2) const;

   friend class ContactTagHash;

/**
* This tuple builds up the hash. It consists of:
* - uint64_t: Body1 Ptr
* - unsigned char: Type1: None = 0, Face = 1, Edge = 2, Vertex = 3
* - unsigned int: Id of the geomTypeId1
* - uint64_t: Body2 Ptr
* - unsigned char: Type2: None = 0, Face = 1, Edge = 2, Vertex = 3
* - unsigned int: Id of the geomTypeId2
*/
   ContactTagTuple m_ContactTagTuple;
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
ContactTag makeContactTag( TRigidBody* b1, unsigned char type1, unsigned int id1, TRigidBody * b2 , unsigned char type2 , unsigned int id2)
{
   // Make sure we always construct the same Tag!
   if(b1->m_id < b2->m_id){
      boost::uint64_t intptr_b1 = reinterpret_cast<boost::uint64_t>(b1);
      boost::uint64_t intptr_b2 = reinterpret_cast<boost::uint64_t>(b2);
      return ContactTag(intptr_b1,type1,id1,intptr_b2,type2,id2);
   }
   boost::uint64_t intptr_b1 = reinterpret_cast<boost::uint64_t>(b1);
   boost::uint64_t intptr_b2 = reinterpret_cast<boost::uint64_t>(b2);
   return ContactTag(intptr_b2,type2,id2,intptr_b1,type1,id1);

}
/** @} */
#endif
