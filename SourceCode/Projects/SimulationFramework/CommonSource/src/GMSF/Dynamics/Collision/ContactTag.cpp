#include "GMSF/Dynamics/Collision/ContactTag.hpp"

ContactTag::ContactTag(){
   m_tag = boost::make_tuple(0, 0, 0, 0, 0, 0);
}

ContactTag::ContactTag( std::uint64_t b1, unsigned char type1, unsigned int id1, std::uint64_t b2 , unsigned char type2 , unsigned int id2)
{
   ASSERTMSG(b1 != b2, "Error: uint64_t have the same value: "<<b1<<" , something is wrong!");
      if(b1>b2){
         std::swap(b1,b2);
         std::swap(type1,type2);
         std::swap(id1,id2);
      }
      m_tag = boost::make_tuple(b1, type1, id1, b2, type2, id2);
}

bool ContactTag::operator==(ContactTag const& c2) const
{
   if( (m_tag.get<0>() == c2.m_tag.get<0>() &&
      m_tag.get<1>() == c2.m_tag.get<1>() &&
      m_tag.get<2>() == c2.m_tag.get<2>() &&

      m_tag.get<3>() == c2.m_tag.get<3>() &&
      m_tag.get<4>() == c2.m_tag.get<4>() &&
      m_tag.get<5>() == c2.m_tag.get<5>() )
      || ( //or if it is switched
      m_tag.get<0>() == c2.m_tag.get<3>() &&
      m_tag.get<1>() == c2.m_tag.get<4>() &&
      m_tag.get<2>() == c2.m_tag.get<5>() &&

      m_tag.get<3>() == c2.m_tag.get<0>() &&
      m_tag.get<4>() == c2.m_tag.get<1>() &&
      m_tag.get<5>() == c2.m_tag.get<2>())
      )
   {
      return true;
   }
   return false;
}


std::size_t ContactTagHash::operator()(ContactTag const& c) const
{
   std::size_t seed = 0;

   boost::hash_combine(seed, c.m_tag.get<0>());
   boost::hash_combine(seed, c.m_tag.get<1>());
   boost::hash_combine(seed, c.m_tag.get<2>());

   boost::hash_combine(seed, c.m_tag.get<3>());
   boost::hash_combine(seed, c.m_tag.get<4>());
   boost::hash_combine(seed, c.m_tag.get<5>());

   return seed;
}
