#ifndef PercussionPool_hpp
#define PercussionPool_hpp

#include <utility>
#include <boost/unordered_map.hpp>
#include <vector>

#include "TypeDefs.hpp"

#include "ContactTag.hpp"
#include "ContactPercussion.hpp"



/** Usage:

   ...
   getPercussion();  sets used flag = true
   removeUnusedPercussions() removes all used flag = false
   ... (prox)
   setPercussion(); sets used flag = false;
   clearUsedPercussionList();
   ...
*/

/**
* @ingroup Inclusion
* @brief Percussion Pool.
*/
template< typename TLayoutConfig>
class PercussionPool    {
public:
   typedef boost::unordered_map<ContactTag, ContactPercussion, ContactTagHash > PercussionMap;

   DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)

   PercussionPool(){

   }

   void rehashPercussionPool(unsigned int n){
      m_PercussionMap.rehash(n);
   }

   void removeUnusedPercussions(){
//       unsigned int size1 = m_PercussionMap.size();
      //cout << "Removing percussion...: " << m_PercussionMap.size() <<endl ;
      typename PercussionMap::iterator it;
      it = m_PercussionMap.begin();
      while(it != m_PercussionMap.end()){
         if(it->second.m_bUsed == false){ // the percussion was not used in the last iteration
            it = m_PercussionMap.erase(it); // it points to the next element!
         }else{ // the percussion was used in the last iteration, so reset flag = not used
            it->second.m_bUsed = false;
            it++;
         }
      }
//      std::cout << "Removed percussion: "<< size1 - m_PercussionMap.size()  <<std::endl;
   }

   void clearUsedPercussionList(){
      m_usedPercussionList.clear();
   }

   void getPercussion(const ContactTag & tag, VectorDyn & P_old) {
      static typename std::pair<ContactTag, ContactPercussion > new_value(ContactTag(), ContactPercussion(P_old.rows()) );  // Standart constructor, Percussion = zero
      new_value.first = tag;

      // try to insert a new one!
      std::pair<typename PercussionMap::iterator, bool> result =  m_PercussionMap.insert(new_value);
//      if(result.second == false){
//        std::cout << "Took Precussion with" << result.first->second.m_P <<std::endl;
//      }
      // If inserted the new values are set to zero!
      // If not inserted the new values are the found ones in the map!
      ASSERTMSG(result.first->second.m_P.rows() == P_old.rows(), "Something went wrong you found a contact which has not the same dimension as the one you want to initialize!");
      P_old = result.first->second.m_P;

      // Set  used flag
      result.first->second.m_bUsed = true;
      // Fill in into the list
      m_usedPercussionList.push_back(result.first);
   }

   void setPercussion(unsigned int index, const VectorDyn & P_new) {
      ASSERTMSG(index < m_usedPercussionList.size(),"Error: False index!");

      typename PercussionMap::iterator it = m_usedPercussionList[index];

      ASSERTMSG(it->second.m_P.rows() == P_new.rows(), "Something went wrong you want to set a contact which has not the same dimension as the one you want to initialize!");
      it->second.m_P = P_new;

//       std::cout << "Set Precussion with" << P_new.transpose() <<std::endl;

   }

   unsigned int getPoolSize(){
      return (unsigned int)m_PercussionMap.size();
   }

private:
   PercussionMap m_PercussionMap; // The percussion map!

   std::vector<typename PercussionMap::iterator> m_usedPercussionList; // used to quickly access the elements which where used, to be able to write back the percussions after Prox
};


#endif
