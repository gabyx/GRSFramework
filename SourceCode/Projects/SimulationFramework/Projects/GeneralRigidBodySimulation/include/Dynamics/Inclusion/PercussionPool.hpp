#ifndef PercussionPool_hpp
#define PercussionPool_hpp

#include <utility>
#include <boost/unordered_map.hpp>
#include <vector>
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
   typedef boost::unordered_map<ContactTag, ContactPercussion<TLayoutConfig>, ContactTagHash > PercussionMap;

   DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)

   PercussionPool(){

   }

   void rehashPercussionPool(unsigned int n){
      m_PercussionMap.rehash(n);
   }

   void removeUnusedPercussions(){
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
      //cout << "Removed percussion: "<< m_PercussionMap.size() <<endl ;
   }

   void clearUsedPercussionList(){
      m_usedPercussionList.clear();
   }

   void getPercussion(const ContactTag & tag, VectorPContact & P_old) {
      static typename std::pair<ContactTag, ContactPercussion<TLayoutConfig> > new_value;  // Standart constructor, Percussion = zero
      new_value.first = tag;
      // try to insert a new one!
      std::pair<typename PercussionMap::iterator, bool> result =  m_PercussionMap.insert(new_value);
      /*if(result.second == false){
      cout << "Took Precussion with" << tag.m_ContactTagTuple.get<0>()<<"/"<<tag.m_ContactTagTuple.get<3>()<<endl;
      }*/
      // If inserted the new values are set to zero!
      // If not inserted the new values are the found ones in the map!
      P_old = result.first->second.m_P;

      // Set  used flag
      result.first->second.m_bUsed = true;
      // Fill in into the list
      m_usedPercussionList.push_back(result.first);
   }

   void setPercussion(unsigned int index, const VectorPContact & P_new) {
      ASSERTMSG(index < m_usedPercussionList.size(),"Error: False index!");
      m_usedPercussionList[index]->second.m_P = P_new;
   }

   unsigned int getPoolSize(){
      return (unsigned int)m_PercussionMap.size();
   }

private:
   PercussionMap m_PercussionMap; // The percussion map!

   std::vector<typename PercussionMap::iterator> m_usedPercussionList; // used to quickly access the elements which where used, to be able to write back the percussions after Prox
};


#endif
