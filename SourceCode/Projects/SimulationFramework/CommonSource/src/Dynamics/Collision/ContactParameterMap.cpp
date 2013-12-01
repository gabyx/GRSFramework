#include <boost/tuple/tuple.hpp>
#include <boost/functional/hash.hpp>
#include <boost/unordered_map.hpp>

#include "ContactParameterMap.hpp"

#include "AssertionDebug.hpp"


ContactParameterTag::ContactParameterTag( unsigned int materialid1, unsigned int materialid2){

    if(materialid1>materialid2) {
        std::swap(materialid1,materialid2);
    }
    m_tag = boost::make_tuple(materialid1,materialid2);
}

bool ContactParameterTag::operator==(ContactParameterTag const& c2) const {
    if( (   m_tag.get<0>() == c2.m_tag.get<0>() &&
            m_tag.get<1>() == c2.m_tag.get<1>()
         )
        || ( //or if it is switched
            m_tag.get<0>() == c2.m_tag.get<1>() &&
            m_tag.get<1>() == c2.m_tag.get<0>()
        )
      )
    {
        return true;
    }
    return false;
}


std::size_t ContactParameterTagHash::operator()(ContactParameterTag const& c) const {
    std::size_t seed = 0;

    boost::hash_combine(seed, c.m_tag.get<0>());
    boost::hash_combine(seed, c.m_tag.get<1>());

    return seed;
}


ContactParameterMap::ContactParameterMap() {
    m_nMaterials = 1;
    m_std_values.m_epsilon_N = 0.5;
    m_std_values.m_epsilon_T = 0.5;
    m_std_values.m_mu = 0.3;

}

bool ContactParameterMap::addContactParameter(typename RigidBodyType::BodyMaterialType  material1,
                                              typename RigidBodyType::BodyMaterialType  material2,
                                              const ContactParams & params)
{
    typename ContactParameterMapType::value_type pair(ContactParameterTag(material1,material2),params);

    std::pair<  typename ContactParameterMapType::iterator, bool> res = m_ContactParams.insert(pair);

    if(res.second){
        m_nMaterials++;
    }

    return res.second;
}


ContactParams & ContactParameterMap::getContactParams(  typename RigidBodyType::BodyMaterialType material1,
                                                        typename RigidBodyType::BodyMaterialType material2){

     typename ContactParameterMapType::iterator it = m_ContactParams.find(ContactParameterTag(material1,material2));

    if(it != m_ContactParams.end()){
        return it->second;
    }
    ASSERTMSG(false,"ContactParams for id: "<< material1 <<" and " <<material2 << " not found"<<std::endl);
    return m_std_values;
}
