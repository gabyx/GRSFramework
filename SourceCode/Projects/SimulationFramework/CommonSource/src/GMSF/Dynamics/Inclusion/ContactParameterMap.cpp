#include "GMSF/Dynamics/Inclusion/ContactParameterMap.hpp"

#include "GMSF/Common/AssertionDebug.hpp"


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
    m_std_values =  ContactParameter::createParams_UCF_ContactModel(0.5,0.5,0.3);
}


