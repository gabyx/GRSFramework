#include "GRSF/Dynamics/Inclusion/ContactParameterMap.hpp"

#include "GRSF/Common/AssertionDebug.hpp"


ContactParameterTag::ContactParameterTag( unsigned int materialid1, unsigned int materialid2){

    if(materialid1>materialid2) {
        std::swap(materialid1,materialid2);
    }
    m_tag = std::make_tuple(materialid1,materialid2);
}

bool ContactParameterTag::operator==(ContactParameterTag const& c2) const {
   return m_tag == c2.m_tag;
}


ContactParameterMap::ContactParameterMap() {
    m_std_values =  ContactParameter::createParams_UCF_ContactModel(0.5,0.5,0.3);
}


