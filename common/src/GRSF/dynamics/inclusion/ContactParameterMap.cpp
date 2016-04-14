// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/dynamics/inclusion/ContactParameterMap.hpp"

#include "GRSF/common/Asserts.hpp"


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


