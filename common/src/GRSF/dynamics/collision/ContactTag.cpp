// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/dynamics/collision/ContactTag.hpp"


void ContactTag::set( const RigidBodyIdType & b1,
                      unsigned char type1,
                      unsigned int id1,
                      const RigidBodyIdType & b2 ,
                      unsigned char type2 ,
                      unsigned int id2) {

    ASSERTMSG(b1 != b2, "Error: ContactTag:: body id the same: "<<RigidBodyId::getBodyIdString(b1));

    // Make sure we always construct the same Tag!
    if(b1 < b2) {
        m_tag = std::make_tuple(b1, type1, id1, b2, type2, id2);
    } else {
        m_tag = std::make_tuple(b2, type2, id2, b1, type1, id1);
    }
}

bool ContactTag::operator==(ContactTag const& c2) const {
    return m_tag == c2.m_tag;
}



