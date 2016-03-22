#include "GRSF/Dynamics/Collision/ContactTag.hpp"


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



