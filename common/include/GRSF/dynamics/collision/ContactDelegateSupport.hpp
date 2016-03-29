// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_collision_ContactDelegateSupport_hpp
#define GRSF_dynamics_collision_ContactDelegateSupport_hpp

#include <vector>

//#define SRUTIL_DELEGATE_PREFERRED_SYNTAX
#include <srutil/delegate/delegate.hpp> // Use fast SR delegates

#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/AssertionDebug.hpp"

/**
* @ingroup Collision
* @brief Contact Delegate List which is used to store all callbacks which are invoked when a new contact has been found!
*/

class ContactDelegateSupport {
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES

    typedef srutil::delegate1<void, CollisionData*  > ContactDelegateType; ///< This is the delegate type which is used, when a new contact is found then all delegates are invoked in the list.

    /** Adds a new ContactDelegateType which will be invoked during the solveCollision() part.*/
    void addContactDelegate(const ContactDelegateType & cD) {
        m_contactDelegateList.push_back(cD);
    }
    void invokeAllContactDelegates(CollisionData *pCollData) const {
        for(auto & f : m_contactDelegateList) {
            f(pCollData);
        }
    }

    void removeAllContactDelegates(){
        m_contactDelegateList.clear();
    }

private:
    std::vector<ContactDelegateType> m_contactDelegateList;
};

#endif
