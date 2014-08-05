#ifndef ContactDelegateSupport_hpp
#define ContactDelegateSupport_hpp

#include <vector>

//#define SRUTIL_DELEGATE_PREFERRED_SYNTAX
#include <srutil/delegate/delegate.hpp> // Use fast SR delegates

#include "TypeDefs.hpp"
#include "LogDefines.hpp"
#include "AssertionDebug.hpp"

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
