#ifndef CollisionSolver_hpp
#define CollisionSolver_hpp

#include <fstream>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/variant.hpp>

#include <boost/shared_ptr.hpp>

//#define SRUTIL_DELEGATE_PREFERRED_SYNTAX
#include <srutil/delegate/delegate.hpp> // Use fast SR delegates

#include "TypeDefs.hpp"
#include "LogDefines.hpp"
#include "AssertionDebug.hpp"

#include DynamicsSystem_INCLUDE_FILE

#include "ContactFrame.hpp"
#include "CollisionData.hpp"
#include "Collider.hpp"
#include "SimpleLogger.hpp"

#include "QuaternionHelpers.hpp"

/**
* @ingroup Collision
* @brief Contact Delegate List which is used to store all callbacks which are invoked when a new contact has been found!
*/

class ContactDelegateList {
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES

    ContactDelegateList() {
        m_ContactDelegateList.clear();
    }

#ifdef SRUTIL_DELEGATE_PREFERRED_SYNTAX
    typedef srutil::delegate<void, (CollisionData*) > ContactDelegate; ///< This is the delegate type which is used, when a new contact is found then all delegates are invoked in the list.
#else
    typedef srutil::delegate1<void, CollisionData*  > ContactDelegate; ///< This is the delegate type which is used, when a new contact is found then all delegates are invoked in the list.
#endif

    /** Adds a new ContactDelegate which will be invoked during the solveCollision() part.*/
    void addContactDelegate(const ContactDelegate & cD) {
        m_ContactDelegateList.push_back(cD);
    }
    void invokeAll(CollisionData *pCollData) const {
        typename std::vector<ContactDelegate>::const_iterator it;
        for(it = m_ContactDelegateList.begin(); it != m_ContactDelegateList.end(); it++) {
            (*it)(pCollData);
        }
    }

    inline bool isEmpty() {
        return m_ContactDelegateList.empty();
    }

private:
    std::vector<ContactDelegate> m_ContactDelegateList;
};

/**
* @ingroup Collision
* @brief This is the CollisionSolver class, which basically solves the collision.
*/
/** @{ */

class CollisionSolver {
public:

    DEFINE_COLLISION_SOLVER_CONFIG_TYPES

    typedef typename std::vector< CollisionData * > CollisionSetType;

    /**
    * @brief Constructor for the collision solver.
    * @param SimBodies A reference to the list of all simulated bodies.
    * @param Bodies A reference to the list all not simulated bodies.
    */
    CollisionSolver(boost::shared_ptr< DynamicsSystemType> pDynSys);

    ~CollisionSolver();

    void initializeLog(Logging::Log* pSolverLog);                          ///< Initializes an Ogre::Log.
    void reset();                                                       ///< Resets the whole Solver. This function is called at the start of the simulation.
    void solveCollision();    ///< Main routine which solves the collision for all bodies.


    const CollisionSetType & getCollisionSetRef();

    inline void clearCollisionSet();

    std::string getIterationStats();

protected:


    CollisionSetType m_collisionSet;       ///< This list is only used if no  ContactDelegate is in m_ContactDelegateList, then the contacts are simply added here.


    //Inclusion Solver needs access to everything!
    //class InclusionSolverNT;
    friend class InclusionSolverCO;
    friend class InclusionSolverCONoG;

    ContactDelegateList m_ContactDelegateList;

    unsigned int m_expectedNContacts;                                                 ///< Expected number of Contacts.
    typename DynamicsSystemType::RigidBodySimContainerType & m_SimBodies;       ///< TODO: Add DynamicsSystem pointer, List of all simulated bodies.
    typename DynamicsSystemType::RigidBodyNotAniContainer & m_Bodies;          ///< List of all fixed not simulated bodies.

    Collider m_Collider;                                               ///< The collider class, which is used as a functor which handles the different collisions.
    friend class Collider;

    Logging::Log *  m_pSolverLog;

    inline void signalContactAdd(); ///< Sends all contact found by the collider which are in m_collisionSet to the delegate!


    PREC m_maxOverlap;

};
/** @} */


#endif
