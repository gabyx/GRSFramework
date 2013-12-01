#ifndef CollisionSolver_hpp
#define CollisionSolver_hpp

#include <fstream>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/variant.hpp>

#include <boost/shared_ptr.hpp>

//#define SRUTIL_DELEGATE_PREFERRED_SYNTAX
#include <srutil/delegate/delegate.hpp> // Use fast SR delegates

#include "AssertionDebug.hpp"

#include "TypeDefs.hpp"
#include "ContactFrame.hpp"
#include "CollisionData.hpp"
#include "Collider.hpp"
#include "LogDefines.hpp"

#include "QuaternionHelpers.hpp"

#include "SimpleLogger.hpp"

/**
* @ingroup Collision
* @brief Contact Delegate List which is used to store all callbacks which are invoked when a new contact has been found!
*/
template< typename TRigidBody>
class ContactDelegateList {
public:

    typedef TRigidBody RigidBodyType;
    typedef typename RigidBodyType::LayoutConfigType LayoutConfigType;
    DEFINE_LAYOUT_CONFIG_TYPES_OF(RigidBodyType::LayoutConfigType)

    ContactDelegateList() {
        m_ContactDelegateList.clear();
    }

#ifdef SRUTIL_DELEGATE_PREFERRED_SYNTAX
    typedef srutil::delegate<void, (CollisionData<RigidBodyType>*) > ContactDelegate; ///< This is the delegate type which is used, when a new contact is found then all delegates are invoked in the list.
#else
    typedef srutil::delegate1<void, CollisionData<RigidBodyType>*  > ContactDelegate; ///< This is the delegate type which is used, when a new contact is found then all delegates are invoked in the list.
#endif

    /** Adds a new ContactDelegate which will be invoked during the solveCollision() part.*/
    void addContactDelegate(const ContactDelegate & cD) {
        m_ContactDelegateList.push_back(cD);
    }
    void invokeAll(CollisionData<RigidBodyType> *pCollData) const {
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

    typedef typename std::vector< CollisionData<RigidBodyType> * > CollisionSetType;

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

    ContactDelegateList<RigidBodyType> m_ContactDelegateList;

    unsigned int m_expectedNContacts;                                                 ///< Expected number of Contacts.
    typename DynamicsSystemType::RigidBodySimContainerType & m_SimBodies;       ///< TODO: Add DynamicsSystem pointer, List of all simulated bodies.
    typename DynamicsSystemType::RigidBodyNotAniContainer & m_Bodies;          ///< List of all fixed not simulated bodies.

    Collider m_Collider;                                               ///< The collider class, which is used as a functor which handles the different collisions.
    friend class Collider;

    Logging::Log *  m_pSolverLog;  ///< Ogre::Log
    std::stringstream logstream;

    inline void signalContactAdd(); ///< Sends all contact found by the collider which are in m_collisionSet to the delegate!


    PREC m_maxOverlap;

};
/** @} */



CollisionSolver::CollisionSolver(boost::shared_ptr< DynamicsSystemType> pDynSys):
    m_SimBodies(pDynSys->m_SimBodies), m_Bodies(pDynSys->m_Bodies),
    m_Collider(&m_collisionSet)
{
    m_expectedNContacts = 300;
}


CollisionSolver::~CollisionSolver() {
    clearCollisionSet();
}



void CollisionSolver::initializeLog( Logging::Log* pSolverLog ) {
    m_pSolverLog = pSolverLog;
    ASSERTMSG(m_pSolverLog != NULL, "Logging::Log: NULL!");
}



void CollisionSolver::reset() {
    // Do a Debug check if sizes match!
    ASSERTMSG( m_SimBodies.size() != 0, "CollisionSolver:: No Bodies added to the system!");


    clearCollisionSet();

    m_expectedNContacts =  m_SimBodies.size() * 3;


    m_maxOverlap = 0;

}


void CollisionSolver::clearCollisionSet() {
    for( typename CollisionSetType::iterator it = m_collisionSet.begin(); it != m_collisionSet.end(); it++) {
        delete (*it);
    }
    m_collisionSet.clear();
}


const typename CollisionSolver::CollisionSetType &
CollisionSolver::getCollisionSetRef()
{
    return m_collisionSet;
}



void CollisionSolver::solveCollision() {


    clearCollisionSet();

#if CoutLevelSolver>1
    LOG(m_pSolverLog, " % -> solveCollision(): "<<std::endl;)
#endif



    // All objects have been updated...

    //// Do simple collision detection (SimBodies to SimBodies)
    typename DynamicsSystemType::RigidBodySimContainerType::iterator bodyIti;
    CollisionData<RigidBodyType> * pColData;
//    for(bodyIti = m_SimBodies.begin(); bodyIti != --m_SimBodies.end(); bodyIti++) {
//        typename DynamicsSystemType::RigidBodySimContainerType::iterator bodyItj = bodyIti;
//        bodyItj++;
//        for(; bodyItj != m_SimBodies.end(); bodyItj++ ) {
//
//            //check for a collision
//            m_Collider.checkCollision((*bodyIti), (*bodyItj));
//
//        }
//    }


    // Do simple collision detection (SimBodies to Bodies)
    typename DynamicsSystemType::RigidBodyNotAniContainer::iterator bodyItk;
    for(bodyIti = m_SimBodies.begin(); bodyIti != m_SimBodies.end(); bodyIti++) {
        for(bodyItk = m_Bodies.begin(); bodyItk != m_Bodies.end(); bodyItk ++) {

                //check for a collision and signal
                m_Collider.checkCollision((*bodyIti), (*bodyItk));


        }
    }

    // Signal all found contact
    signalContactAdd();

}


std::string CollisionSolver::getIterationStats() {
    std::stringstream s;
    s << m_maxOverlap;
    return s.str();
}

void CollisionSolver::signalContactAdd() {

    if(m_collisionSet.size()!=0){

        for( auto colDataIt = m_collisionSet.begin(); colDataIt != m_collisionSet.end(); colDataIt++ ){

            ASSERTMSG( std::abs((*colDataIt)->m_cFrame.m_e_x.dot((*colDataIt)->m_cFrame.m_e_y)) < 1e-3 &&
                      std::abs((*colDataIt)->m_cFrame.m_e_y.dot((*colDataIt)->m_cFrame.m_e_z))< 1e-3, "Vectors not orthogonal");

            #if CoutLevelSolverWhenContact>2
                LOG(m_pSolverLog,"Contact Frame: n: " << (*colDataIt)->m_cFrame.m_e_z.transpose() << std::endl;)
            #endif

            //Set contact frame point
            (*colDataIt)->m_cFrame.m_p = (*colDataIt)->m_pBody1->m_r_S + (*colDataIt)->m_r_S1C1;

            // Calculate some Statistics
            m_maxOverlap = std::max(m_maxOverlap,(*colDataIt)->m_overlap);

            m_ContactDelegateList.invokeAll(*colDataIt); // Propagate pointers! they will not be deleted!

        }

    }
}

#endif
