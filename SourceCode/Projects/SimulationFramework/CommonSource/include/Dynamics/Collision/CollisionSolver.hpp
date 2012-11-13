#ifndef CollisionSolver_hpp
#define CollisionSolver_hpp

#include <fstream>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/variant.hpp>
#include <OGRE/Ogre.h>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

//#define SRUTIL_DELEGATE_PREFERRED_SYNTAX
#include <srutil/delegate/delegate.hpp> // Use fast SR delegates

#include "AssertionDebug.hpp"

#include "TypeDefs.hpp"

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
class ContactDelegateList{
public:

   typedef TRigidBody RigidBodyType;
   typedef typename RigidBodyType::LayoutConfigType LayoutConfigType;
   DEFINE_LAYOUT_CONFIG_TYPES_OF(RigidBodyType::LayoutConfigType)

   ContactDelegateList(){
     m_ContactDelegateList.clear();
   }

    #ifdef SRUTIL_DELEGATE_PREFERRED_SYNTAX
      typedef srutil::delegate<void, (CollisionData<RigidBodyType>*) > ContactDelegate; ///< This is the delegate type which is used, when a new contact is found then all delegates are invoked in the list.
   #else
      typedef srutil::delegate1<void, CollisionData<RigidBodyType>*  > ContactDelegate; ///< This is the delegate type which is used, when a new contact is found then all delegates are invoked in the list.
   #endif

      /** Adds a new ContactDelegate which will be invoked during the solveCollision() part.*/
   void addContactDelegate(const ContactDelegate & cD){
      m_ContactDelegateList.push_back(cD);
   }
   void invokeAll(CollisionData<RigidBodyType> *pCollData) const{
      typename std::vector<ContactDelegate>::const_iterator it;
      for(it = m_ContactDelegateList.begin(); it != m_ContactDelegateList.end(); it++){
         (*it)(pCollData);
      }
   }

   inline bool isEmpty(){ return m_ContactDelegateList.empty();}

private:
   std::vector<ContactDelegate> m_ContactDelegateList;
};

/**
* @ingroup Collision
* @brief This is the CollisionSolver class, which basically solves the collision.
*/
/** @{ */
template< typename TCollisionSolverConfig >
class CollisionSolver{
public:

  typedef TCollisionSolverConfig CollisionSolverConfig;
  DEFINE_COLLISION_SOLVER_CONFIG_TYPES_OF(TCollisionSolverConfig)

  /**
  * @brief Constructor for the collision solver.
  * @param nSimBodies How many bodies are simulated. This should match #SimBodies.size().
  * @param SimBodies A reference to the list of all simulated bodies.
  * @param Bodies A reference to the list all not simulated bodies.
  */
  CollisionSolver(typename DynamicsSystemType::RigidBodySimPtrListType & SimBodies,
                  typename DynamicsSystemType::RigidBodyNotAniPtrListType & Bodies);

   ~CollisionSolver();

  void initializeLog(Logging::Log* pSolverLog);                          ///< Initializes an Ogre::Log.
  void reset();                                                       ///< Resets the whole Solver. This function is called at the start of the simulation.
  void reserveCollisionSetSpace(unsigned int nContacts);              ///< Reserves some space for the collision set.
  void solveCollision();    ///< Main routine which solves the collision for all bodies.

  std::vector< CollisionData<RigidBodyType> * > m_CollisionSet;       ///< This list is only used if no  ContactDelegate is in m_ContactDelegateList, then the contacts are simply added here.
  typedef typename std::vector< CollisionData<RigidBodyType> * > CollisionSet;
  inline void clearCollisionSet();
  ContactDelegateList<RigidBodyType> m_ContactDelegateList;



protected:
  //Inclusion Solver needs access to everything!
  template< typename TInclusionSolverConfig> friend class InclusionSolverNT;
  template< typename TInclusionSolverConfig> friend class InclusionSolverCO;
  template< typename TInclusionSolverConfig> friend class InclusionSolverCONoG;


  const unsigned int m_nDofqObj, m_nDofuObj, m_nSimBodies;
  unsigned int m_expectedNContacts;                                                 ///< Expected number of Contacts.
  typename DynamicsSystemType::RigidBodySimPtrListType & m_SimBodies;       ///< TODO: Add DynamicsSystem pointer, List of all simulated bodies.
  typename DynamicsSystemType::RigidBodyNotAniPtrListType & m_Bodies;          ///< List of all fixed not simulated bodies.

  Collider<LayoutConfigType, CollisionSolver<TCollisionSolverConfig> > m_Collider;                                               ///< The collider class, which is used as a functor which handles the different collisions.
  friend class Collider<LayoutConfigType, CollisionSolver<TCollisionSolverConfig> >;

  Logging::Log *  m_pSolverLog;  ///< Ogre::Log
  std::stringstream logstream;

  inline void signalContactAdd(CollisionData<RigidBodyType> * pColData); ///< Adds the contact either sends it to the delegate functions or it adds it in the set m_CollisionSet if no delegate has been added.

};
/** @} */


template< typename TCollisionSolverConfig >
CollisionSolver<TCollisionSolverConfig>::CollisionSolver(
                                         typename DynamicsSystemType::RigidBodySimPtrListType & SimBodies,
                                         typename DynamicsSystemType::RigidBodyNotAniPtrListType & Bodies):
m_SimBodies(SimBodies), m_Bodies(Bodies),
m_nSimBodies(SimBodies.size()),m_nDofqObj(NDOFqObj),m_nDofuObj(NDOFuObj)
{
   m_Collider.init(this);
    m_expectedNContacts = 10;
}

template< typename TCollisionSolverConfig >
CollisionSolver<TCollisionSolverConfig>::~CollisionSolver()
{
   clearCollisionSet();
}


template< typename TCollisionSolverConfig >
void CollisionSolver<TCollisionSolverConfig>::initializeLog( Logging::Log* pSolverLog )
{
  m_pSolverLog = pSolverLog;
  ASSERTMSG(m_pSolverLog != NULL, "Logging::Log: NULL!");
}


template< typename TCollisionSolverConfig >
void CollisionSolver<TCollisionSolverConfig>::reset()
{
  // Do a Debug check if sizes match!
  ASSERTMSG( m_SimBodies.size() != 0, "CollisionSolver:: No Bodies added to the system!");


  clearCollisionSet();

  reserveCollisionSetSpace(m_nSimBodies * 3);

}

template< typename TCollisionSolverConfig >
void CollisionSolver<TCollisionSolverConfig>::clearCollisionSet()
{
   for( typename CollisionSet::iterator it = m_CollisionSet.begin(); it != m_CollisionSet.end(); it++){
        delete (*it);
   }
   m_CollisionSet.clear();
}

template< typename TCollisionSolverConfig >
void CollisionSolver<TCollisionSolverConfig>::reserveCollisionSetSpace(unsigned int nContacts)
{
  m_expectedNContacts = nContacts;
}


template< typename TCollisionSolverConfig >
void CollisionSolver<TCollisionSolverConfig>::solveCollision(){


  clearCollisionSet();

   #if CoutLevelSolver>1
      LOG(m_pSolverLog, " % -> solveCollision(): "<<std::endl;)
   #endif



    // All objects have been updated...

    //// Do simple collision detection (SimBodies to SimBodies)
    typename DynamicsSystemType::RigidBodySimPtrListType::iterator bodyIti;
    for(bodyIti = m_SimBodies.begin(); bodyIti != --m_SimBodies.end(); bodyIti++){
      typename DynamicsSystemType::RigidBodySimPtrListType::iterator bodyItj = bodyIti;
      bodyItj++;
      for(; bodyItj != m_SimBodies.end(); bodyItj++ ){

        //check for a collision
        m_Collider.checkCollision((*bodyIti), (*bodyItj));


      }
    }


    // Do simple collision detection (SimBodies to Bodies)
    typename DynamicsSystemType::RigidBodyNotAniPtrListType::iterator bodyItk;
    for(bodyIti = m_SimBodies.begin(); bodyIti != m_SimBodies.end(); bodyIti++){
        for(bodyItk = m_Bodies.begin(); bodyItk != m_Bodies.end(); bodyItk ++){

            //check for a collision
            m_Collider.checkCollision((*bodyIti), (*bodyItk));

        }
    }
}

template<typename TCollisionSolverConfig>
  inline void CollisionSolver<TCollisionSolverConfig>::signalContactAdd(CollisionData<RigidBodyType> * pColData){

     // Before we send, determine what kind of contactmodel we have!
     // TODO (implemented only NContactModel)
     ASSERTMSG( std::abs(pColData->m_e_x.dot(pColData->m_e_y)) < 1e-3 && std::abs(pColData->m_e_y.dot(pColData->m_e_z))< 1e-3, "Vectors not parallel");

     m_CollisionSet.push_back(pColData); // Copy it to the owning list! colData gets deleted!


      if(!m_ContactDelegateList.isEmpty()){
         m_ContactDelegateList.invokeAll(m_CollisionSet.back()); // Propagate pointers! they will not be deleted!
      }
  }

#endif
