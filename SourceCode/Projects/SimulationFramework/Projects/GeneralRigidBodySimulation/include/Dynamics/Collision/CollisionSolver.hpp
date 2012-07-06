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

#include "RigidBody.hpp"
#include "CollisionData.hpp"
#include "Collider.hpp"
#include "LogDefines.hpp"

#include "QuaternionHelpers.hpp"

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
      typedef srutil::delegate<void, (const DynamicsState<LayoutConfigType> *, CollisionData<RigidBodyType>*) > ContactDelegate; ///< This is the delegate type which is used, when a new contact is found then all delegates are invoked in the list.
   #else
      typedef srutil::delegate2<void, const DynamicsState<LayoutConfigType> *, CollisionData<RigidBodyType>*  > ContactDelegate; ///< This is the delegate type which is used, when a new contact is found then all delegates are invoked in the list.
   #endif

      /** Adds a new ContactDelegate which will be invoked during the solveCollision() part.*/
   void addContactDelegate(const ContactDelegate & cD){
      m_ContactDelegateList.push_back(cD);
   }
   void invokeAll(const DynamicsState<LayoutConfigType> * state, CollisionData<RigidBodyType> *pCollData) const{
      typename std::vector<ContactDelegate>::const_iterator it;
      for(it = m_ContactDelegateList.begin(); it != m_ContactDelegateList.end(); it++){
         (*it)(state, pCollData);
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
  CollisionSolver(std::vector< boost::shared_ptr<RigidBodyType > > & SimBodies,
                  std::vector< boost::shared_ptr<RigidBodyType > > & Bodies);

   ~CollisionSolver();

  void initializeLog(Ogre::Log* pSolverLog);                          ///< Initializes an Ogre::Log.
  void reset();                                                       ///< Resets the whole Solver. This function is called at the start of the simulation.
  void reserveCollisionSetSpace(unsigned int nContacts);              ///< Reserves some space for the collision set.
  void solveCollision(const DynamicsState<LayoutConfigType> * state);    ///< Main routine which solves the collision for all bodies.

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
  std::vector< boost::shared_ptr< RigidBodyType > > & m_SimBodies;       ///< TODO: Add DynamicsSystem pointer, List of all simulated bodies.
  std::vector< boost::shared_ptr< RigidBodyType > > & m_Bodies;          ///< List of all fixed not simulated bodies.

  const DynamicsState<LayoutConfigType> * m_state;

  Collider<LayoutConfigType, CollisionSolver<TCollisionSolverConfig> > m_Collider;                                               ///< The collider class, which is used as a functor which handles the different collisions.
  friend class Collider<LayoutConfigType, CollisionSolver<TCollisionSolverConfig> >;

  Ogre::Log*  m_pSolverLog;  ///< Ogre::Log
  std::stringstream logstream;

  inline void signalContactAdd(CollisionData<RigidBodyType> * pColData); ///< Adds the contact either sends it to the delegate functions or it adds it in the set m_CollisionSet if no delegate has been added.

  void updateAllObjects(const DynamicsState<LayoutConfigType> * state);       ///< General function which does some updateing of all objects. Currently not used.

};
/** @} */


template< typename TCollisionSolverConfig >
CollisionSolver<TCollisionSolverConfig>::CollisionSolver(
                                         std::vector< boost::shared_ptr<RigidBodyType > > & SimBodies,
                                         std::vector< boost::shared_ptr<RigidBodyType > > & Bodies):
m_SimBodies(SimBodies), m_Bodies(Bodies),
m_nSimBodies(SimBodies.size()),m_nDofqObj(NDOFqObj),m_nDofuObj(NDOFuObj),m_state(NULL)
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
void CollisionSolver<TCollisionSolverConfig>::initializeLog( Ogre::Log* pSolverLog )
{
  m_pSolverLog = pSolverLog;
  ASSERTMSG(m_pSolverLog != NULL, "Ogre::Log: NULL!");
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
void CollisionSolver<TCollisionSolverConfig>::solveCollision(const DynamicsState<LayoutConfigType> * state){


  clearCollisionSet();

   #if CoutLevelSolver>0
      CLEARLOG;
      logstream <<" % -> solveCollision(): "<<std::endl;
      LOG(m_pSolverLog);
   #endif


    // Save internal state, this will be given to all signalContactAdd
    m_state  =  state;

    // All objects have been updated...

    //// Do simple collision detection (SimBodies to SimBodies)
    for(int i=0; i < m_SimBodies.size() - 1; i++){
      for(int j=i+1; j< m_SimBodies.size();j++){

        //set the objects in the collider
        m_Collider.initializeBodies(m_SimBodies[i], m_SimBodies[j]);
         // apply visitor
        boost::apply_visitor(m_Collider, m_SimBodies[i]->m_geometry, m_SimBodies[j]->m_geometry);

      }
    }

    // Do simple collision detection (SimBodies to Bodies)
    for(int i=0; i < m_SimBodies.size(); i++){
      for(int j=0; j< m_Bodies.size();j++){
        m_Collider.initializeBodies(m_SimBodies[i], m_Bodies[j]);

        boost::apply_visitor(m_Collider, m_SimBodies[i]->m_geometry, m_Bodies[j]->m_geometry);

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
         m_ContactDelegateList.invokeAll(m_state, m_CollisionSet.back()); // Propagate pointers! they will not be deleted!
      }
  }

template< typename TCollisionSolverConfig >
void CollisionSolver<TCollisionSolverConfig>::updateAllObjects(const DynamicsState<LayoutConfigType> * state)
{

}

#endif
