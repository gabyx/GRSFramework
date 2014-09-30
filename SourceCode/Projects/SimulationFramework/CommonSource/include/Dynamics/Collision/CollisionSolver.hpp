#ifndef CollisionSolver_hpp
#define CollisionSolver_hpp

#include <fstream>
#include <vector>
#include <memory>
#include <boost/variant.hpp>

#include <memory>


#include "TypeDefs.hpp"
#include "LogDefines.hpp"
#include "AssertionDebug.hpp"

#include DynamicsSystem_INCLUDE_FILE

#include "ContactFrame.hpp"
#include "CollisionData.hpp"
#include "Collider.hpp"
#include "SimpleLogger.hpp"

#include "QuaternionHelpers.hpp"

#include "ContactDelegateSupport.hpp"

/**
* @ingroup Collision
* @brief This is the CollisionSolver class, which basically solves the collision.
*/
/** @{ */

class CollisionSolver : public ContactDelegateSupport{
public:

    DEFINE_COLLISION_SOLVER_CONFIG_TYPES

    typedef typename std::vector< CollisionData * > CollisionSetType;

    /**
    * @brief Constructor for the collision solver.
    * @param SimBodies A reference to the list of all simulated bodies.
    * @param Bodies A reference to the list all not simulated bodies.
    */
    CollisionSolver(std::shared_ptr< DynamicsSystemType> pDynSys);

    ~CollisionSolver();

    void initializeLog(Logging::Log* pSolverLog);                          ///< Initializes an Ogre::Log.
    void reset();                                                       ///< Resets the whole Solver. This function is called at the start of the simulation.
    void solveCollision();    ///< Main routine which solves the collision for all bodies.


    const CollisionSetType & getCollisionSetRef();

    inline void clearCollisionSet();

    std::string getIterationStats();
    std::string getStatsHeader();

protected:


    CollisionSetType m_collisionSet;       ///< This list is only used if no  ContactDelegate is in m_ContactDelegateList, then the contacts are simply added here.


    //Inclusion Solver needs access to everything!
    //class InclusionSolverNT;
    friend class InclusionSolverCO;
    friend class InclusionSolverCONoG;

    typename DynamicsSystemType::RigidBodySimContainerType & m_simBodies;       ///< TODO: Add DynamicsSystem pointer, List of all simulated bodies.
    typename DynamicsSystemType::RigidBodyStaticContainerType & m_staticBodies;          ///< List of all fixed not simulated bodies.

    ColliderBody m_Collider;                                               ///< The collider class, which is used as a functor which handles the different collisions.
    friend class Collider;

    Logging::Log *  m_pSolverLog;

    inline void signalContactAdd(); ///< Sends all contact found by the collider which are in m_collisionSet to the delegate!


    PREC m_maxOverlap;

};
/** @} */


#endif
