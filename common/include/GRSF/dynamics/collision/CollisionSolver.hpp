// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_collision_CollisionSolver_hpp
#define GRSF_dynamics_collision_CollisionSolver_hpp

#include <boost/variant.hpp>
#include <fstream>
#include <memory>
#include <vector>

#include <memory>

#include "GRSF/common/Asserts.hpp"
#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include DynamicsSystem_INCLUDE_FILE

#include "GRSF/common/LinearReusableStorage.hpp"
#include "GRSF/common/SimpleLogger.hpp"
#include "GRSF/dynamics/collision/Collider.hpp"
#include "GRSF/dynamics/collision/CollisionData.hpp"
#include "GRSF/dynamics/collision/ContactFrame.hpp"
#include "GRSF/dynamics/general/QuaternionHelpers.hpp"

#include "GRSF/dynamics/collision/ContactDelegateSupport.hpp"

/**
* @ingroup Collision
* @brief This is the CollisionSolver class, which basically solves the collision.
*/
/** @{ */

class CollisionSolver : public ContactDelegateSupport
{
public:
    DEFINE_COLLISION_SOLVER_CONFIG_TYPES

    using CollisionSetType = LinearReusableStorage<CollisionData>;

    /**
    * @brief Constructor for the collision solver.
    * @param SimBodies A reference to the list of all simulated bodies.
    * @param Bodies A reference to the list all not simulated bodies.
    */
    CollisionSolver(std::shared_ptr<DynamicsSystemType> pDynSys);

    ~CollisionSolver();

    void initializeLog(Logging::Log* pSolverLog);  ///< Initializes an Ogre::Log.
    void reset();           ///< Resets the whole Solver. This function is called at the start of the simulation.
    void solveCollision();  ///< Main routine which solves the collision for all bodies.

    const CollisionSetType& getCollisionSetRef();

    inline void clearCollisionSet();

    std::string getIterationStats();
    std::string getStatsHeader();

protected:
    CollisionSetType m_collisionSet;  ///< This list is only used if no  ContactDelegate is in m_ContactDelegateList,
                                      /// then the contacts are simply added here.

    // Inclusion Solver needs access to everything!
    // class InclusionSolverNT;
    friend class InclusionSolverCO;
    friend class InclusionSolverCONoG;

    typename DynamicsSystemType::RigidBodySimContainerType&
        m_simBodies;  ///< TODO: Add DynamicsSystem pointer, List of all simulated bodies.
    typename DynamicsSystemType::RigidBodyStaticContainerType&
        m_staticBodies;  ///< List of all fixed not simulated bodies.

    ColliderBody<CollisionSetType>
        m_Collider;  ///< The collider class, which is used as a functor which handles the different collisions.
    friend class Collider;

    Logging::Log* m_pSolverLog;

    inline void
    signalContactAdd();  ///< Sends all contact found by the collider which are in m_collisionSet to the delegate!

    PREC m_maxOverlap;
};
/** @} */

#endif
