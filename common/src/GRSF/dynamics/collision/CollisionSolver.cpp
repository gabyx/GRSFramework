// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/dynamics/collision/CollisionSolver.hpp"


CollisionSolver::CollisionSolver(std::shared_ptr< DynamicsSystemType> pDynSys):
    m_simBodies(pDynSys->m_simBodies), m_staticBodies(pDynSys->m_staticBodies),
    m_Collider(&m_collisionSet)
{
}


CollisionSolver::~CollisionSolver() {
    clearCollisionSet();
}



void CollisionSolver::initializeLog( Logging::Log* pSolverLog ) {
    m_pSolverLog = pSolverLog;
    ASSERTMSG(m_pSolverLog != nullptr, "Logging::Log: nullptr!");
}



void CollisionSolver::reset() {
    // Do a Debug check if sizes match!
    ASSERTMSG( m_simBodies.size() != 0, "CollisionSolver:: No Bodies added to the system!");

    removeAllContactDelegates();

    clearCollisionSet();
    m_maxOverlap = 0;

}


void CollisionSolver::clearCollisionSet() {
    m_collisionSet.clear();
}


const typename CollisionSolver::CollisionSetType &
CollisionSolver::getCollisionSetRef()
{
    return m_collisionSet;
}



void CollisionSolver::solveCollision() {


    clearCollisionSet();
    m_maxOverlap = 0;

    LOGSLLEVEL2(m_pSolverLog, "---> solveCollision(): "<<std::endl;)


    // All objects have been updated...

    //// Do simple collision detection (SimBodies to SimBodies)

    if(m_simBodies.size()){
        for(auto bodyIti = m_simBodies.begin(); bodyIti != --m_simBodies.end(); bodyIti++) {
            typename DynamicsSystemType::RigidBodySimContainerType::iterator bodyItj = bodyIti;
            bodyItj++;
            for(; bodyItj != m_simBodies.end(); bodyItj++ ) {

                //check for a collision
                m_Collider.checkCollision((*bodyIti), (*bodyItj));

            }
        }
    }


    // Do simple collision detection (SimBodies to Bodies)
    for(auto bodyIti = m_simBodies.begin(); bodyIti != m_simBodies.end(); bodyIti++) {
        for(auto bodyItk = m_staticBodies.begin(); bodyItk != m_staticBodies.end(); bodyItk ++) {

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

std::string CollisionSolver::getStatsHeader() {
    std::stringstream s;
    s << "MaxOverlap [m]";
    return s.str();
}

void CollisionSolver::signalContactAdd() {

    if(m_collisionSet.size()!=0){

        for( auto colDataPtr : m_collisionSet){

            ASSERTMSG( std::abs(colDataPtr->m_cFrame.m_e_x.dot(colDataPtr->m_cFrame.m_e_y)) < 1e-3 &&
                      std::abs(colDataPtr->m_cFrame.m_e_y.dot(colDataPtr->m_cFrame.m_e_z))< 1e-3, "Vectors not orthogonal");

            LOGSLLEVEL3_CONTACT(m_pSolverLog,"---> Contact Frame: n: " << colDataPtr->m_cFrame.m_e_z.transpose() << std::endl;)

            //Set contact frame point
            colDataPtr->m_cFrame.m_p = colDataPtr->m_pBody[0]->m_r_S + colDataPtr->m_r_SC[0];

            // Calculate some Statistics
            m_maxOverlap = std::max(m_maxOverlap,colDataPtr->m_overlap);

            invokeAllContactDelegates(colDataPtr); // Propagate pointers! they will not be deleted!
        }
    }
}
